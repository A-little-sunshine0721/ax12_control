#!/usr/bin/env python3
"""키보드로 관절 조작 (arm_controller 실행 중에 사용)
  1~9 : ID 선택 (2·3은 joint23으로 함께)   ← → : −/+ step   ↑ ↓ : step 크기
  0   : 선택 관절 0°        z : 전체 0°        e : 선택 관절 토크 ON/OFF
  s   : 현재 위치를 원위치로 저장   h : 전체 원위치 복귀   g : 선택 관절 원위치 복귀
  l   : 저장된 원위치 보기   q : 종료
"""
import sys, os, re, time, json, select, termios, tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Float64MultiArray

STEPS = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0]
HELP = ("\n=== Dynamixel 키보드 조작 ===\n"
        " 1~9: ID 선택 | ←/→: 이동 | ↑/↓: 스텝 | 0: 선택 0° | z: 전체 0° | e: 토크 ON/OFF\n"
        " s: 원위치 저장 | h: 전체 원위치 복귀 | g: 선택 관절 원위치 | l: 원위치 보기 | q: 종료\n")


class KeyTeleop(Node):
    def __init__(self):
        super().__init__('ax12_key_teleop')
        self.declare_parameter('groups', ['2,3'])
        self.declare_parameter('home_file',
                               os.path.expanduser('~/ax12_ws/src/ax12_control/scripts/home.json'))
        self.groups = [[int(x) for x in str(g).replace(' ', '').split(',') if x]
                       for g in self.get_parameter('groups').value]
        self.home_file = self.get_parameter('home_file').value
        self.home = self.load_home()
        self.pubs, self.en_pubs, self.subs = {}, {}, {}
        self.state, self.tgt, self.t_pub, self.enabled = {}, {}, {}, {}
        self.sel, self.step_i, self.quit = None, 3, False
        self.msg = ('원위치 불러옴: ' + self.home_str()) if self.home else '숫자키로 ID 선택 (원위치 없음: s로 저장)'
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        print(HELP)
        self.create_timer(0.02, self.poll)
        self.create_timer(0.2, self.draw)
        self.create_timer(1.0, self.discover)

    # ---------- 원위치 ----------
    def load_home(self):
        try:
            return {k: float(v) for k, v in json.load(open(self.home_file)).items()}
        except Exception:
            return {}

    def home_str(self):
        return ', '.join(f'{j} {v:+.1f}°' for j, v in sorted(self.home.items()))

    def save_home(self):
        now, saved, skipped = time.time(), {}, []
        for j in sorted(self.available()):
            s = self.state.get(j)
            if s and now - s['t'] < 1.0 and s['q']:
                saved[j] = round(sum(s['q']) / len(s['q']), 2)
            else:
                skipped.append(j)
        if not saved:
            self.msg = '저장할 관절 없음 (arm_controller 실행·토크 ON 상태 확인)'; return
        self.home = saved
        os.makedirs(os.path.dirname(self.home_file), exist_ok=True)
        json.dump(self.home, open(self.home_file, 'w'), indent=2)
        self.msg = '원위치 저장: ' + self.home_str() + (f' | 제외(OFF/응답없음): {skipped}' if skipped else '')

    def go_home(self, joints):
        if not self.home:
            self.msg = '저장된 원위치 없음 → s로 먼저 저장'; return
        done = []
        for j in joints:
            if j in self.home and j in self.available():
                self.ensure(j); self.send(j, self.home[j]); done.append(j)
        self.msg = ('원위치 복귀: ' + ', '.join(f'{j} {self.home[j]:+.1f}°' for j in done)) if done \
            else '복귀할 관절 없음'

    # ---------- 관절 / 토픽 ----------
    def joint_of(self, i):
        for g in self.groups:
            if i in g:
                return 'joint' + ''.join(map(str, g))
        return f'joint{i}'

    def available(self):
        out = set()
        for n, _ in self.get_topic_names_and_types():
            m = re.match(r'^/ax12/(joint\w+)/cmd$', n)
            if m:
                out.add(m.group(1))
        return out

    def ensure(self, j):
        if j in self.pubs:
            return
        self.pubs[j] = self.create_publisher(Float64, f'ax12/{j}/cmd', 10)
        self.en_pubs[j] = self.create_publisher(Bool, f'ax12/{j}/enable', 10)
        self.subs[j] = self.create_subscription(
            Float64MultiArray, f'ax12/{j}/debug', lambda m, j=j: self.on_dbg(j, m), 10)
        self.enabled.setdefault(j, True)

    def discover(self):
        for j in self.available():
            self.ensure(j)

    def on_dbg(self, j, m):
        d = list(m.data)
        n = (len(d) - 2) // 3
        self.state[j] = dict(target=d[0], q=d[2:2 + n], t=time.time())
        if time.time() - self.t_pub.get(j, 0) > 0.3:
            self.tgt[j] = d[0]

    def send(self, j, v):
        self.pubs[j].publish(Float64(data=float(v)))
        self.tgt[j] = v
        self.t_pub[j] = time.time()

    # ---------- 키 입력 ----------
    def poll(self):
        if not select.select([sys.stdin], [], [], 0)[0]:
            return
        data = os.read(self.fd, 64).decode(errors='ignore')
        k = 0
        while k < len(data):
            if data.startswith('\x1b[', k) and k + 2 < len(data):
                self.arrow(data[k + 2]); k += 3
            else:
                self.key(data[k]); k += 1

    def key(self, c):
        if c in '123456789':
            i = int(c); j = self.joint_of(i)
            if j not in self.available():
                self.msg = f'ID {i} ({j}) 없음 — arm_controller에 등록됐는지 확인'; return
            self.ensure(j); self.sel = (i, j)
            self.msg = f'ID {i} 선택' + (f' → {j} 묶음으로 함께 움직임' if j != f'joint{i}' else '')
        elif c == '0' and self.sel:
            self.send(self.sel[1], 0.0); self.msg = f'{self.sel[1]} → 0°'
        elif c in 'zZ':
            for j in self.available():
                self.ensure(j); self.send(j, 0.0)
            self.msg = '전체 0°'
        elif c in 'eE' and self.sel:
            j = self.sel[1]; self.enabled[j] = not self.enabled.get(j, True)
            self.en_pubs[j].publish(Bool(data=self.enabled[j]))
            self.msg = f"{j} 토크 {'ON' if self.enabled[j] else 'OFF'}"
        elif c in 'sS':
            self.save_home()
        elif c in 'hH':
            self.go_home(sorted(self.available()))
        elif c in 'gG':
            if self.sel:
                self.go_home([self.sel[1]])
            else:
                self.msg = '먼저 숫자키로 ID를 선택하세요'
        elif c in 'lL':
            self.msg = ('원위치: ' + self.home_str()) if self.home else '저장된 원위치 없음'
        elif c in 'qQ':
            self.quit = True

    def arrow(self, a):
        if a == 'A':
            self.step_i = min(len(STEPS) - 1, self.step_i + 1); self.msg = f'step {STEPS[self.step_i]}°'
        elif a == 'B':
            self.step_i = max(0, self.step_i - 1); self.msg = f'step {STEPS[self.step_i]}°'
        elif a in 'CD':
            if not self.sel:
                self.msg = '먼저 숫자키로 ID를 선택하세요'; return
            j = self.sel[1]
            if j not in self.tgt:
                self.msg = f'{j} 상태 수신 대기 중… (arm_controller 실행 중인지 확인)'; return
            d = STEPS[self.step_i] * (1 if a == 'C' else -1)
            self.send(j, self.tgt[j] + d)
            self.msg = f'{j} {"+" if d > 0 else ""}{d}°'

    # ---------- 표시 ----------
    def draw(self):
        if self.sel:
            i, j = self.sel
            s = self.state.get(j)
            tg = f"{self.tgt[j]:+7.1f}°" if j in self.tgt else '   -   '
            cur = ' '.join(f'{x:+7.2f}°' for x in s['q']) if s else '-'
            hm = f" | 원위치 {self.home[j]:+.1f}°" if j in self.home else ''
            head = f"[ID {i} | {j}] 목표 {tg} | 현재 {cur}{hm}"
        else:
            head = '[선택 없음]'
        line = f"{head} | step {STEPS[self.step_i]}° | {self.msg}"
        sys.stdout.write('\r' + line[:180].ljust(180)); sys.stdout.flush()

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        print('\n종료')


def main():
    if not sys.stdin.isatty():
        print('터미널에서 직접 실행하세요 (ros2 run ax12_control key_teleop)'); return
    rclpy.init()
    node = KeyTeleop()
    try:
        while rclpy.ok() and not node.quit:
            rclpy.spin_once(node, timeout_sec=0.02)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
