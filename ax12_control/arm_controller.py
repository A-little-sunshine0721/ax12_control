#!/usr/bin/env python3
"""Dynamixel 다관절 통합 제어기 (AX-12A: Protocol 1.0 + XL330: Protocol 2.0 혼용)
구독
  /ax12/<joint>/cmd      std_msgs/Float64        관절각 [deg]
  /ax12/cmd_deg          sensor_msgs/JointState  name=[관절...], position=[deg...]
  /ax12/joint_cmd        std_msgs/Float64        (호환) joint23
  /ax12/enable, /ax12/<joint>/enable   std_msgs/Bool
발행
  /joint_states          모터별 (rad)
  /ax12/<joint>/debug    [target, ref, q..., e..., corr..., (2축이면 sync)]
  /ax12/<joint>/status   [temp..., volt..., load(AX: %, XL: mA)...]
"""
import os, json, math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from dynamixel_sdk import (PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS,
                           DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD)

MODELS = {12: ('AX-12A', 1), 1190: ('XL330-M077', 2), 1200: ('XL330-M288', 2)}
SPEC = {1: dict(span=300.0, maxraw=1023, kraw=1023 / 300.0, torque=24),
        2: dict(span=360.0, maxraw=4095, kraw=4096 / 360.0, torque=64)}
DEFAULT_CFG = {1: {'zero_deg': 150.0, 'dir': 1, 'min_deg': -140.0, 'max_deg': 140.0},
               2: {'zero_deg': 180.0, 'dir': 1, 'min_deg': -170.0, 'max_deg': 170.0}}
XL_HW = {0: '입력전압', 2: '과열', 3: '엔코더', 4: '전기충격', 5: '과부하'}


class ArmController(Node):
    def __init__(self):
        super().__init__('ax12_arm_controller')
        P = {
            'port': '/dev/ttyACM0', 'baud': 1000000,
            'ids': [1, 2, 3, 4, 5, 6],
            'groups': ['2,3'],
            'joints_file': os.path.expanduser('~/ax12_ws/src/ax12_control/scripts/joints.json'),
            'rate_hz': 50.0,
            'max_vel': 90.0, 'max_acc': 300.0,
            'kp': 0.0, 'ki': 2.0, 'kc': 0.1, 'kci': 1.0,
            'i_limit': 3.0, 'deadband': 0.15, 'deadband_xl': 0.05, 'cmd_margin': 3.0,
            'margin': 1, 'slope': 32, 'punch': 32, 'ax_speed': 600,     # AX 내부 설정
            'xl_profile_vel': 0,
            'xl_pos_p': 800, 'xl_pos_i': 0, 'xl_pos_d': 0,             # XL330 내부 위치 PID
            'gripper_ids': [6], 'gripper_current': 300.0, 'gripper_hold_on_exit': False,                                       # XL 0 = 프로파일 없음
            'fault_err': 30.0, 'fault_time': 1.0, 'comm_fail_sec': 5.0,
            'temp_warn': 60, 'temp_stop': 68,
            'torque_off_on_exit': True,
        }
        for k, v in P.items():
            self.declare_parameter(k, v)
        self.p = p = {k: self.get_parameter(k).value for k in P}
        self.dt = 1.0 / p['rate_hz']
        log = self.get_logger()

        self.port = PortHandler(p['port'])
        self.P = {1: PacketHandler(1.0), 2: PacketHandler(2.0)}
        if not (self.port.openPort() and self.port.setBaudRate(p['baud'])):
            raise RuntimeError(f"포트 열기 실패: {p['port']} (다른 프로그램 사용 중?)")
        self.sync = {1: GroupSyncWrite(self.port, self.P[1], 30, 2),
                     2: GroupSyncWrite(self.port, self.P[2], 116, 4)}

        # ---- 모터 탐색 ----
        self.motor = self.detect(p['ids'])
        missing = [i for i in p['ids'] if i not in self.motor]
        if missing:
            log.warn(f'응답 없는 ID (제외): {missing}')
        if not self.motor:
            raise RuntimeError('응답하는 모터 없음')
        self.ids = sorted(self.motor)
        self.grip = {i for i in p['gripper_ids'] if i in self.motor and self.motor[i]['proto'] == 2}
        if self.grip:
            log.info(f"  그리퍼: IDs {sorted(self.grip)} (전류 제한 {p['gripper_current']:.0f} mA)")
        for i in self.ids:
            log.info(f"  ID {i}: {self.motor[i]['name']} (Protocol {self.motor[i]['proto']}.0)")

        J = {}
        if os.path.exists(p['joints_file']):
            J = {int(k): v for k, v in json.load(open(p['joints_file'])).items()}
        self.cfg = {i: self.check_cfg(i, J.get(i)) for i in self.ids}
        self.db = {i: p['deadband'] if self.motor[i]['proto'] == 1 else p['deadband_xl'] for i in self.ids}

        # ---- 관절 구성 ----
        groups, used = [], set()
        for g in p['groups']:
            gids = [int(x) for x in str(g).replace(' ', '').split(',') if x]
            if len(gids) < 2:
                continue
            if not all(i in self.motor for i in gids) or used & set(gids):
                log.warn(f'그룹 {gids} 구성 불가 → 건너뜀'); continue
            groups.append(gids); used |= set(gids)
        defs = [('joint' + ''.join(map(str, g)), g) for g in groups] + \
               [(f'joint{i}', [i]) for i in self.ids if i not in used]
        self.joints = {}
        for name, ids in defs:
            qmin = max(self.cfg[i]['min_deg'] for i in ids)
            qmax = min(self.cfg[i]['max_deg'] for i in ids)
            cmin, cmax = qmin + p['cmd_margin'], qmax - p['cmd_margin']
            if cmin > cmax:
                cmin = cmax = (qmin + qmax) / 2
            self.joints[name] = dict(name=name, ids=ids, qmin=qmin, qmax=qmax,
                                     cmin=cmin, cmax=cmax, enabled=False, last=None)
        self.motor_state = {}
        for jt in self.joints.values():
            try:
                self.enable_joint(jt)
            except Exception as ex:
                log.error(f"{jt['name']} 활성화 실패: {ex}")

        # ---- 토픽 ----
        for name, jt in self.joints.items():
            self.create_subscription(Float64, f'ax12/{name}/cmd',
                                     lambda m, jt=jt: self.set_target(jt, m.data), 10)
            self.create_subscription(Bool, f'ax12/{name}/enable',
                                     lambda m, jt=jt: self.on_enable(jt, m.data), 10)
            jt['pub_dbg'] = self.create_publisher(Float64MultiArray, f'ax12/{name}/debug', 10)
            jt['pub_st'] = self.create_publisher(Float64MultiArray, f'ax12/{name}/status', 10)
            if all(i in self.grip for i in jt['ids']):
                self.create_subscription(Float64, f'ax12/{name}/current',
                                         lambda m, jt=jt: self.set_current(jt, m.data), 10)
        self.create_subscription(JointState, 'ax12/cmd_deg', self.on_multi, 10)
        self.create_subscription(Bool, 'ax12/enable',
                                 lambda m: [self.on_enable(j, m.data) for j in self.joints.values()], 10)
        if 'joint23' in self.joints:
            self.create_subscription(Float64, 'ax12/joint_cmd',
                                     lambda m: self.set_target(self.joints['joint23'], m.data), 10)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(self.dt, self.loop)
        self.create_timer(1.0, self.log_status)
        for jt in self.joints.values():
            log.info(f"  {jt['name']:<8} IDs {jt['ids']}  명령범위 {jt['cmin']:+.0f}~{jt['cmax']:+.0f}°  "
                     f"토픽 /ax12/{jt['name']}/cmd")

    # ---------- 탐색 / 설정 ----------
    def detect(self, ids):
        found = {}
        self.port.clearPort()
        for proto in (1, 2):
            for i in ids:
                if i in found:
                    continue
                try:
                    m, r, _ = self.P[proto].ping(self.port, i)
                except (IndexError, ValueError):
                    time.sleep(0.02); self.port.clearPort(); continue
                if r == COMM_SUCCESS and m in MODELS and MODELS[m][1] == proto:
                    found[i] = dict(name=MODELS[m][0], proto=proto, **SPEC[proto])
            time.sleep(0.05); self.port.clearPort()
        return found

    def check_cfg(self, i, c):
        pr = self.motor[i]['proto']; span = self.motor[i]['span']
        c = dict(DEFAULT_CFG[pr], **(c or {}))
        bad = c['min_deg'] >= c['max_deg'] or any(
            not 0 <= c['zero_deg'] + c['dir'] * q <= span for q in (c['min_deg'], c['max_deg']))
        if bad:
            self.get_logger().warn(f'ID {i} 설정이 모터 범위(0~{span:.0f}°) 밖 → 기본값 사용')
            return dict(DEFAULT_CFG[pr])
        return c

    def raw2j(self, i, raw):
        c, m = self.cfg[i], self.motor[i]
        return (raw / m['kraw'] - c['zero_deg']) * c['dir']

    def j2raw(self, i, q):
        c, m = self.cfg[i], self.motor[i]
        return max(0, min(m['maxraw'], int(round((c['zero_deg'] + c['dir'] * q) * m['kraw']))))

    def w(self, i, addr, size, val):
        pk = self.P[self.motor[i]['proto']]
        {1: pk.write1ByteTxRx, 2: pk.write2ByteTxRx, 4: pk.write4ByteTxRx}[size](self.port, i, addr, val)

    # ---------- 상태 읽기 ----------
    def read_state(self, i):
        """(q[deg], qd[deg/s], load(AX %/XL mA), fault 문자열|None, volt[V], temp[°C])"""
        m = self.motor[i]; pk = self.P[m['proto']]; dr = self.cfg[i]['dir']
        for _ in range(2):
            try:
                if m['proto'] == 1:
                    d, r, err = pk.readTxRx(self.port, i, 36, 8)
                    if r == COMM_SUCCESS and len(d) == 8:
                        pos, spd, ld = d[0] | d[1] << 8, d[2] | d[3] << 8, d[4] | d[5] << 8
                        sgn = lambda v: -1 if v & 0x400 else 1
                        fault = '과부하' if err & 0x20 else ('과열' if err & 0x04 else None)
                        return (self.raw2j(i, pos), sgn(spd) * (spd & 0x3FF) * 0.666 * dr,
                                sgn(ld) * (ld & 0x3FF) / 10.23 * dr, fault, d[6] / 10.0, d[7])
                else:
                    d, r, err = pk.readTxRx(self.port, i, 126, 21)     # 126 ~ 146
                    if r == COMM_SUCCESS and len(d) == 21:
                        s16 = lambda v: v - 0x10000 if v & 0x8000 else v
                        s32 = lambda v: v - 0x100000000 if v & 0x80000000 else v
                        cur = s16(d[0] | d[1] << 8)
                        vel = s32(d[2] | d[3] << 8 | d[4] << 16 | d[5] << 24)
                        pos = s32(d[6] | d[7] << 8 | d[8] << 16 | d[9] << 24)
                        fault = None
                        if err & 0x80:                                  # Hardware Error 알림
                            hw, _, _ = pk.read1ByteTxRx(self.port, i, 70)
                            fault = ','.join(n for b, n in XL_HW.items() if hw >> b & 1) or 'HW 에러'
                        return (self.raw2j(i, pos), vel * 0.229 * 6.0 * dr, float(cur * dr),
                                fault, (d[18] | d[19] << 8) / 10.0, d[20])
            except (IndexError, ValueError):
                pass
            self.port.clearPort()
        return None

    # ---------- 활성화 / 정지 ----------
    def enable_joint(self, jt):
        p = self.p; qs = []
        for i in jt['ids']:
            pr = self.motor[i]['proto']
            self.w(i, SPEC[pr]['torque'], 1, 0)
            if pr == 1:
                self.w(i, 26, 1, p['margin']); self.w(i, 27, 1, p['margin'])
                self.w(i, 28, 1, p['slope']);  self.w(i, 29, 1, p['slope'])
                self.w(i, 48, 2, p['punch']);  self.w(i, 32, 2, p['ax_speed'])
                self.w(i, 34, 2, 1023)
            else:
                hw, r, _ = self.P[2].read1ByteTxRx(self.port, i, 70)
                if r == COMM_SUCCESS and hw:
                    self.get_logger().warn(f'ID {i} HW 에러({hw:#04x}) → 재부팅으로 초기화')
                    self.P[2].reboot(self.port, i); time.sleep(0.6); self.port.clearPort()
                if i in self.grip:
                    self.w(i, 11, 1, 5)                                # 전류 제한 위치 제어 (그리퍼)
                    self.w(i, 102, 2, int(p['gripper_current']))
                else:
                    self.w(i, 11, 1, 3)                                # Position mode
                self.w(i, 84, 2, int(p['xl_pos_p'])); self.w(i, 82, 2, int(p['xl_pos_i']))
                self.w(i, 80, 2, int(p['xl_pos_d']))                     # 모드 설정 후 게인 적용
                self.w(i, 108, 4, 0); self.w(i, 112, 4, p['xl_profile_vel'])
            s = self.read_state(i)
            if s is None:
                raise RuntimeError(f'ID {i} 읽기 실패')
            qs.append(s[0])
            self.w(i, 30 if pr == 1 else 116, 2 if pr == 1 else 4, self.j2raw(i, s[0]))
        mean = sum(qs) / len(qs)
        jt.update(ref=max(jt['qmin'], min(jt['qmax'], mean)),
                  target=max(jt['cmin'], min(jt['cmax'], mean)), vel=0.0,
                  I={i: 0.0 for i in jt['ids']}, sat={i: 0 for i in jt['ids']}, fault_t=0.0, fail=0)
        for i in jt['ids']:
            self.w(i, SPEC[self.motor[i]['proto']]['torque'], 1, 1)
        jt['enabled'] = True
        self.get_logger().info(f"✅ {jt['name']} 활성화 (현재 {mean:+.1f}°)")

    def disable_joint(self, jt, reason):
        jt['reason'] = reason
        for i in jt['ids']:
            self.w(i, SPEC[self.motor[i]['proto']]['torque'], 1, 0)
        jt['enabled'] = False
        self.get_logger().error(f"⛔ {jt['name']} 정지: {reason} → /ax12/{jt['name']}/enable 에 true")

    # ---------- 콜백 ----------
    def set_target(self, jt, v):
        if not jt['enabled']:
            self.get_logger().warn(f"{jt['name']} 비활성 (원인: {jt.get('reason', '-')}) → "
                                   f"/ax12/{jt['name']}/enable 에 true 후 다시 명령"); return
        if not (jt['cmin'] <= v <= jt['cmax']):
            self.get_logger().warn(f"⚠ {jt['name']}: {v:.1f}°는 범위({jt['cmin']:+.0f}~{jt['cmax']:+.0f}°) 밖 "
                                   f"→ 명령 무시, 현재 목표 {jt['target']:+.1f}° 유지"); return
        if abs(v - jt['target']) > 1.0:
            jt['I'] = {i: 0.0 for i in jt['ids']}
        jt['target'] = v

    def set_current(self, jt, ma):
        ma = max(10.0, min(1750.0, ma))
        for i in jt['ids']:
            self.w(i, 102, 2, int(ma))
        self.p['gripper_current'] = ma
        self.get_logger().info(f"{jt['name']} 잡는 힘(전류 제한) → {ma:.0f} mA")

    def on_enable(self, jt, on):
        if on and not jt['enabled']:
            try:
                self.enable_joint(jt)
            except Exception as ex:
                self.get_logger().error(f"{jt['name']} 활성화 실패: {ex}")
        elif not on and jt['enabled']:
            self.disable_joint(jt, '사용자 요청')

    def on_multi(self, msg):
        if len(msg.name) != len(msg.position):
            self.get_logger().warn('cmd_deg: name과 position 개수가 다름'); return
        for n, v in zip(msg.name, msg.position):
            jt = self.joints.get(n)
            if jt is None:
                self.get_logger().warn(f'알 수 없는 관절: {n} (사용 가능: {list(self.joints)})')
            else:
                self.set_target(jt, v)

    # ---------- 궤적 ----------
    def update_ref(self, jt):
        d = jt['target'] - jt['ref']
        if abs(d) < 1e-6 and abs(jt['vel']) < 1e-6:
            return False
        vmax, amax, dt = self.p['max_vel'], self.p['max_acc'], self.dt
        vl = math.copysign(min(vmax, math.sqrt(2 * amax * abs(d))), d)
        v = jt['vel']
        jt['vel'] = min(vl, v + amax * dt) if v < vl else max(vl, v - amax * dt)
        s = jt['vel'] * dt
        if abs(s) >= abs(d):
            jt['ref'], jt['vel'] = jt['target'], 0.0
        else:
            jt['ref'] += s
        return True

    # ---------- 제어 루프 ----------
    def loop(self):
        p = self.p
        for sw in self.sync.values():
            sw.clearParam()
        cnt = {1: 0, 2: 0}
        for jt in self.joints.values():
            if not jt['enabled']:
                continue
            ids = jt['ids']
            moving = self.update_ref(jt)
            st = {i: self.read_state(i) for i in ids}
            if any(v is None for v in st.values()):
                jt['fail'] += 1
                self.get_logger().warn(f"{jt['name']} 통신 실패 (재시도 중)", throttle_duration_sec=2.0)
                if jt['fail'] * self.dt > p['comm_fail_sec']:
                    self.disable_joint(jt, f"통신 실패 {p['comm_fail_sec']:.0f}초 지속")
                continue
            jt['fail'] = 0
            self.motor_state.update(st)
            q = {i: st[i][0] for i in ids}
            e = {i: jt['ref'] - q[i] for i in ids}

            stop = None
            for i in ids:
                fault, T = st[i][3], st[i][5]
                if fault:
                    stop = f'ID {i} {fault}'
                elif T >= p['temp_stop']:
                    stop = f'ID {i} 과열 {T}°C'
                elif T >= p['temp_warn']:
                    self.get_logger().warn(f"{jt['name']} ID {i} 온도 {T}°C", throttle_duration_sec=5.0)
            emax = max(abs(v) for v in e.values())
            grip = all(i in self.grip for i in ids)
            jt['fault_t'] = jt['fault_t'] + self.dt if (emax > p['fault_err'] and not grip) else 0.0
            if stop:
                self.disable_joint(jt, stop); continue
            if jt['fault_t'] > p['fault_time']:
                hold = max(jt['qmin'], min(jt['qmax'], sum(q.values()) / len(q)))
                self.get_logger().warn(f"⚠ {jt['name']} 추종 오차 {emax:.1f}° 지속 (걸림 의심) "
                                       f"→ 현재 위치 {hold:+.1f}°에서 정지·유지")
                jt['target'] = jt['ref'] = hold
                jt['vel'], jt['fault_t'] = 0.0, 0.0
                jt['I'] = {i: 0.0 for i in ids}
                e = {i: jt['ref'] - q[i] for i in ids}

            cp = {i: sum(e[i] - e[j] for j in ids if j != i) for i in ids}
            if not moving and not grip:
                for i in ids:
                    ec = e[i] + p['kci'] * cp[i]
                    if abs(ec) <= self.db[i] or jt['sat'][i] * ec > 0:
                        ec = 0.0
                    jt['I'][i] = max(-p['i_limit'], min(p['i_limit'], jt['I'][i] + p['ki'] * ec * self.dt))
            corr = {i: p['kp'] * e[i] + jt['I'][i] + p['kc'] * cp[i] for i in ids}

            for i in ids:
                c, pr = self.cfg[i], self.motor[i]['proto']
                ur = jt['ref'] + corr[i]
                jt['sat'][i] = 1 if ur > c['max_deg'] else (-1 if ur < c['min_deg'] else 0)
                raw = self.j2raw(i, max(c['min_deg'], min(c['max_deg'], ur)))
                if pr == 1:
                    self.sync[1].addParam(i, [DXL_LOBYTE(raw), DXL_HIBYTE(raw)])
                else:
                    self.sync[2].addParam(i, [DXL_LOBYTE(DXL_LOWORD(raw)), DXL_HIBYTE(DXL_LOWORD(raw)),
                                              DXL_LOBYTE(DXL_HIWORD(raw)), DXL_HIBYTE(DXL_HIWORD(raw))])
                cnt[pr] += 1

            dbg = [jt['target'], jt['ref']] + [q[i] for i in ids] + [e[i] for i in ids] + [corr[i] for i in ids]
            if len(ids) == 2:
                dbg.append(q[ids[0]] - q[ids[1]])
            jt['pub_dbg'].publish(Float64MultiArray(data=[float(x) for x in dbg]))
            jt['pub_st'].publish(Float64MultiArray(data=[float(st[i][5]) for i in ids] +
                                 [st[i][4] for i in ids] + [float(st[i][2]) for i in ids]))
            jt['last'] = (q, e, st)
        for pr, n in cnt.items():
            if n:
                self.sync[pr].txPacket()

        if self.motor_state:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            ks = sorted(self.motor_state)
            js.name = [f'id{i}' for i in ks]
            js.position = [math.radians(self.motor_state[i][0]) for i in ks]
            js.velocity = [math.radians(self.motor_state[i][1]) for i in ks]
            js.effort = [float(self.motor_state[i][2]) for i in ks]
            self.pub_js.publish(js)

    def log_status(self):
        parts = []
        for jt in self.joints.values():
            if not jt['enabled'] or not jt['last']:
                parts.append(f"{jt['name']} OFF"); continue
            q, e, st = jt['last']
            s = f"{jt['name']} {jt['target']:+.1f}°→" + " ".join(
                f"[{i}] {q[i]:+.2f}(e{e[i]:+.2f},{st[i][5]}°C)" for i in jt['ids'])
            if len(jt['ids']) == 2:
                a, b = jt['ids']
                s += f" sync {q[a]-q[b]:+.2f}"
            parts.append(s)
        self.get_logger().info(" | ".join(parts))

    def destroy_node(self):
        if self.p.get('torque_off_on_exit', True):
            for i in self.ids:
                if i in self.grip and self.p.get('gripper_hold_on_exit'):
                    continue
                try:
                    self.w(i, SPEC[self.motor[i]['proto']]['torque'], 1, 0)
                except Exception:
                    pass
        self.port.closePort()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = ArmController()
    except Exception as ex:
        print(f'[arm_controller] 시작 실패: {ex}')
        rclpy.try_shutdown(); return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
