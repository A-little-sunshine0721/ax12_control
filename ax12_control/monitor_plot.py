#!/usr/bin/env python3
"""모터 상태 실시간 그래프 (arm_controller 실행 중에 사용)
  위치·목표 / 오차 / 부하(AX %) / 전류(XL mA) / 온도 / 전압
  ros2 run ax12_control monitor_plot --ros-args -p window_sec:=30.0
"""
import re, time, threading
from collections import deque, defaultdict
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PANELS = [('pos', 'Position [deg]  (solid: actual, dashed: target)'), ('err', 'Error [deg]'),
          ('load', 'Load [%]  (AX-12A)'), ('cur', 'Current [mA]  (XL330)'),
          ('temp', 'Temperature [degC]'), ('volt', 'Voltage [V]')]


class Monitor(Node):
    def __init__(self):
        super().__init__('ax12_monitor_plot')
        self.declare_parameter('groups', ['2,3'])           # arm_controller와 동일하게
        self.declare_parameter('xl_ids', [6])               # status 3번째 값이 전류(mA)인 모터
        self.declare_parameter('window_sec', 20.0)
        self.groups = [[int(x) for x in str(g).replace(' ', '').split(',') if x]
                       for g in self.get_parameter('groups').value]
        self.xl = set(self.get_parameter('xl_ids').value)
        self.win = float(self.get_parameter('window_sec').value)
        self.t0 = time.time()
        self.lock = threading.Lock()
        mk = lambda: deque(maxlen=6000)
        self.d = defaultdict(lambda: defaultdict(mk))       # d[ID][항목] = [(t, v)]
        self.tg = defaultdict(mk)                           # tg[관절] = [(t, target)]
        self.subscribed = set()
        self.create_timer(1.0, self.discover)

    def ids_of(self, j):
        for g in self.groups:
            if j == 'joint' + ''.join(map(str, g)):
                return g
        m = re.match(r'^joint(\d+)$', j)
        return [int(m.group(1))] if m else []

    def discover(self):
        for n, _ in self.get_topic_names_and_types():
            m = re.match(r'^/ax12/(joint\w+)/(debug|status)$', n)
            if not m or n in self.subscribed:
                continue
            j, kind = m.groups()
            ids = self.ids_of(j)
            if not ids:
                continue
            cb = (lambda msg, j=j, ids=ids: self.on_dbg(j, ids, msg)) if kind == 'debug' \
                else (lambda msg, ids=ids: self.on_st(ids, msg))
            self.create_subscription(Float64MultiArray, n, cb, 10)
            self.subscribed.add(n)
            self.get_logger().info(f'구독: {n}')

    def on_dbg(self, j, ids, msg):           # [target, ref, q..., e..., corr..., (sync)]
        d, n = list(msg.data), len(ids)
        if len(d) < 2 + 3 * n:
            return
        t = time.time() - self.t0
        with self.lock:
            self.tg[j].append((t, d[0]))
            for k, i in enumerate(ids):
                self.d[i]['pos'].append((t, d[2 + k]))
                self.d[i]['err'].append((t, d[2 + n + k]))

    def on_st(self, ids, msg):               # [temp..., volt..., load(AX %)/current(XL mA)...]
        d, n = list(msg.data), len(ids)
        if len(d) < 3 * n:
            return
        t = time.time() - self.t0
        with self.lock:
            for k, i in enumerate(ids):
                self.d[i]['temp'].append((t, d[k]))
                self.d[i]['volt'].append((t, d[n + k]))
                self.d[i]['cur' if i in self.xl else 'load'].append((t, d[2 * n + k]))


def main():
    rclpy.init()
    node = Monitor()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, axes = plt.subplots(3, 2, figsize=(14, 9), sharex=True)
    axes = axes.flatten()
    fig.canvas.manager.set_window_title('Dynamixel monitor')
    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    col = lambda i: colors[(i - 1) % len(colors)]
    for ax, (_, title) in zip(axes, PANELS):
        ax.set_title(title, fontsize=10); ax.grid(alpha=0.3)
    axes[1].axhline(0, color='k', lw=0.6)
    axes[4].set_xlabel('time [s]'); axes[5].set_xlabel('time [s]')
    lines = {}

    def put(ax, key, label, pts, **kw):
        if not pts:
            return
        if key not in lines:
            lines[key], = ax.plot([], [], label=label, **kw)
            ax.legend(loc='upper left', fontsize=8, ncol=4)
        ts, vs = zip(*pts)
        lines[key].set_data(ts, vs)

    def update(_):
        now = time.time() - node.t0
        tmin = now - node.win
        with node.lock:
            snap = {i: {k: [p for p in v if p[0] >= tmin] for k, v in dd.items()}
                    for i, dd in node.d.items()}
            tsnap = {j: [p for p in v if p[0] >= tmin] for j, v in node.tg.items()}
        for idx, (key, _) in enumerate(PANELS):
            ax = axes[idx]
            for i in sorted(snap):
                put(ax, (key, i), f'ID {i}', snap[i].get(key, []), color=col(i), lw=1.5)
            if key == 'pos':
                for j, pts in tsnap.items():
                    ids = node.ids_of(j)
                    put(ax, ('tgt', j), f'{j} target', pts, color=col(ids[0]), lw=1, ls='--', alpha=0.7)
            ax.relim(); ax.autoscale_view(scalex=False)
        axes[0].set_xlim(tmin, now)
        return []

    ani = FuncAnimation(fig, update, interval=200, cache_frame_data=False)  # noqa: F841
    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
