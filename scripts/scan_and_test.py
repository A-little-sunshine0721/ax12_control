#!/usr/bin/env python3
"""ID 스캔 → 선택 → 각도 스윕 테스트 (AX-12A / XL330 자동 구분)
  python3 scan_and_test.py            # 스캔 후 선택
  python3 scan_and_test.py --all      # 전체 자동
  python3 scan_and_test.py --scan     # 스캔만
  python3 scan_and_test.py --step 15  # 간격 변경
"""
import sys, time, argparse
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="/dev/ttyACM0")
ap.add_argument("--baud", type=int, default=1000000)
ap.add_argument("--step", type=int, default=30)
ap.add_argument("--tol", type=float, default=3.0)
ap.add_argument("--all", action="store_true")
ap.add_argument("--scan", action="store_true")
args = ap.parse_args()

MODELS = {
    12:   dict(name="AX-12A",     proto=1, res=1024, span=300),
    1190: dict(name="XL330-M077", proto=2, res=4096, span=360),
    1200: dict(name="XL330-M288", proto=2, res=4096, span=360),
}
AX = dict(TORQUE=24, GOAL=30, SPEED=32, POS=36, MOVING=46, CW=6, CCW=8)
XL = dict(TORQUE=64, MODE=11, PVEL=112, GOAL=116, POS=132, MOVING=122,
          HWERR=70, MAXP=48, MINP=52)
AX_ERR = ["전압", "각도제한", "과열", "범위", "체크섬", "과부하", "명령"]

port = PortHandler(args.port)
if not (port.openPort() and port.setBaudRate(args.baud)):
    sys.exit(f"포트 열기 실패: {args.port} (다른 프로그램이 사용 중인지 확인)")
P = {1: PacketHandler(1.0), 2: PacketHandler(2.0)}

# ---------- 1. 스캔 ----------
print(f"[1] 스캔: {args.port} @ {args.baud} bps")

def flush():
    time.sleep(0.05)
    port.clearPort()

def safe_ping(k, i):
    try:
        return k.ping(port, i)
    except (IndexError, ValueError):
        flush()
        return 0, -1, 0

motors = []
flush()
for i in range(254):                                   # Protocol 1.0 먼저
    model, r, _ = safe_ping(P[1], i)
    if r == COMM_SUCCESS and model in MODELS and MODELS[model]["proto"] == 1:
        motors.append(dict(id=i, **MODELS[model]))
flush()
try:                                                   # Protocol 2.0 broadcast
    res2, r = P[2].broadcastPing(port)
except (IndexError, ValueError):
    res2, r = {}, -1
flush()
if r == COMM_SUCCESS:
    for i, (model, _) in res2.items():
        if model in MODELS and MODELS[model]["proto"] == 2:
            motors.append(dict(id=i, **MODELS[model]))
motors.sort(key=lambda m: m["id"])

if not motors:
    port.closePort(); sys.exit("발견된 모터 없음 (전원/케이블/보레이트 확인)")
for m in motors:
    print(f"    ID {m['id']:3d}  {m['name']:<11} (Protocol {m['proto']}.0)")
if args.scan:
    port.closePort(); sys.exit(0)

# ---------- 2. 선택 ----------
if args.all:
    sel = motors
else:
    s = input("\n[2] 테스트할 ID (공백 구분, Enter = 전체): ").strip()
    ids = {int(x) for x in s.split()} if s else {m["id"] for m in motors}
    sel = [m for m in motors if m["id"] in ids]
    if not sel:
        port.closePort(); sys.exit("선택된 모터 없음")
print("    ⚠ 0°~최대각까지 회전합니다. 조립 상태라면 간섭 여부를 확인하세요.")
if not args.all and input("    시작할까요? [Y/n] ").strip().lower() == "n":
    port.closePort(); sys.exit(0)

# ---------- 제어 함수 ----------
def pk(m): return P[m["proto"]]
def deg2raw(m, d): return min(m["res"] - 1, max(0, round(d * m["res"] / m["span"])))
def raw2deg(m, r): return r * m["span"] / m["res"]

def setup(m):
    k, i = pk(m), m["id"]
    if m["proto"] == 1:
        cw, _, _ = k.read2ByteTxRx(port, i, AX["CW"])
        ccw, _, _ = k.read2ByteTxRx(port, i, AX["CCW"])
        m["lim"] = (raw2deg(m, cw), raw2deg(m, ccw))
        k.write2ByteTxRx(port, i, AX["SPEED"], 150)
        k.write1ByteTxRx(port, i, AX["TORQUE"], 1)
    else:
        k.write1ByteTxRx(port, i, XL["TORQUE"], 0)
        k.write1ByteTxRx(port, i, XL["MODE"], 3)
        mn, _, _ = k.read4ByteTxRx(port, i, XL["MINP"])
        mx, _, _ = k.read4ByteTxRx(port, i, XL["MAXP"])
        m["lim"] = (raw2deg(m, mn), raw2deg(m, mx))
        k.write4ByteTxRx(port, i, XL["PVEL"], 70)
        k.write1ByteTxRx(port, i, XL["TORQUE"], 1)
    lo, hi = m["lim"]
    warn = "" if lo <= 0.5 and hi >= m["span"] - 1 else "  ⚠ 제한 밖 각도는 FAIL 정상"
    print(f"    ID {m['id']:3d} {m['name']:<11} 위치제한 {lo:.0f}°~{hi:.0f}°{warn}")

def goal(m, d):
    if m["proto"] == 1: pk(m).write2ByteTxRx(port, m["id"], AX["GOAL"], deg2raw(m, d))
    else:               pk(m).write4ByteTxRx(port, m["id"], XL["GOAL"], deg2raw(m, d))

def read(m):
    k, i = pk(m), m["id"]
    if m["proto"] == 1:
        raw, r, err = k.read2ByteTxRx(port, i, AX["POS"])
        e = ",".join(AX_ERR[b] for b in range(7) if err >> b & 1)
    else:
        raw, r, _ = k.read4ByteTxRx(port, i, XL["POS"])
        hw, _, _ = k.read1ByteTxRx(port, i, XL["HWERR"])
        e = f"HW:{hw:#04x}" if hw else ""
    return (raw2deg(m, raw) if r == COMM_SUCCESS else float("nan")), e

def wait_stop(m, timeout=6.0):
    addr = AX["MOVING"] if m["proto"] == 1 else XL["MOVING"]
    t0 = time.time(); time.sleep(0.15)
    while time.time() - t0 < timeout:
        v, r, _ = pk(m).read1ByteTxRx(port, m["id"], addr)
        if r == COMM_SUCCESS and v == 0:
            return True
        time.sleep(0.05)
    return False

def torque_off():
    for m in sel:
        a = AX["TORQUE"] if m["proto"] == 1 else XL["TORQUE"]
        pk(m).write1ByteTxRx(port, m["id"], a, 0)

# ---------- 3. 초기화 & 스윕 ----------
print("\n[3] 초기화")
for m in sel:
    setup(m)

max_span = max(m["span"] for m in sel)
up = list(range(0, max_span + 1, args.step))
targets = up + up[-2::-1]
results = {m["id"]: [] for m in sel}

print(f"\n[4] 스윕 (step {args.step}°, 허용오차 ±{args.tol}°)")
print(f"{'target':>7} | " + " | ".join(f"ID{m['id']:<3}{'':>15}" for m in sel))
try:
    for tgt in targets:
        active = [m for m in sel if tgt <= m["span"]]
        for m in active:
            goal(m, tgt)
        cells = []
        for m in sel:
            if m not in active:
                cells.append(f"{'-':^19}"); continue
            stopped = wait_stop(m)
            act, e = read(m)
            ok = stopped and abs(act - tgt) <= args.tol and not e
            results[m["id"]].append((tgt, act, ok))
            tag = "PASS" if ok else ("TIME" if not stopped else "FAIL")
            cells.append(f"{act:6.1f}° {act-tgt:+5.1f} {tag}" + (f"[{e}]" if e else ""))
        print(f"{tgt:6d}° | " + " | ".join(cells))
except KeyboardInterrupt:
    print("\n중단")
finally:
    torque_off()
    port.closePort()

# ---------- 5. 요약 ----------
print("\n[5] 요약")
for m in sel:
    r = results[m["id"]]
    if not r:
        continue
    fails = sorted({t for t, _, ok in r if not ok})
    vals = [abs(a - t) for t, a, _ in r if a == a]
    maxerr = max(vals) if vals else float("nan")
    status = "✅ 전 구간 정상" if not fails else f"❌ 실패 각도 {fails}"
    print(f"    ID {m['id']:3d} {m['name']:<11} {len(r)-len(fails)}/{len(r)} PASS, "
          f"최대오차 {maxerr:.2f}°  {status}")
