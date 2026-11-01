#!/usr/bin/env python3
"""관절 설정(방향 반전 + 상하한) 기반 제어
  python3 joint_tool.py read                # 토크 OFF, 손으로 돌리며 각도 확인 (한계 찾기)
  python3 joint_tool.py limits              # joints.json 상하한을 모터 EEPROM에 저장
  python3 joint_tool.py move --deg 30       # 모든 관절을 관절각 30°로 (방향 자동 반영)
  python3 joint_tool.py move --deg 30 --ids 2
  python3 joint_tool.py show                # 설정과 현재 상태 확인
"""
import os, sys, json, time, argparse
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, DXL_LOBYTE, DXL_HIBYTE
from comp_tool import apply_all, rd, wait_stop

STEP = 300 / 1023
JCFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "joints.json")
DEFAULT = {   # 실제 기구에 맞게 수정 → 파일로 저장됨
    "2": {"zero_deg": 0.0,   "dir":  1, "min_deg": 0.0, "max_deg": 180.0},
    "3": {"zero_deg": 300.0, "dir": -1, "min_deg": 0.0, "max_deg": 180.0},
}

def load_joints():
    if not os.path.exists(JCFG):
        json.dump(DEFAULT, open(JCFG, "w"), indent=2)
        print(f"※ 기본 설정 생성: {JCFG}  (값을 확인/수정하세요)")
    cfg = {int(i): v for i, v in json.load(open(JCFG)).items()}
    for i, j in cfg.items():                      # 설정 유효성 검사
        for q in (j["min_deg"], j["max_deg"]):
            m = j["zero_deg"] + j["dir"] * q
            if not (0 <= m <= 300):
                sys.exit(f"❌ ID {i}: 관절 {q}° → 모터 {m:.1f}° (0~300° 범위 밖). joints.json 수정 필요")
        if j["min_deg"] >= j["max_deg"]:
            sys.exit(f"❌ ID {i}: min_deg < max_deg 이어야 함")
    return cfg

def j2m(j, q):       # 관절각 → 모터각 (상하한 clamp 포함)
    q = max(j["min_deg"], min(j["max_deg"], q))
    return j["zero_deg"] + j["dir"] * q, q

def m2j(j, m):       # 모터각 → 관절각
    return (m - j["zero_deg"]) * j["dir"]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["read", "limits", "move", "show"])
    ap.add_argument("--deg", type=float)
    ap.add_argument("--ids", type=int, nargs="+")
    ap.add_argument("--speed", type=int, default=100)
    ap.add_argument("--release", action="store_true")
    a = ap.parse_args()

    J = load_joints()
    ids = a.ids or sorted(J)
    port, k = PortHandler("/dev/ttyACM0"), PacketHandler(1.0)
    assert port.openPort() and port.setBaudRate(1000000), "포트 열기 실패"
    try:
        if a.cmd == "read":
            for i in ids: k.write1ByteTxRx(port, i, 24, 0)
            print("토크 OFF — 손으로 돌려서 원하는 한계 위치를 확인하세요 (Ctrl+C 종료)\n")
            while True:
                s = []
                for i in ids:
                    p = rd(port, k, i, 36)
                    s.append(f"ID{i}: 모터 {p*STEP:6.1f}° | 관절 {m2j(J[i], p*STEP):+7.1f}°"
                             if p is not None else f"ID{i}: 읽기 실패")
                print("\r" + "   ".join(s), end="", flush=True)
                time.sleep(0.1)

        elif a.cmd == "limits":
            print("[EEPROM Angle Limit 저장]")
            for i in ids:
                lo_m, _ = j2m(J[i], J[i]["min_deg"]); hi_m, _ = j2m(J[i], J[i]["max_deg"])
                cw, ccw = sorted((int(round(lo_m / STEP)), int(round(hi_m / STEP))))
                if cw == ccw:
                    sys.exit(f"❌ ID {i}: 범위가 0 (바퀴 모드가 됨)")
                k.write1ByteTxRx(port, i, 24, 0)            # EEPROM 쓰기는 토크 OFF
                k.write2ByteTxRx(port, i, 6, cw); k.write2ByteTxRx(port, i, 8, ccw)
                rcw, rccw = rd(port, k, i, 6), rd(port, k, i, 8)
                ok = (rcw, rccw) == (cw, ccw)
                print(f"  ID {i}: 모터 {cw*STEP:.1f}° ~ {ccw*STEP:.1f}° "
                      f"(관절 {J[i]['min_deg']}° ~ {J[i]['max_deg']}°) {'✅' if ok else '❌'}")

        elif a.cmd == "move":
            if a.deg is None:
                sys.exit("--deg 필요")
            apply_all(port, k, ids, verbose=False)          # 저장된 compliance 적용
            sync = GroupSyncWrite(port, k, 30, 2)
            for i in ids:
                m, q = j2m(J[i], a.deg)
                if q != a.deg:
                    print(f"  ⚠ ID {i}: {a.deg}° → 범위 제한으로 {q}°")
                raw = int(round(m / STEP))
                k.write2ByteTxRx(port, i, 32, a.speed)
                k.write1ByteTxRx(port, i, 24, 1)
                sync.addParam(i, [DXL_LOBYTE(raw), DXL_HIBYTE(raw)])
            sync.txPacket()                                 # 동시 출발
            for i in ids: wait_stop(port, k, i)
            print(f"[이동] 관절각 {a.deg}°")
            for i in ids:
                p = rd(port, k, i, 36)
                if p is not None:
                    print(f"  ID {i}: 관절 {m2j(J[i], p*STEP):+7.2f}° (모터 {p*STEP:6.2f}°)")
                if a.release: k.write1ByteTxRx(port, i, 24, 0)
            print("토크", "OFF" if a.release else "ON 유지")

        elif a.cmd == "show":
            print(f"설정 파일: {JCFG}\n")
            print(f"{'ID':>3} {'dir':>4} {'zero':>7} {'관절범위':>14} | {'EEPROM 한계(모터)':>18} | {'현재 관절각':>10}")
            for i in ids:
                j = J[i]; cw, ccw, p = rd(port, k, i, 6), rd(port, k, i, 8), rd(port, k, i, 36)
                lim = f"{cw*STEP:.0f}~{ccw*STEP:.0f}°" if cw is not None else "-"
                cur = f"{m2j(j, p*STEP):+.1f}°" if p is not None else "-"
                print(f"{i:3d} {j['dir']:+4d} {j['zero_deg']:6.1f}° "
                      f"{j['min_deg']:6.1f}~{j['max_deg']:.1f}° | {lim:>18} | {cur:>10}")
    except KeyboardInterrupt:
        print()
    finally:
        port.closePort()

if __name__ == "__main__":
    main()
