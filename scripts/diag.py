#!/usr/bin/env python3
"""AX-12A 동작 불가 진단:  python3 diag.py --ids 2 3"""
import argparse
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
from comp_tool import rd

STEP = 300 / 1023
ERR = ["입력전압", "각도제한", "과열", "범위", "체크섬", "과부하", "명령"]
ap = argparse.ArgumentParser(); ap.add_argument("--ids", type=int, nargs="+", default=[2, 3])
a = ap.parse_args()
port, k = PortHandler("/dev/ttyACM0"), PacketHandler(1.0)
assert port.openPort() and port.setBaudRate(1000000), "포트 열기 실패 (다른 프로그램이 사용 중?)"

for i in a.ids:
    print(f"\n===== ID {i} =====")
    try:
        m, r, e = k.ping(port, i)
    except (IndexError, ValueError):
        r = -1
    if r != COMM_SUCCESS:
        print("  ❌ 응답 없음 → 전원/케이블/ID 확인"); continue
    v = {n: rd(port, k, i, ad, sz) for n, ad, sz in [
        ("torque", 24, 1), ("cw", 6, 2), ("ccw", 8, 2), ("goal", 30, 2), ("pos", 36, 2),
        ("moving", 46, 1), ("tlim", 34, 2), ("maxt", 14, 2), ("volt", 42, 1),
        ("temp", 43, 1), ("shutdown", 18, 1), ("speed", 32, 2)]}
    errs = [ERR[b] for b in range(7) if e >> b & 1]
    print(f"  토크 {'ON' if v['torque'] else 'OFF'} | 현재 {v['pos']*STEP:.1f}° | 목표 {v['goal']*STEP:.1f}° | "
          f"moving {v['moving']}")
    print(f"  각도제한 {v['cw']*STEP:.1f}° ~ {v['ccw']*STEP:.1f}° | 속도 {v['speed']}")
    print(f"  Torque Limit {v['tlim']} | Max Torque {v['maxt']} | {v['volt']/10:.1f} V | {v['temp']}°C")
    print(f"  에러 플래그: {errs or '없음'}")

    probs = []
    if v["cw"] == 0 and v["ccw"] == 0: probs.append("바퀴 모드(CW=CCW=0) → 위치 명령 무시됨")
    if v["tlim"] == 0:  probs.append("Torque Limit = 0 → 과부하/과열 셧다운 이후 상태 (전원 재투입 또는 34번 주소 복구)")
    if v["maxt"] == 0:  probs.append("Max Torque = 0 (EEPROM)")
    if not v["torque"]: probs.append("토크 OFF")
    if "각도제한" in errs: probs.append("목표가 각도제한 밖 → 명령 거부됨")
    if "과부하" in errs:  probs.append("과부하 → 기구 걸림/간섭 확인")
    if "과열" in errs:    probs.append("과열 → 식힌 후 재시도")
    if v["volt"] and v["volt"] < 90: probs.append(f"전압 낮음 {v['volt']/10:.1f}V → 힘 부족 가능")
    if abs(v["goal"] - v["pos"]) > 3 and not v["moving"]:
        probs.append("목표와 현재가 다른데 정지 상태 → 막혀 있거나 힘 부족")
    print("  ▶ " + (" / ".join(probs) if probs else "레지스터상 이상 없음"))
port.closePort()
