#!/usr/bin/env python3
"""AX-12A Compliance 모터별 튜닝 → JSON 저장 → 불러와서 적용
  python3 comp_tool.py auto              # ID 1~6: 저장값 없는 모터만 튜닝 → 저장 → 적용 → 확인
  python3 comp_tool.py auto --force      # ID 1~6 전부 다시 튜닝
  python3 comp_tool.py tune --ids 3      # 특정 모터만 튜닝
  python3 comp_tool.py apply             # 저장값 불러와서 적용 (전원 켤 때마다)
  python3 comp_tool.py show              # 현재값 / 저장값 비교
다른 코드에서:  from comp_tool import apply_all;  apply_all(port, pk)
"""
import os, json, time, argparse, itertools
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DEFAULT_IDS = [1, 2, 3, 4, 5, 6]
CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "compliance.json")
STEP = 300 / 1023
TARGETS = [30, 90, 60, 150, 120, 210, 180, 270, 240]   # 상승/하강 이동 혼합

# ---------------- 통신 유틸 ----------------
def rd(port, k, i, addr, n=2):
    for _ in range(3):
        try:
            f = k.read2ByteTxRx if n == 2 else k.read1ByteTxRx
            v, r, _ = f(port, i, addr)
            if r == COMM_SUCCESS:
                return v
        except (IndexError, ValueError):
            pass
        time.sleep(0.01); port.clearPort()
    return None

def alive(port, k, i):
    return rd(port, k, i, 0) == 12                      # 모델번호 12 = AX-12A

def set_comp(port, k, i, margin, slope, punch):
    k.write1ByteTxRx(port, i, 26, margin); k.write1ByteTxRx(port, i, 27, margin)
    k.write1ByteTxRx(port, i, 28, slope);  k.write1ByteTxRx(port, i, 29, slope)
    k.write2ByteTxRx(port, i, 48, punch)

def get_comp(port, k, i):
    return dict(margin=rd(port, k, i, 26, 1), slope=rd(port, k, i, 28, 1),
                punch=rd(port, k, i, 48))

def wait_stop(port, k, i, timeout=4.0):
    t0 = time.time(); time.sleep(0.1)
    while time.time() - t0 < timeout:
        if rd(port, k, i, 46, 1) == 0:
            return
        time.sleep(0.03)

# ---------------- JSON 저장 / 불러오기 ----------------
def load_cfg(path=CFG):
    if os.path.exists(path):
        with open(path) as f:
            return {int(i): v for i, v in json.load(f).items()}
    return {}

def save_cfg(cfg, path=CFG):
    with open(path, "w") as f:
        json.dump({str(i): v for i, v in sorted(cfg.items())}, f, indent=2)

# ---------------- 적용 (다른 코드에서 import 가능) ----------------
def apply_all(port, k, ids=None, cfg=None, verbose=True):
    cfg = load_cfg() if cfg is None else cfg
    ids = ids or sorted(cfg)
    applied, missing = [], []
    for i in ids:
        v = cfg.get(i)
        if v is None or not alive(port, k, i):
            missing.append(i); continue
        set_comp(port, k, i, v["margin"], v["slope"], v["punch"])
        applied.append(i)
        if verbose:
            print(f"  ID {i}: margin={v['margin']} slope={v['slope']} punch={v['punch']}")
    if verbose and missing:
        print(f"  ⚠ 적용 못 함 (저장값 없음 또는 응답 없음): {missing}")
    return applied, missing

# ---------------- 튜닝 ----------------
def measure(port, k, i, repeat):
    errs, jit = [], 0
    for _ in range(repeat):
        for t in TARGETS:
            traw = int(round(t / STEP))
            k.write2ByteTxRx(port, i, 30, traw)
            wait_stop(port, k, i); time.sleep(0.15)
            s = [x for x in (rd(port, k, i, 36) for _ in range(8)) if x is not None]
            if s:
                errs.append(abs(traw - s[-1]))
                jit = max(jit, max(s) - min(s))
    return errs, jit

def tune_one(port, k, i, quick=False, repeat=1):
    M, S, P = ([0, 1], [16, 32], [32, 64]) if quick else \
              ([0, 1], [8, 16, 32, 64], [32, 48, 64, 96])
    k.write2ByteTxRx(port, i, 32, 100)
    k.write1ByteTxRx(port, i, 24, 1)
    res, combos = [], list(itertools.product(M, S, P))
    try:
        for n, (m, s, p) in enumerate(combos, 1):
            set_comp(port, k, i, m, s, p)
            errs, jit = measure(port, k, i, repeat)
            if not errs:
                continue
            r = dict(margin=m, slope=s, punch=p, jit=jit,
                     mean=sum(errs) / len(errs) * STEP, max=max(errs) * STEP)
            res.append(r)
            print(f"    [{n:2d}/{len(combos)}] m={m} s={s:3d} p={p:3d} → 평균 {r['mean']:.2f}° "
                  f"최대 {r['max']:.2f}° 떨림 {jit}" + (" ⚠" if jit > 1 else ""))
    finally:
        k.write2ByteTxRx(port, i, 30, int(round(150 / STEP)))   # 중앙 복귀
        wait_stop(port, k, i)
        k.write1ByteTxRx(port, i, 24, 0)
    if not res:
        return None, None
    stable = [r for r in res if r["jit"] <= 1] or res
    best = min(stable, key=lambda r: (round(r["max"], 2), r["mean"]))
    base = next((r for r in res if (r["margin"], r["slope"], r["punch"]) == (1, 32, 32)), None)
    return best, base

def run_tune(port, k, ids, quick, repeat):
    cfg = load_cfg()
    for i in ids:
        if not alive(port, k, i):
            print(f"\n=== ID {i} === ⚠ 응답 없음 → 건너뜀"); continue
        print(f"\n=== ID {i} 튜닝 ===")
        best, base = tune_one(port, k, i, quick, repeat)
        if best is None:
            print("  측정 실패"); continue
        cfg[i] = dict(margin=best["margin"], slope=best["slope"], punch=best["punch"],
                      mean_deg=round(best["mean"], 3), max_deg=round(best["max"], 3))
        save_cfg(cfg)                                       # 모터마다 즉시 저장
        msg = (f"  ✅ ID {i}: m={best['margin']} s={best['slope']} p={best['punch']} "
               f"(평균 {best['mean']:.2f}° 최대 {best['max']:.2f}°)")
        if base:
            msg += f" | 기본값: 평균 {base['mean']:.2f}° 최대 {base['max']:.2f}°"
        print(msg)
    print(f"\n💾 저장: {CFG}")

def show(port, k, ids):
    cfg = load_cfg()
    print(f"\n{'ID':>3} | {'현재 m/s/p':^12} | {'저장 m/s/p':^12} | {'저장 시 오차':^16} | 상태")
    for i in ids:
        if not alive(port, k, i):
            print(f"{i:3d} | {'-':^12} | {'-':^12} | {'-':^16} | 응답 없음"); continue
        c, s = get_comp(port, k, i), cfg.get(i)
        cur = f"{c['margin']}/{c['slope']}/{c['punch']}"
        sav = f"{s['margin']}/{s['slope']}/{s['punch']}" if s else "-"
        err = f"{s['mean_deg']:.2f}° / {s['max_deg']:.2f}°" if s else "-"
        st = "✅ 적용됨" if s and (c['margin'], c['slope'], c['punch']) == \
             (s['margin'], s['slope'], s['punch']) else ("미튜닝" if not s else "⚠ 미적용")
        print(f"{i:3d} | {cur:^12} | {sav:^12} | {err:^16} | {st}")

# ---------------- CLI ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["auto", "tune", "apply", "show"])
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--ids", type=int, nargs="+", default=DEFAULT_IDS)
    ap.add_argument("--quick", action="store_true", help="조합 축소 (모터당 약 1분)")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--force", action="store_true", help="auto에서 저장값 있어도 재튜닝")
    a = ap.parse_args()

    port, k = PortHandler(a.port), PacketHandler(1.0)
    assert port.openPort() and port.setBaudRate(1000000), "포트 열기 실패"
    try:
        if a.cmd == "tune":
            run_tune(port, k, a.ids, a.quick, a.repeat)
        elif a.cmd == "apply":
            print(f"[apply] {CFG}"); apply_all(port, k, a.ids)
        elif a.cmd == "show":
            show(port, k, a.ids)
        elif a.cmd == "auto":
            cfg = load_cfg()
            todo = [i for i in a.ids if a.force or i not in cfg]
            print(f"[auto] 대상 {a.ids} | 튜닝 필요: {todo or '없음'}")
            if todo:
                run_tune(port, k, todo, a.quick, a.repeat)
            print("\n[apply] 저장값 불러와서 적용")
            apply_all(port, k, a.ids)
            show(port, k, a.ids)
    except KeyboardInterrupt:
        print("\n중단 (완료된 모터까지는 저장됨)")
    finally:
        port.closePort()

if __name__ == "__main__":
    main()
