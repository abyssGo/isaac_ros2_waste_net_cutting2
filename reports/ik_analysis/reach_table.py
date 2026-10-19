import re, glob, os, math, csv
from pathlib import Path
G = str(Path(__file__).resolve().parents[1] / "gpu_logs")   # reports/gpu_logs
SHOULDER = (0.0, -2.0, 0.6345); ARM = 0.411 + 0.368
WRIST_FROM_TCP = (0.251, 0.0, 0.0056)   # 수평 절단 자세에서 TCP -> 손목 중심
rows = []
for run in sorted(glob.glob(f"{G}/run*")):
    pa = open(f"{run}/pc_a.log", encoding="utf-8", errors="ignore").read().splitlines()
    sim = open(f"{run}/sim.log", encoding="utf-8", errors="ignore").read()
    motions = []
    for line in pa:
        m = re.search(r"robot1_node\]: Motion \[([^\]]+)\]: \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)", line)
        if m: motions.append((m.group(1), *map(float, m.groups()[1:])))
    iks = re.findall(r"\[NetClean IK\] robot1 (SUCCESS|FAILED)", sim)
    if len(motions) != len(iks):
        print(os.path.basename(run), "pair mismatch", len(motions), len(iks)); continue
    for (label, x, y, z), res in zip(motions, iks):
        if "CONTACT" not in label: continue
        w = (x + WRIST_FROM_TCP[0], y + WRIST_FROM_TCP[1], z + WRIST_FROM_TCP[2])
        d = math.dist(w, SHOULDER)
        rows.append((os.path.basename(run), label, y, z, round((ARM - d) * 1000, 1), res))
print(f"CONTACT 명령 {len(rows)}개")
ok = [r for r in rows if r[5] == "SUCCESS"]; bad = [r for r in rows if r[5] == "FAILED"]
print(f"성공 {len(ok)}: 도달 여유 최소 {min(r[4] for r in ok)} mm / 최대 {max(r[4] for r in ok)} mm")
print(f"실패 {len(bad)}: 도달 여유 " + ", ".join(f"{r[4]} mm" for r in bad))
for r in bad: print("   ", r)
print("여유 0~10 mm 성공 사례:", [(r[0][:4], r[1], r[4]) for r in ok if r[4] < 10])
with open(f"{G}/robot1_contact_reach.csv", "w", newline="") as f:
    wr = csv.writer(f); wr.writerow(["run", "label", "tcp_y", "tcp_z", "reach_margin_mm", "ik"]); wr.writerows(rows)
