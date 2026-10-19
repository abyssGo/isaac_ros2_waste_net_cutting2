# 실행: $ISAAC_SIM_ROOT/python.sh reports/ik_analysis/ik_repro.py  (경로는 이 파일 위치 기준)
import os, sys
from pathlib import Path
ISAAC_SIM_ROOT = Path(os.environ.get("ISAAC_SIM_ROOT", Path.home() / "isaacsim"))
sys.path.insert(0, str(ISAAC_SIM_ROOT / "exts/isaacsim.robot_motion.lula/pip_prebundle"))
import numpy as np, lula
A = str(Path(__file__).resolve().parents[2] / "sim/assets/project1/robot2_sample") + "/"
robot = lula.load_robot(A + "m0609_description.yaml", A + "m0609_isaac_sim.urdf")
kin = robot.kinematics()
BASE = np.array([0.0, -2.0, 0.5])

def quat_wxyz_to_mat(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

ROT = quat_wxyz_to_mat(np.array([-0.69899, -0.01358, 0.71488, 0.01357]))

def solve(flange_world, warm=None, seed=123456, descents=100, extra_seeds=()):
    cfg = lula.CyclicCoordDescentIkConfig()
    cfg.sampling_seed = seed; cfg.max_num_descents = descents
    seeds = ([np.asarray(warm, float)] if warm is not None else []) + [np.asarray(s, float) for s in extra_seeds]
    cfg.cspace_seeds = seeds
    pose = lula.Pose3(lula.Rotation3(ROT), np.asarray(flange_world) - BASE)
    r = lula.compute_ik_ccd(kin, pose, "link_6", cfg)
    return r.success, np.asarray(r.cspace_position), r.position_error, max(r.x_axis_orientation_error, r.y_axis_orientation_error, r.z_axis_orientation_error)

q_app = [-2.204, 0.5137, 0.712, -1.3015, 0.9908, -2.0439]   # 실패 직전 자세(APPROACH)
FAIL = [-0.56, -2.445, 1.1151]                               # 실패한 CONTACT flange 목표

ok, q, pe, oe = solve(FAIL, q_app)
print(f"[1] 실패 재현 (warm=APPROACH, seed 고정): success={ok} pos_err={pe*1000:.2f}mm ori_err={oe:.4f}")

print("[2] 같은 warm start 로 목표 Y 를 옮겨 가며 (X, Z 고정)")
for y in np.arange(-2.400, -2.4801, -0.005):
    ok, q, pe, oe = solve([FAIL[0], y, FAIL[2]], q_app)
    print(f"    y={y:.3f} success={ok} pos_err={pe*1000:6.2f}mm ori_err={oe:.4f}")

print("[3] 해 존재 여부: sampling_seed 를 바꿔 다시 풀기 (warm 없음)")
found = []
for s in range(1, 41):
    ok, q, pe, oe = solve(FAIL, None, seed=s)
    if ok: found.append((s, q))
print(f"    40개 seed 중 성공 {len(found)}개")
for s, q in found[:3]:
    print(f"    seed={s} q={np.round(q,3).tolist()}  |q-q_app|max={np.max(np.abs(q-np.array(q_app))):.3f} rad")

ok, q, pe, oe = solve(FAIL, q_app, descents=500)
print(f"[4] warm=APPROACH, max_num_descents 500: success={ok} pos_err={pe*1000:.2f}mm")
if found:
    ok, q, pe, oe = solve(FAIL, q_app, extra_seeds=[found[0][1]])
    print(f"[5] warm=APPROACH + 성공해를 추가 seed 로: success={ok}")

print("[6] 경계 부근 해의 관절값과 한계")
lo = np.array([kin.c_space_coord_limits(i).lower for i in range(6)]); hi = np.array([kin.c_space_coord_limits(i).upper for i in range(6)])
print("    limits lower", np.round(lo,3).tolist()); print("    limits upper", np.round(hi,3).tolist())
for y in (-2.400, -2.415, -2.425, -2.445):
    ok, q, pe, oe = solve([FAIL[0], y, FAIL[2]], q_app)
    margin = np.minimum(q - lo, hi - q)
    wc = kin.pose(q, "link_5").translation + BASE
    print(f"    y={y:.3f} ok={ok} q={np.round(q,3).tolist()} min_margin=j{int(np.argmin(margin))+1}:{margin.min():.3f}rad wrist={np.round(wc,3).tolist()}")
