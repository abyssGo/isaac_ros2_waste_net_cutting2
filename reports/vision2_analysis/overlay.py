# 실행: python3 overlay.py <캡처 폴더> <cycle 번호> <Y 보정(m)>  (모델 경로는 이 파일 위치 기준)
import json, sys, numpy as np, cv2
from pathlib import Path
MODEL = Path(__file__).resolve().parents[2] / "models/netclean_yolo11n/weights/best.pt"
from ultralytics import YOLO
cap, cyc, dy_lag = sys.argv[1], sys.argv[2], float(sys.argv[3])
img = cv2.imread(f"{cap}/cycle{cyc}_rgb.png"); K = json.load(open(f"{cap}/cycle{cyc}_info.json"))["k"]
fx, cx, fy, cy = K[0], K[2], K[4], K[5]
CAM = np.array([-0.1, 2.0, 1.2])
def proj(p):  # world -> pixel (optical: x=+Y, y=-Z, z=-X)
    xc, yc, zc = p[1] - CAM[1], CAM[2] - p[2], CAM[0] - p[0]
    return fx * xc / zc + cx, fy * yc / zc + cy
AABB = {"plastic_bottle": ([-0.668, -5.3797, 0.8868], [-0.6008, -5.3069, 1.0969]),
        "can": ([-0.7319, -4.7501, 0.9718], [-0.5295, -4.6128, 1.181]),
        "buoy": ([-0.6868, -5.094, 1.1537], [-0.5762, -4.8642, 1.2868])}
shift = np.array([0, 7.0 + dy_lag, 0])
res = YOLO(str(MODEL)).predict(img, device="0", conf=0.4, verbose=False)[0]
det = {res.names[int(c)]: b.tolist() for b, c in zip(res.boxes.xyxy.cpu(), res.boxes.cls.cpu())}
for name, (lo, hi) in AABB.items():
    lo, hi = np.array(lo) + shift, np.array(hi) + shift
    corners = [proj([x, y, z]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    us, vs = [c[0] for c in corners], [c[1] for c in corners]
    pb = (min(us), min(vs), max(us), max(vs))
    cv2.rectangle(img, (int(pb[0]), int(pb[1])), (int(pb[2]), int(pb[3])), (255, 0, 255), 2)
    pc = proj((lo + hi) / 2)
    msg = f"{name}: truth center u={pc[0]:.1f} v={pc[1]:.1f}"
    if name in det:
        b = det[name]; cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
        msg += f" | yolo center u={(b[0]+b[2])/2:.1f} v={(b[1]+b[3])/2:.1f} | du={(b[0]+b[2])/2-pc[0]:+.1f}px dv={(b[1]+b[3])/2-pc[1]:+.1f}px"
    print(msg)
cv2.putText(img, "magenta: USD truth projected   green: YOLO", (8, 630), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
cv2.imwrite(f"{cap}/cycle{cyc}_overlay.png", img)
