"""Generate the factory signboard textures (Korean labels) into ../textures/."""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parents[1] / "textures"
FONT = "/usr/share/fonts/truetype/nanum/NanumSquareRoundEB.ttf"
if not Path(FONT).exists():
    FONT = "/usr/share/fonts/truetype/nanum/NanumSquareRoundB.ttf"


def sign(name, size, bg, lines, accent=None, warning=False):
    image = Image.new("RGB", size, bg)
    draw = ImageDraw.Draw(image)
    width, height = size
    band = int(height * 0.09) if accent else 0
    if accent:
        draw.rectangle([0, 0, width, band], fill=accent)
        draw.rectangle([0, height - band, width, height], fill=accent)
    fonts = [ImageFont.truetype(FONT, int(height * frac)) for _, frac, _ in lines]
    boxes = [draw.textbbox((0, 0), text, font=f) for (text, _, _), f in zip(lines, fonts)]
    gap = int(height * 0.06)
    total = sum(b[3] - b[1] for b in boxes) + gap * (len(lines) - 1)
    y = (height - total) / 2
    for (text, _, color), font, box in zip(lines, fonts, boxes):
        text_w, text_h = box[2] - box[0], box[3] - box[1]
        x = (width - text_w) / 2
        if warning and color is None:
            side = text_h * 1.15
            x += side * 0.6
            tx, ty = x - side * 1.25, y + text_h / 2 - side / 2
            draw.polygon([(tx + side / 2, ty), (tx + side, ty + side), (tx, ty + side)], fill=(20, 20, 20))
            draw.polygon([(tx + side / 2, ty + side * 0.2), (tx + side * 0.83, ty + side * 0.88), (tx + side * 0.17, ty + side * 0.88)], fill=bg)
            draw.rectangle([tx + side * 0.46, ty + side * 0.38, tx + side * 0.54, ty + side * 0.66], fill=(20, 20, 20))
            draw.rectangle([tx + side * 0.46, ty + side * 0.72, tx + side * 0.54, ty + side * 0.80], fill=(20, 20, 20))
        draw.text((x - box[0], y - box[1]), text, font=font, fill=color or (255, 255, 255) if not warning else color or (20, 20, 20))
        y += text_h + gap
    image.save(OUT / name)


OUT.mkdir(exist_ok=True)
sign("sign_line.png", (2048, 512), (18, 52, 86),
     [("NETCLEAN  폐어망 재활용 라인 1", 0.22, None), ("수거 어망 → 이물 탐지 → 절단 → 흡착 제거 → 재검사", 0.11, (255, 214, 64))],
     accent=(255, 196, 0))
sign("sign_robot_cell.png", (1024, 512), (255, 204, 0),
     [("로봇 작업 구역", 0.2, None), ("ROBOT CELL · 출입 시 정지 버튼", 0.1, (20, 20, 20))],
     accent=(20, 20, 20), warning=True)
for name, ko, en, color in [("pet", "PET 병", "PLASTIC BOTTLE", (30, 110, 200)), ("can", "캔", "CAN", (110, 120, 130)), ("buoy", "부표", "BUOY", (200, 60, 40))]:
    sign(f"sign_bin_{name}.png", (1024, 512), color, [(f"{ko} 선별품", 0.22, None), (en, 0.12, None)])
sign("sign_net_storage.png", (1024, 512), (30, 120, 70), [("수거 어망 적치장", 0.2, None), ("COLLECTED NET STORAGE", 0.1, None)])
print("signs written to", OUT)
