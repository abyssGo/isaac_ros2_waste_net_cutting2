"""OpenCV renderer shared by both NetClean debug popups."""

from __future__ import annotations

from typing import Any, Dict, Iterable, Optional, Sequence, Tuple

import cv2
import numpy as np


Color = Tuple[int, int, int]


class Palette:
    BG: Color = (244, 244, 241)         # warm poster white
    PANEL: Color = (250, 250, 247)
    PANEL_2: Color = (225, 240, 252)    # pale yellow
    BORDER: Color = (25, 25, 25)
    TEXT: Color = (20, 20, 20)
    ON_DARK: Color = (248, 248, 245)
    MUTED: Color = (82, 82, 78)
    BLUE: Color = (235, 225, 20)        # cyan bbox: plastic_bottle
    RED: Color = (220, 45, 225)         # magenta bbox: can
    YELLOW: Color = (70, 220, 75)       # green bbox: buoy
    GREEN: Color = (40, 220, 80)       # action included / success
    ORANGE: Color = (45, 190, 245)     # poster yellow / current task
    GREY: Color = (150, 150, 150)      # excluded
    MAGENTA: Color = (255, 40, 220)    # error
    CYAN: Color = (220, 220, 0)        # ROI
    BLACK: Color = (0, 0, 0)


CLASS_COLORS: Dict[str, Color] = {
    "plastic_bottle": Palette.BLUE,
    "can": Palette.RED,
    "buoy": Palette.YELLOW,
}


def _safe_float(value: Any) -> Optional[float]:
    try:
        if value is None:
            return None
        result = float(value)
        if not np.isfinite(result):
            return None
        return result
    except (TypeError, ValueError):
        return None


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


class PopupRenderer:
    """Render one complete debug dashboard into a BGR image."""

    def __init__(
        self,
        mode: str,
        width: int = 1280,
        height: int = 720,
        roi: Optional[Sequence[int]] = None,
        validation_defaults: Optional[Dict[str, Optional[float]]] = None,
    ) -> None:
        if mode not in ("vision1", "vision2"):
            raise ValueError("mode must be 'vision1' or 'vision2'")
        self.mode = mode
        self.width = max(1180, int(width))
        self.height = max(680, int(height))
        self.top_h = max(72, int(self.height * 0.10))
        self.bottom_h = max(54, int(self.height * 0.075))
        # The camera now owns the complete content width.  Information is drawn
        # as a compact translucent overlay instead of reserving a side panel.
        self.panel_w = max(320, int(self.width * 0.26))
        self.video_w = self.width
        self.video_h = self.height - self.top_h - self.bottom_h
        self.roi = list(roi) if roi and len(roi) == 4 else None
        self.validation_defaults = validation_defaults or {}

    @property
    def title(self) -> str:
        if self.mode == "vision1":
            return "NETCLEAN VISION 1  |  CUTTING DEBUG"
        return "NETCLEAN VISION 2  |  REMOVAL DEBUG"

    @property
    def action_name(self) -> str:
        return "/cut/execute" if self.mode == "vision1" else "/remove/execute"

    def render(self, frame: np.ndarray, status: Optional[Dict[str, Any]]) -> np.ndarray:
        status = status if isinstance(status, dict) else {}
        canvas = np.full((self.height, self.width, 3), Palette.BG, dtype=np.uint8)
        frame = self._normalize_frame(frame)
        video, transform = self._fit_frame(frame)
        x0, y0, _ = transform
        canvas[y0:y0 + video.shape[0], x0:x0 + video.shape[1]] = video

        self._draw_top_bar(canvas, status)
        self._draw_video_border(canvas)
        self._draw_roi(canvas, transform)

        detections = self._detections(status)
        for detection in detections:
            self._draw_detection(canvas, detection, transform)

        self._draw_side_panel(canvas, status, detections)
        self._draw_bottom_bar(canvas, status)
        return canvas

    def _normalize_frame(self, frame: np.ndarray) -> np.ndarray:
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            return np.full((720, 1280, 3), (12, 14, 17), dtype=np.uint8)
        if frame.ndim == 2:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame

    def _fit_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int, float]]:
        src_h, src_w = frame.shape[:2]
        # Contain-fit preserves the original camera ratio and guarantees that
        # the complete 1x1 m net remains visible. No source pixel is cropped.
        scale = min(self.video_w / src_w, self.video_h / src_h)
        dst_w = max(1, int(round(src_w * scale)))
        dst_h = max(1, int(round(src_h * scale)))
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        resized = cv2.resize(frame, (dst_w, dst_h), interpolation=interpolation)
        x0 = (self.video_w - dst_w) // 2
        y0 = self.top_h + (self.video_h - dst_h) // 2
        return resized, (x0, y0, scale)

    def _draw_top_bar(self, canvas: np.ndarray, status: Dict[str, Any]) -> None:
        cv2.rectangle(canvas, (0, 0), (self.width - 1, self.top_h - 1), Palette.PANEL, -1)
        cv2.line(canvas, (0, self.top_h - 1), (self.width, self.top_h - 1), Palette.BORDER, 1)

        self._put_display_text(canvas, self.title, (18, 32), 0.72, Palette.TEXT, 2)
        model = str(status.get("model_name", "best.pt"))
        device = str(status.get("device", "UNKNOWN"))
        self._put_text(
            canvas,
            f"LIVE UI  /  MODEL {model}  /  DEVICE {device}",
            (18, 56),
            0.48,
            Palette.MUTED,
            1,
        )

        phase = str(status.get("phase", "WAITING")).upper()
        scan = str(status.get("scan", "INITIAL SCAN")).upper()
        action_server = str(status.get("action_server", "WAITING")).upper()
        phase_color = self._state_color(phase)
        phase_x = max(390, self.width // 2 - 90)
        self._badge(canvas, phase, (phase_x, 15), phase_color, 178)
        self._put_text(
            canvas,
            f"{scan}  |  {self.action_name} {action_server}",
            (phase_x, 58),
            0.48,
            Palette.TEXT,
            1,
            max_width=295,
        )

        current = _safe_int(status.get("current_task"))
        total = _safe_int(status.get("total_tasks"))
        verb = "CUTTING TARGET" if self.mode == "vision1" else "REMOVAL TARGET"
        task_text = f"{verb}  {current} / {total}" if total else f"{verb}  0 / 0"
        self._put_text(
            canvas,
            task_text,
            (self.width - self.panel_w + 12, 31),
            0.62,
            Palette.ORANGE if current else Palette.MUTED,
            2,
            max_width=self.panel_w - 26,
        )
        station = str(status.get("station", "WAITING")).upper()
        self._put_text(
            canvas,
            f"STATION: {station}",
            (self.width - self.panel_w + 12, 58),
            0.48,
            Palette.MUTED,
            1,
        )

    def _draw_video_border(self, canvas: np.ndarray) -> None:
        cv2.rectangle(
            canvas,
            (0, self.top_h),
            (self.width - 1, self.height - self.bottom_h - 1),
            Palette.BORDER,
            1,
        )

    def _draw_roi(self, canvas: np.ndarray, transform: Tuple[int, int, float]) -> None:
        if self.roi is None:
            return
        x0, y0, scale = transform
        rx1, ry1, rx2, ry2 = [int(v) for v in self.roi]
        p1 = (x0 + int(rx1 * scale), y0 + int(ry1 * scale))
        p2 = (x0 + int(rx2 * scale), y0 + int(ry2 * scale))
        cv2.rectangle(canvas, p1, p2, Palette.CYAN, 2, cv2.LINE_AA)
        self._label(canvas, "ROI", (p1[0], max(self.top_h + 4, p1[1] - 22)), Palette.CYAN)

    def _detections(self, status: Dict[str, Any]) -> list:
        raw = status.get("detections", [])
        if not isinstance(raw, list):
            return []
        output = []
        for item in raw:
            if not isinstance(item, dict):
                continue
            bbox = item.get("bbox", [])
            if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
                continue
            normalized = dict(item)
            normalized["bbox"] = [_safe_int(v) for v in bbox]
            normalized["confidence"] = _safe_float(item.get("confidence")) or 0.0
            normalized["class_name"] = str(item.get("class_name", "unknown"))
            normalized["object_id"] = str(
                item.get("object_id", normalized["class_name"])
            )
            output.append(normalized)
        return output

    def _draw_detection(
        self,
        canvas: np.ndarray,
        det: Dict[str, Any],
        transform: Tuple[int, int, float],
    ) -> None:
        x0, y0, scale = transform
        x1, y1, x2, y2 = det["bbox"]
        left = x0 + int(x1 * scale)
        top = y0 + int(y1 * scale)
        right = x0 + int(x2 * scale)
        bottom = y0 + int(y2 * scale)
        left = int(np.clip(left, 0, self.video_w - 1))
        right = int(np.clip(right, 0, self.video_w - 1))
        top = int(np.clip(top, self.top_h, self.height - self.bottom_h - 1))
        bottom = int(np.clip(bottom, self.top_h, self.height - self.bottom_h - 1))

        class_name = det["class_name"]
        color = CLASS_COLORS.get(class_name, Palette.MAGENTA)
        cv2.rectangle(canvas, (left, top), (right, bottom), color, 3, cv2.LINE_AA)

        confidence = int(round(float(det["confidence"]) * 100.0))
        object_id = det["object_id"]
        label_origin = (left, max(self.top_h + 3, top - 29))
        self._label(canvas, f"{object_id}  {confidence}%", label_origin, color)

        state = str(det.get("action_state", "EXCLUDED")).upper()
        included = bool(det.get("action_included", False))
        if "ERROR" in state or "FAILED" in state or "REJECTED" in state:
            status_color = Palette.MAGENTA
            status_text = state
        elif "LIVE MONITOR" in state:
            status_color = Palette.ORANGE
            status_text = "LIVE"
        elif "PROGRESS" in state or "REMOVING" in state or "CUTTING" in state:
            status_color = Palette.ORANGE
            status_text = "IN PROGRESS"
        elif "COMPLETE" in state or "DONE" in state:
            status_color = Palette.GREEN
            status_text = "COMPLETE"
        elif included:
            status_color = Palette.GREEN
            status_text = "ACTION INCLUDED"
        else:
            status_color = Palette.GREY
            reason = str(det.get("reject_reason", "EXCLUDED"))
            status_text = reason if reason else "EXCLUDED"
        # QUEUED/ACTION INCLUDED is already shown in the large side card.
        # Suppressing its video badge prevents labels from covering nearby boxes.
        show_status_badge = not included or any(
            token in state
            for token in (
                "ERROR",
                "FAILED",
                "REJECTED",
                "PROGRESS",
                "REMOVING",
                "CUTTING",
                "COMPLETE",
                "DONE",
            )
        )
        if show_status_badge:
            status_origin = (
                left,
                min(bottom + 5, self.height - self.bottom_h - 26),
            )
            self._label(canvas, status_text, status_origin, status_color)

        if self.mode == "vision1":
            self._draw_vertices(canvas, left, top, right, bottom, color)
        else:
            self._draw_center(
                canvas,
                det,
                left,
                top,
                right,
                bottom,
                status_color,
                transform,
            )

    def _draw_vertices(
        self,
        canvas: np.ndarray,
        left: int,
        top: int,
        right: int,
        bottom: int,
        color: Color,
    ) -> None:
        points = [
            ("P1", (left, top), (6, 17)),
            ("P2", (right, top), (-31, 17)),
            ("P3", (right, bottom), (-31, -8)),
            ("P4", (left, bottom), (6, -8)),
        ]
        for name, point, offset in points:
            cv2.circle(canvas, point, 5, color, -1, cv2.LINE_AA)
            text_point = (point[0] + offset[0], point[1] + offset[1])
            self._put_text(
                canvas,
                name,
                text_point,
                0.45,
                Palette.TEXT,
                2,
                shadow=True,
            )

    def _draw_center(
        self,
        canvas: np.ndarray,
        det: Dict[str, Any],
        left: int,
        top: int,
        right: int,
        bottom: int,
        status_color: Color,
        transform: Tuple[int, int, float],
    ) -> None:
        cx = (left + right) // 2
        cy = (top + bottom) // 2
        cv2.circle(canvas, (cx, cy), 6, Palette.TEXT, 2, cv2.LINE_AA)
        bx1, by1, bx2, by2 = det["bbox"]
        source_cx = (bx1 + bx2) // 2
        source_cy = (by1 + by2) // 2
        self._put_text(
            canvas,
            f"C ({source_cx}, {source_cy})",
            (cx + 10, cy - 10),
            0.48,
            Palette.TEXT,
            1,
            shadow=True,
        )

        grasp_pixel = det.get("grasp_pixel")
        if isinstance(grasp_pixel, (list, tuple)) and len(grasp_pixel) == 2:
            grasp_u = _safe_int(grasp_pixel[0])
            grasp_v = _safe_int(grasp_pixel[1])
            transform_x, transform_y, transform_scale = transform
            gx = transform_x + int(round(grasp_u * transform_scale))
            gy = transform_y + int(round(grasp_v * transform_scale))
            gx = int(np.clip(gx, 0, self.video_w - 1))
            gy = int(np.clip(gy, self.top_h, self.height - self.bottom_h - 1))
            cv2.line(canvas, (cx, cy), (gx, gy), Palette.GREEN, 2, cv2.LINE_AA)
            cv2.circle(canvas, (gx, gy), 7, Palette.GREEN, 3, cv2.LINE_AA)
            cv2.line(canvas, (gx - 13, gy), (gx + 13, gy), Palette.GREEN, 2, cv2.LINE_AA)
            cv2.line(canvas, (gx, gy - 13), (gx, gy + 13), Palette.GREEN, 2, cv2.LINE_AA)
            self._put_text(
                canvas,
                f"G ({grasp_u}, {grasp_v})",
                (gx + 10, gy + 18),
                0.48,
                Palette.GREEN,
                2,
                shadow=True,
            )
        else:
            gx, gy = cx, cy
            cv2.line(canvas, (cx - 14, cy), (cx + 14, cy), status_color, 2, cv2.LINE_AA)
            cv2.line(canvas, (cx, cy - 14), (cx, cy + 14), status_color, 2, cv2.LINE_AA)

        state = str(det.get("action_state", "")).upper()
        if not any(token in state for token in ("PROGRESS", "REMOVING", "CUTTING")):
            return
        details = []
        depth = _safe_float(det.get("depth_m"))
        if depth is not None:
            details.append(f"DEPTH {depth:.3f} m")
        camera = det.get("camera_xyz")
        if isinstance(camera, (list, tuple)) and len(camera) == 3:
            xyz = [_safe_float(v) for v in camera]
            if all(v is not None for v in xyz):
                details.append(
                    f"CAMERA ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})"
                )
        world = det.get("world_xyz")
        if isinstance(world, (list, tuple)) and len(world) == 3:
            xyz = [_safe_float(v) for v in world]
            if all(v is not None for v in xyz):
                details.append(f"WORLD ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})")
        for index, line in enumerate(details):
            self._put_text(
                canvas,
                line,
                (gx + 10, gy + 38 + index * 20),
                0.45,
                Palette.TEXT,
                1,
                shadow=True,
            )

    def _draw_side_panel(
        self,
        canvas: np.ndarray,
        status: Dict[str, Any],
        detections: Iterable[Dict[str, Any]],
    ) -> None:
        detections = list(detections)
        margin = 14
        panel_x = self.width - self.panel_w - margin
        panel_right = self.width - margin
        panel_top = self.top_h + 12
        panel_bottom = self.height - self.bottom_h
        # Poster-style warm-white overlay with black type and yellow accents.
        overlay = canvas.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_top),
            (panel_right, panel_bottom - 12),
            Palette.PANEL,
            -1,
        )
        cv2.addWeighted(overlay, 0.93, canvas, 0.07, 0.0, canvas)
        cv2.rectangle(
            canvas,
            (panel_x, panel_top),
            (panel_right, panel_bottom - 12),
            Palette.BORDER,
            1,
        )

        x = panel_x + 16
        y = panel_top + 29
        self._put_display_text(
            canvas,
            "NON-NET / LIVE",
            (x, y),
            0.61,
            Palette.TEXT,
            2,
        )
        self._badge(
            canvas,
            str(len(detections)),
            (panel_right - 48, y - 23),
            Palette.CYAN,
            42,
        )
        y += 14

        # The NetClean detector is configured for a maximum of three targets.
        # Keeping three cards also preserves room for all performance metrics.
        max_cards = 3
        for det in detections[:max_cards]:
            y = self._draw_detection_card(canvas, det, x, y + 7)
        if len(detections) > max_cards:
            self._put_text(
                canvas,
                f"+ {len(detections) - max_cards} MORE OBJECT(S)",
                (x, y + 20),
                0.46,
                Palette.MUTED,
                1,
            )
            y += 28

        y += 7
        cv2.line(canvas, (x, y), (panel_right - 14, y), Palette.BORDER, 1)
        y += 24
        self._put_display_text(
            canvas,
            "LIVE PERFORMANCE",
            (x, y),
            0.56,
            Palette.TEXT,
            2,
        )
        y += 25

        confidences = [float(d["confidence"]) for d in detections]
        average_conf = sum(confidences) / len(confidences) if confidences else None
        valid_targets = sum(bool(d.get("action_included", False)) for d in detections)
        inference_ms = _safe_float(status.get("inference_ms"))
        fps = _safe_float(status.get("fps"))
        current = _safe_int(status.get("current_task"))
        total = _safe_int(status.get("total_tasks"))
        rows = [
            ("Objects detected", str(len(detections))),
            ("Valid action targets", str(valid_targets)),
            ("Average confidence", self._percent(average_conf)),
            ("Inference time", self._number(inference_ms, " ms", 1)),
            ("Vision FPS", self._number(fps, "", 1)),
        ]
        if self.mode == "vision2":
            valid_depth = sum(_safe_float(d.get("depth_m")) is not None for d in detections)
            rows.append(("Valid depth targets", str(valid_depth)))
        rows.append(("Action progress", f"{current}/{total}" if total else "0/0"))
        y = self._draw_rows(canvas, rows, x, y)

        validation = status.get("validation", {})
        if not isinstance(validation, dict):
            validation = {}
        precision = self._metric(validation, "precision")
        recall = self._metric(validation, "recall")
        map50 = self._metric(validation, "map50")
        map50_95 = self._metric(validation, "map50_95")
        # Empty validation values contributed no live debugging information and
        # consumed a large block.  Show this section only when measured values
        # have actually been configured.
        if not all(value == "N/A" for value in (precision, recall, map50, map50_95)):
            y += 7
            cv2.line(canvas, (x, y), (panel_right - 14, y), Palette.BORDER, 1)
            y += 23
            self._put_display_text(
                canvas,
                "MODEL VALIDATION",
                (x, y),
                0.50,
                Palette.TEXT,
                2,
            )
            y += 23
            self._put_text(
                canvas,
                f"P {precision}   R {recall}   mAP50 {map50}",
                (x, y),
                0.46,
                Palette.TEXT,
                1,
                max_width=self.panel_w - 32,
            )
            self._put_text(
                canvas,
                f"mAP50-95 {map50_95}",
                (x, y + 24),
                0.46,
                Palette.TEXT,
                1,
            )

    def _draw_detection_card(
        self,
        canvas: np.ndarray,
        det: Dict[str, Any],
        x: int,
        y: int,
    ) -> int:
        card_w = self.panel_w - 32
        card_h = 59

        class_name = det["class_name"]
        color = CLASS_COLORS.get(class_name, Palette.MAGENTA)
        state = str(det.get("action_state", "EXCLUDED")).upper()
        included = bool(det.get("action_included", False))
        if any(token in state for token in ("ERROR", "FAILED", "REJECTED")):
            marker, marker_color = "X", Palette.MAGENTA
        elif "LIVE MONITOR" in state:
            marker, marker_color = ">", Palette.ORANGE
        elif any(token in state for token in ("PROGRESS", "REMOVING", "CUTTING")):
            marker, marker_color = ">", Palette.ORANGE
        elif "COMPLETE" in state or "DONE" in state:
            marker, marker_color = "V", Palette.GREEN
        elif included:
            marker, marker_color = "O", Palette.GREEN
        else:
            marker, marker_color = "X", Palette.GREY
        self._put_text(canvas, marker, (x + 2, y + 25), 0.60, marker_color, 2)
        self._put_text(
            canvas,
            det["object_id"],
            (x + 27, y + 25),
            0.56,
            Palette.TEXT,
            2,
            max_width=card_w - 105,
        )
        confidence = int(round(float(det["confidence"]) * 100.0))
        self._put_text(
            canvas,
            f"{confidence}%",
            (x + card_w - 51, y + 25),
            0.56,
            color,
            2,
        )
        task_index = _safe_int(det.get("task_index"))
        state_text = state
        if "LIVE MONITOR" in state:
            state_text = "LIVE DETECTION"
        elif not included:
            reason = str(det.get("reject_reason", ""))
            state_text = f"EXCLUDED - {reason}" if reason else "EXCLUDED"
        elif task_index:
            state_text = f"{state} - TARGET {task_index}"
        target_mode = str(det.get("target_mode", ""))
        valid_depth_ratio = _safe_float(det.get("valid_depth_ratio"))
        if target_mode == "DEPTH_MEDIAN_CENTER":
            depth_text = (
                f"DEPTH GRASP {valid_depth_ratio * 100.0:.0f}%"
                if valid_depth_ratio is not None
                else "DEPTH GRASP"
            )
            state_text = f"{depth_text} | {state_text}"
        self._put_text(
            canvas,
            f"{class_name}  |  {state_text}",
            (x + 27, y + 50),
            0.45,
            Palette.MUTED,
            1,
            max_width=card_w - 44,
        )
        return y + card_h

    def _draw_rows(
        self,
        canvas: np.ndarray,
        rows: Iterable[Tuple[str, str]],
        x: int,
        y: int,
    ) -> int:
        for label, value in rows:
            self._put_text(canvas, label, (x, y), 0.48, Palette.MUTED, 1)
            value_w, _ = cv2.getTextSize(
                value,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                1,
            )[0]
            self._put_text(
                canvas,
                value,
                (self.width - 32 - value_w, y),
                0.48,
                Palette.TEXT,
                1,
            )
            y += 25
        return y

    def _draw_bottom_bar(self, canvas: np.ndarray, status: Dict[str, Any]) -> None:
        y0 = self.height - self.bottom_h
        cv2.rectangle(canvas, (0, y0), (self.width - 1, self.height - 1), Palette.ORANGE, -1)
        cv2.line(canvas, (0, y0), (self.width, y0), Palette.BORDER, 1)
        message = str(status.get("message", "Waiting for vision status"))
        message_upper = message.upper()
        if "ERROR" in message_upper:
            color = Palette.MAGENTA
        elif "WARNING" in message_upper or "EXCLUDED" in message_upper:
            color = Palette.YELLOW
        elif "COMPLETE" in message_upper or "SUCCESS" in message_upper:
            color = Palette.GREEN
        else:
            color = Palette.BLACK
        self._put_text(
            canvas,
            message,
            (18, y0 + 36),
            0.58,
            color,
            2,
            max_width=self.width - 36,
        )

    def _metric(self, validation: Dict[str, Any], key: str) -> str:
        value = _safe_float(validation.get(key))
        if value is None:
            value = _safe_float(self.validation_defaults.get(key))
        return f"{value:.3f}" if value is not None and value >= 0.0 else "N/A"

    @staticmethod
    def _number(value: Optional[float], suffix: str, decimals: int) -> str:
        return f"{value:.{decimals}f}{suffix}" if value is not None else "N/A"

    @staticmethod
    def _percent(value: Optional[float]) -> str:
        return f"{value * 100.0:.1f}%" if value is not None else "N/A"

    @staticmethod
    def _state_color(state: str) -> Color:
        if any(token in state for token in ("ERROR", "FAILED")):
            return Palette.MAGENTA
        if any(token in state for token in ("WORKING", "PROGRESS", "DETECTING", "RECHECK")):
            return Palette.ORANGE
        if any(token in state for token in ("READY", "COMPLETE", "SUCCESS")):
            return Palette.GREEN
        return Palette.GREY

    def _badge(
        self,
        image: np.ndarray,
        text: str,
        origin: Tuple[int, int],
        color: Color,
        width: int,
    ) -> None:
        x, y = origin
        cv2.rectangle(image, (x, y), (x + width, y + 30), color, -1)
        cv2.rectangle(image, (x, y), (x + width, y + 30), Palette.BLACK, 2)
        self._put_text(
            image,
            text,
            (x + 8, y + 21),
            0.48,
            Palette.BLACK,
            2,
            max_width=width - 16,
        )

    def _label(
        self,
        image: np.ndarray,
        text: str,
        origin: Tuple[int, int],
        color: Color,
    ) -> None:
        x, y = origin
        text = self._ellipsize(text, 0.58, 1, 285)
        (tw, th), baseline = cv2.getTextSize(
            text,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            1,
        )
        y = int(np.clip(y, self.top_h + 2, self.height - self.bottom_h - th - baseline - 4))
        x = int(np.clip(x, 1, self.video_w - tw - 8))
        overlay = image.copy()
        cv2.rectangle(overlay, (x, y), (x + tw + 8, y + th + baseline + 6), Palette.BLACK, -1)
        cv2.addWeighted(overlay, 0.68, image, 0.32, 0.0, image)
        cv2.rectangle(image, (x, y), (x + tw + 10, y + th + baseline + 8), color, 2)
        cv2.putText(
            image,
            text,
            (x + 5, y + th + 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            Palette.ON_DARK,
            1,
            cv2.LINE_AA,
        )

    def _put_text(
        self,
        image: np.ndarray,
        text: str,
        origin: Tuple[int, int],
        scale: float,
        color: Color,
        thickness: int,
        max_width: Optional[int] = None,
        shadow: bool = False,
    ) -> None:
        text = str(text)
        scale *= 1.15
        if max_width is not None:
            text = self._ellipsize(text, scale, thickness, max_width)
        if shadow:
            cv2.putText(
                image,
                text,
                (origin[0] + 1, origin[1] + 1),
                cv2.FONT_HERSHEY_SIMPLEX,
                scale,
                Palette.BLACK,
                max(2, thickness + 1),
                cv2.LINE_AA,
            )
        cv2.putText(
            image,
            text,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def _put_display_text(
        self,
        image: np.ndarray,
        text: str,
        origin: Tuple[int, int],
        scale: float,
        color: Color,
        thickness: int,
    ) -> None:
        """High-contrast display type inspired by the supplied poster."""
        scale *= 1.15

        cv2.putText(
            image,
            str(text),
            origin,
            cv2.FONT_HERSHEY_DUPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    @staticmethod
    def _ellipsize(text: str, scale: float, thickness: int, max_width: int) -> str:
        if cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0][0] <= max_width:
            return text
        candidate = text
        while candidate:
            candidate = candidate[:-1]
            output = candidate + "..."
            width = cv2.getTextSize(
                output, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness
            )[0][0]
            if width <= max_width:
                return output
        return "..."
