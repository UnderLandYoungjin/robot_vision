"""
OringPathPlanner_V5
챔버 전체 보기 카메라(1920x1080) 이미지에서 슬릿밸브 외곽을 '둥근 사각형 모델'로 피팅하고,
도면 기준 오프셋(외곽→O-ring 중심 거리)으로 매끄러운 로봇 국부 검사 경로를 생성한다.

V5 변경:
  - 카메라 자동 검색·선택: --camera 생략 시 연결된 카메라를 찾아 번호 목록 표시 후 선택
  - --list-cameras : 카메라 목록만 출력 (장치 이름은 pygrabber 설치 시 표시)

V4 변경:
  - 카메라 연결 상태에서 바로 촬영 → 경로 생성 (파일 저장 불필요)
  - 실시간 미리보기 창: [Space] 촬영·경로 생성, [S] 결과 저장, [Q]/[Esc] 종료
  - 이미지 파일 모드도 그대로 지원 (--image)

V3 변경(V2 대비):
  - 울퉁불퉁한 윤곽을 그대로 따라가지 않음 → 외곽(배경 대비가 가장 확실한 경계)만 사용
  - 4변은 강건 직선 피팅(Huber) + 중앙값, 모서리 반경은 자동 추정(또는 도면값 지정)
  - 경로 = 피팅된 외곽을 안쪽으로 오프셋한 이론 형상 → 직선은 직선, 모서리는 정원호
  - 여러 줄(레인) 경로 지원: --inset-mm 4,7.5,11
  - 외곽 피팅 잔차로 국부 돌출/들뜸 후보 위치 표시

사용 예:
    python path.py                                   (카메라 검색 → 목록에서 선택 → 실시간 모드)
    python path.py --list-cameras                    (연결된 카메라 목록만 확인)
    python path.py --camera 1                        (카메라 번호 지정)
    python path.py --image sample.png                (이미지 파일 모드)
    python path.py --image sample.png --length-mm 348 --inset-mm 7.5
    python path.py --inset-mm 4,7.5,11 --corner-r-mm 10
    python path.py --calib calib.json --inset-mm 7.5

calib.json (화소 ↔ 로봇/챔버 좌표 대응점, 4점 이상):
{ "pixel": [[u1,v1],[u2,v2],[u3,v3],[u4,v4]], "world": [[x1,y1],[x2,y2],[x3,y3],[x4,y4]] }

Python 3.12.3 / OpenCV 4.x 이상 / numpy
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np


# ================================================================ 파일 입출력 (한글 경로 대응)
def imread_any(path: Path) -> np.ndarray | None:
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_any(path: Path, img: np.ndarray) -> None:
    ok, buf = cv2.imencode(path.suffix or ".png", img)
    if ok:
        buf.tofile(str(path))


def resolve_image(arg: Path | None) -> Path | None:
    """인자가 없으면 스크립트 폴더의 sample.png → 없으면 파일 선택 창."""
    script_dir = Path(__file__).resolve().parent
    if arg is not None:
        p = arg if arg.is_absolute() else (Path.cwd() / arg)
        if not p.exists():
            p2 = script_dir / arg
            p = p2 if p2.exists() else p
        return p
    default = script_dir / "sample.png"
    if default.exists():
        return default
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        chosen = filedialog.askopenfilename(
            title="전체 보기 이미지 선택",
            initialdir=str(script_dir),
            filetypes=[("Image", "*.png *.jpg *.jpeg *.bmp *.tif *.tiff"), ("All", "*.*")])
        root.destroy()
        return Path(chosen) if chosen else None
    except Exception:
        return None


# ================================================================ 좌표 변환
class PixelToWorld:
    """화소(px) ↔ 챔버/로봇 좌표(mm)."""

    def __init__(self, H: np.ndarray):
        self.H = H
        self.Hinv = np.linalg.inv(H)

    @staticmethod
    def _apply(M: np.ndarray, pts: np.ndarray) -> np.ndarray:
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, M).reshape(-1, 2)

    def to_world(self, px: np.ndarray) -> np.ndarray:
        return self._apply(self.H, px)

    def to_pixel(self, mm: np.ndarray) -> np.ndarray:
        return self._apply(self.Hinv, mm)

    def mm_per_px_at(self, u: float, v: float) -> float:
        """해당 화소 위치의 국부 배율(mm/px)."""
        p = np.array([[u, v], [u + 1, v], [u, v + 1]], dtype=np.float64)
        w = self.to_world(p)
        return float((np.linalg.norm(w[1] - w[0]) + np.linalg.norm(w[2] - w[0])) / 2.0)

    @classmethod
    def from_calib_file(cls, path: Path) -> "PixelToWorld":
        data = json.loads(path.read_text(encoding="utf-8"))
        src = np.array(data["pixel"], dtype=np.float64)
        dst = np.array(data["world"], dtype=np.float64)
        if len(src) < 4 or len(src) != len(dst):
            raise ValueError("calib.json 의 pixel/world 대응점은 4점 이상, 개수 동일해야 합니다.")
        H, _ = cv2.findHomography(src, dst, 0 if len(src) == 4 else cv2.RANSAC)
        if H is None:
            raise ValueError("호모그래피 계산 실패 — 대응점 배치를 확인하세요.")
        return cls(H)

    @classmethod
    def from_scale(cls, mm_per_px: float, origin_px: tuple[float, float]) -> "PixelToWorld":
        x0, y0 = origin_px
        H = np.array([[mm_per_px, 0, -x0 * mm_per_px],
                      [0, mm_per_px, -y0 * mm_per_px],
                      [0, 0, 1]], dtype=np.float64)
        return cls(H)


# ================================================================ 외곽 추출
def make_mask(gray: np.ndarray, thresh: int) -> np.ndarray:
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    if thresh <= 0:
        _, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    else:
        _, mask = cv2.threshold(blur, thresh, 255, cv2.THRESH_BINARY_INV)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    return mask


def largest_outer_contour(mask: np.ndarray) -> np.ndarray:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        raise RuntimeError("외곽을 찾지 못했습니다. --thresh 값을 조정하세요.")
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < mask.shape[0] * mask.shape[1] * 0.01:
        raise RuntimeError("외곽이 너무 작습니다. --thresh 값을 조정하세요.")
    return c.reshape(-1, 2).astype(np.float64)


# ================================================================ 둥근 사각형 모델 피팅
class RoundedRect:
    """화면 좌표계의 둥근 사각형: 중심, 축 방향, 반폭/반높이, 모서리 반경 (모두 px)."""

    def __init__(self, cx, cy, ux, a, b, r):
        self.c = np.array([cx, cy], dtype=np.float64)
        self.ux = np.asarray(ux, dtype=np.float64) / np.linalg.norm(ux)   # 긴 변 방향
        self.uy = np.array([-self.ux[1], self.ux[0]])                      # 짧은 변 방향
        self.a, self.b, self.r = float(a), float(b), float(r)

    def to_local(self, pts: np.ndarray) -> np.ndarray:
        d = pts - self.c
        return np.column_stack([d @ self.ux, d @ self.uy])

    def to_image(self, local: np.ndarray) -> np.ndarray:
        return self.c + local[:, :1] * self.ux + local[:, 1:2] * self.uy

    def signed_distance(self, pts: np.ndarray) -> np.ndarray:
        """외곽 바깥 +, 안쪽 − (px)."""
        p = np.abs(self.to_local(pts))
        q = p - np.array([self.a - self.r, self.b - self.r])
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
        inside = np.minimum(np.maximum(q[:, 0], q[:, 1]), 0.0)
        return outside + inside - self.r

    def offset_polyline(self, inset: float, step: float = 0.5) -> np.ndarray:
        """안쪽으로 inset(px)만큼 오프셋한 경로. 좌상단 직선 시작, 화면 기준 시계방향."""
        a, b = self.a - inset, self.b - inset
        r = max(self.r - inset, 0.5)
        if a <= r or b <= r:
            raise ValueError("오프셋이 너무 커서 경로를 만들 수 없습니다. --inset-mm 을 줄이세요.")
        ax, by = a - r, b - r
        segs = []

        def line(p0, p1):
            n = max(int(np.linalg.norm(np.subtract(p1, p0)) / step), 2)
            t = np.linspace(0, 1, n, endpoint=False)[:, None]
            segs.append(np.asarray(p0) + t * (np.asarray(p1) - np.asarray(p0)))

        def arc(cx, cy, t0, t1):
            n = max(int(abs(t1 - t0) * r / step), 2)
            t = np.linspace(t0, t1, n, endpoint=False)
            segs.append(np.column_stack([cx + r * np.cos(t), cy + r * np.sin(t)]))

        h = np.pi / 2
        line((-ax, -b), (ax, -b));   arc(ax, -by, -h, 0)
        line((a, -by), (a, by));     arc(ax, by, 0, h)
        line((ax, b), (-ax, b));     arc(-ax, by, h, 2 * h)
        line((-a, by), (-a, -by));   arc(-ax, -by, 2 * h, 3 * h)
        return self.to_image(np.vstack(segs))


def fit_rounded_rect(outer: np.ndarray, corner_r_px: float | None) -> RoundedRect:
    # 1) 초기 자세: 최소 외접 사각형
    (cx, cy), (w, h), ang = cv2.minAreaRect(outer.astype(np.float32))
    if w < h:
        w, h = h, w
        ang += 90.0
    t = np.radians(ang)
    rr = RoundedRect(cx, cy, (np.cos(t), np.sin(t)), w / 2, h / 2, 0)

    # 2) 긴 변(위·아래) 중앙부 점으로 방향 재추정 — 모서리·돌출 영향 배제
    loc = rr.to_local(outer)
    mid = np.abs(loc[:, 0]) < 0.35 * w
    dirs = []
    for sel in (mid & (loc[:, 1] < 0), mid & (loc[:, 1] > 0)):
        if sel.sum() > 20:
            vx, vy, _, _ = cv2.fitLine(outer[sel].astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
            v = np.array([vx, vy])
            if v @ rr.ux < 0:
                v = -v
            dirs.append(v)
    if dirs:
        rr = RoundedRect(cx, cy, np.mean(dirs, axis=0), w / 2, h / 2, 0)

    # 3) 4변 위치 = 각 변 중앙부 점들의 중앙값 (튀는 점에 강건)
    loc = rr.to_local(outer)
    mid_x = np.abs(loc[:, 0]) < 0.35 * w
    mid_y = np.abs(loc[:, 1]) < 0.25 * h
    top = np.median(loc[mid_x & (loc[:, 1] < 0), 1])
    bot = np.median(loc[mid_x & (loc[:, 1] > 0), 1])
    left = np.median(loc[mid_y & (loc[:, 0] < 0), 0])
    right = np.median(loc[mid_y & (loc[:, 0] > 0), 0])
    c_local = np.array([[(left + right) / 2, (top + bot) / 2]])
    c_img = rr.to_image(c_local)[0]
    a, b = (right - left) / 2, (bot - top) / 2

    # 4) 모서리 반경: 사각 꼭짓점과 윤곽 최근접 거리 d = r(√2 − 1) 관계로 추정
    if corner_r_px is None:
        rr0 = RoundedRect(c_img[0], c_img[1], rr.ux, a, b, 0)
        loc0 = rr0.to_local(outer)
        rs = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                d = np.min(np.linalg.norm(loc0 - np.array([sx * a, sy * b]), axis=1))
                rs.append(d / (np.sqrt(2) - 1))
        r = float(np.median(rs))
    else:
        r = corner_r_px
    r = float(np.clip(r, 0.5, min(a, b) * 0.95))
    return RoundedRect(c_img[0], c_img[1], rr.ux, a, b, r)


# ================================================================ 경로 처리
def resample_closed(pts: np.ndarray, step: float) -> tuple[np.ndarray, float]:
    p = np.vstack([pts, pts[:1]])
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    n = max(int(np.ceil(total / step)), 4)
    t = np.linspace(0.0, total, n, endpoint=False)
    return np.column_stack([np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])]), total


def tangent_angles(pts: np.ndarray) -> np.ndarray:
    d = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    return np.degrees(np.arctan2(d[:, 1], d[:, 0]))


def fov_corners(center_mm, angle_deg, fov_w, fov_h) -> np.ndarray:
    hw, hh = fov_w / 2.0, fov_h / 2.0
    local = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]])
    r = np.radians(angle_deg)
    R = np.array([[np.cos(r), -np.sin(r)], [np.sin(r), np.cos(r)]])
    return local @ R.T + np.asarray(center_mm)


# ================================================================ 경로 생성 (한 장 처리)
class PlanResult:
    def __init__(self):
        self.ok = False
        self.message = ""
        self.vis = None
        self.mask = None
        self.rows = []
        self.lines = []


def plan_from_image(img: np.ndarray, args, insets_mm: list[float]) -> PlanResult:
    res = PlanResult()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 1) 외곽만 추출
    res.mask = make_mask(gray, args.thresh)
    try:
        outer = largest_outer_contour(res.mask)
    except RuntimeError as e:
        res.message = str(e)
        res.vis = img.copy()
        return res

    # 2) 배율·모델 피팅
    if args.calib:
        conv = PixelToWorld.from_calib_file(args.calib)
        cx, cy = outer.mean(axis=0)
        mm_px = conv.mm_per_px_at(cx, cy)
    else:
        (_, _), (w, h), _ = cv2.minAreaRect(outer.astype(np.float32))
        mm_px = args.length_mm / max(w, h)
        conv = None

    corner_r_px = None if args.corner_r_mm is None else args.corner_r_mm / mm_px
    try:
        rr = fit_rounded_rect(outer, corner_r_px)
    except Exception as e:
        res.message = f"외곽 피팅 실패: {e}"
        res.vis = img.copy()
        return res

    if conv is None:
        mm_px = args.length_mm / (2 * rr.a)
        corner = rr.to_image(np.array([[-rr.a, -rr.b]]))[0]
        conv = PixelToWorld.from_scale(mm_px, (corner[0], corner[1]))

    # 3) 피팅 잔차
    resid_mm = rr.signed_distance(outer) * mm_px
    bump_idx = np.where(np.abs(resid_mm) > args.bump_mm)[0]

    # 4) 레인별 경로
    step_mm = args.fov_w * (1.0 - args.overlap)
    lanes_px, fovs_px, summary = [], [], []
    for lane, inset in enumerate(insets_mm):
        try:
            dense_px = rr.offset_polyline(inset / mm_px)
        except ValueError as e:
            res.message = f"레인 {lane} (inset {inset}mm): {e}"
            res.vis = img.copy()
            return res
        dense_mm = conv.to_world(dense_px)
        path_mm, perim = resample_closed(dense_mm, step_mm)
        angles = tangent_angles(path_mm)
        path_px = conv.to_pixel(path_mm)
        lanes_px.append((dense_px, path_px))
        for i, ((x, y), a, (u, v)) in enumerate(zip(path_mm, angles, path_px)):
            res.rows.append([lane, i, f"{inset:.2f}", f"{x:.3f}", f"{y:.3f}", f"{a:.2f}", f"{u:.1f}", f"{v:.1f}"])
            fovs_px.append(conv.to_pixel(fov_corners((x, y), a, args.fov_w, args.fov_h)))
        summary.append((lane, inset, perim, len(path_mm)))

    # 5) 시각화
    vis = img.copy()
    cv2.polylines(vis, [outer.astype(np.int32)], True, (0, 200, 0), 1, cv2.LINE_AA)
    cv2.polylines(vis, [rr.offset_polyline(0.0).astype(np.int32)], True, (255, 255, 0), 1, cv2.LINE_AA)
    for corners in fovs_px:
        cv2.polylines(vis, [corners.astype(np.int32)], True, (200, 0, 200), 1, cv2.LINE_AA)
    lane_colors = [(0, 220, 255), (0, 140, 255), (0, 255, 140), (255, 160, 0)]
    for k, (dense_px, path_px) in enumerate(lanes_px):
        col = lane_colors[k % len(lane_colors)]
        cv2.polylines(vis, [dense_px.astype(np.int32)], True, col, 1, cv2.LINE_AA)
        for i, (u, v) in enumerate(path_px):
            cv2.circle(vis, (int(round(u)), int(round(v))), 2, (0, 0, 255), -1, cv2.LINE_AA)
            if i % 10 == 0:
                cv2.putText(vis, f"{k}-{i}", (int(u) + 3, int(v) - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA)
    for j in bump_idx[::5]:
        cv2.circle(vis, tuple(outer[j].astype(int)), 6, (0, 0, 255), 1, cv2.LINE_AA)
    res.vis = vis

    res.lines = [
        f"이미지 크기      : {img.shape[1]} x {img.shape[0]} px",
        f"배율             : {mm_px:.4f} mm/px ({'calib.json' if args.calib else f'긴 변 {args.length_mm}mm 기준'})",
        f"피팅 외곽        : {2*rr.a*mm_px:.2f} x {2*rr.b*mm_px:.2f} mm, R {rr.r*mm_px:.2f} mm, "
        f"기울기 {np.degrees(np.arctan2(rr.ux[1], rr.ux[0])):.2f} deg",
        f"피팅 잔차        : RMS {np.sqrt(np.mean(resid_mm**2)):.3f} / 최대 {np.max(np.abs(resid_mm)):.3f} mm, "
        f"{args.bump_mm}mm 초과 {len(bump_idx)}점",
        f"촬영 간격        : {step_mm:.2f} mm (시야 {args.fov_w}mm, 겹침 {args.overlap*100:.0f}%)",
    ]
    for lane, inset, perim, n in summary:
        res.lines.append(f"  레인 {lane}: inset {inset:.2f}mm, 둘레 {perim:.1f}mm, 촬영 {n}개")
    res.lines.append(f"총 촬영 위치     : {len(res.rows)} 개")
    res.ok = True
    res.message = "OK"
    return res


def save_result(res: PlanResult, raw: np.ndarray, out_dir: Path, tag: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    imwrite_any(out_dir / f"{tag}_raw.png", raw)
    if res.vis is not None:
        imwrite_any(out_dir / f"{tag}_overlay.png", res.vis)
    if res.mask is not None:
        imwrite_any(out_dir / f"{tag}_mask.png", res.mask)
    if res.rows:
        with (out_dir / f"{tag}_robot_path.csv").open("w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["lane", "index", "inset_mm", "x_mm", "y_mm", "angle_deg", "u_px", "v_px"])
            w.writerows(res.rows)
    print(f"저장 완료 → {out_dir.resolve()} ({tag}_*)")


# ================================================================ 카메라
def open_camera(index: int, width: int, height: int) -> cv2.VideoCapture | None:
    backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY] if sys.platform.startswith("win") else [cv2.CAP_ANY]
    for be in backends:
        cap = cv2.VideoCapture(index, be)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, frame = cap.read()
        if ok and frame is not None:
            return cap
        cap.release()
    return None


def camera_names() -> list[str]:
    """Windows DirectShow 장치 이름 (pygrabber 설치 시). 순서 = OpenCV DSHOW 번호."""
    try:
        from pygrabber.dshow_graph import FilterGraph
        return list(FilterGraph().get_input_devices())
    except Exception:
        return []


def scan_cameras(max_index: int = 8) -> list[tuple[int, int, int]]:
    """0~max_index 번을 열어 보고 (번호, 가로, 세로) 목록 반환."""
    found = []
    be = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
    for i in range(max_index + 1):
        cap = cv2.VideoCapture(i, be)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                found.append((i, frame.shape[1], frame.shape[0]))
        cap.release()
    return found


def choose_camera(max_index: int = 8) -> int | None:
    print("카메라 검색 중...")
    cams = scan_cameras(max_index)
    names = camera_names()
    if not cams:
        print("연결된 카메라가 없습니다. USB 연결과 다른 프로그램(cam.py 등) 사용 여부를 확인하세요.")
        return None
    print("=== 연결된 카메라 ===")
    for i, w, h in cams:
        name = names[i] if i < len(names) else "(이름 확인: pip install pygrabber)"
        print(f"  [{i}] {name}  기본 {w}x{h}")
    if len(cams) == 1:
        print(f"카메라 1대 → [{cams[0][0]}] 자동 선택")
        return cams[0][0]
    valid = {c[0] for c in cams}
    while True:
        sel = input(f"사용할 카메라 번호 입력 {sorted(valid)} (Enter = {cams[0][0]}): ").strip()
        if sel == "":
            return cams[0][0]
        if sel.isdigit() and int(sel) in valid:
            return int(sel)
        print("목록에 있는 번호를 입력하세요.")


def grab_stable(cap: cv2.VideoCapture, skip: int = 5) -> np.ndarray | None:
    """버퍼에 남은 이전 프레임을 버리고 최신 프레임을 받는다."""
    frame = None
    for _ in range(skip):
        ok, f = cap.read()
        if ok:
            frame = f
    return frame


def fit_to_screen(img: np.ndarray, max_w: int = 1600, max_h: int = 900) -> np.ndarray:
    h, w = img.shape[:2]
    s = min(max_w / w, max_h / h, 1.0)
    return img if s >= 1.0 else cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)


def put_info(img: np.ndarray, text: str, color=(0, 255, 255)) -> None:
    cv2.rectangle(img, (0, 0), (img.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(img, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)


def run_camera(args, insets_mm: list[float]) -> int:
    if args.camera is None:
        args.camera = choose_camera()
        if args.camera is None:
            return 1
    cap = open_camera(args.camera, args.cam_width, args.cam_height)
    if cap is None:
        print(f"오류: 카메라 {args.camera}번을 열 수 없습니다. --camera 1 처럼 번호를 바꿔 보세요.")
        return 1
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"카메라 {args.camera}번 연결: {real_w} x {real_h}")
    if (real_w, real_h) != (args.cam_width, args.cam_height):
        print(f"  주의: 요청 해상도 {args.cam_width}x{args.cam_height} 와 다릅니다.")
    print("[Space] 촬영·경로 생성   [S] 결과 저장   [L] 미리보기로 복귀   [Q]/[Esc] 종료")

    out_dir = args.out or Path(__file__).resolve().parent / "output"
    win = "OringPathPlanner_V5"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    live = True
    last_raw, last_res = None, None
    try:
        while True:
            if live:
                ok, frame = cap.read()
                if not ok or frame is None:
                    print("오류: 카메라 프레임을 읽지 못했습니다.")
                    break
                disp = fit_to_screen(frame.copy())
                put_info(disp, f"LIVE {real_w}x{real_h}  [Space] Plan  [Q] Quit")
            else:
                disp = fit_to_screen(last_res.vis.copy())
                if last_res.ok:
                    put_info(disp, f"OK  points {len(last_res.rows)}  [S] Save  [L] Live  [Space] Retake",
                             (0, 255, 0))
                else:
                    put_info(disp, "FAIL (see console)  [L] Live  [Space] Retake", (0, 0, 255))
            cv2.imshow(win, disp)

            key = cv2.waitKey(1 if live else 30) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord(" "):
                raw = grab_stable(cap)
                if raw is None:
                    print("오류: 촬영 실패")
                    continue
                last_raw = raw
                last_res = plan_from_image(raw, args, insets_mm)
                print("-" * 60)
                if last_res.ok:
                    print("\n".join(last_res.lines))
                else:
                    print(f"경로 생성 실패: {last_res.message}")
                live = False
            elif key in (ord("s"), ord("S")) and last_res is not None:
                tag = time.strftime("%Y%m%d_%H%M%S")
                save_result(last_res, last_raw, out_dir, tag)
            elif key in (ord("l"), ord("L")):
                live = True
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return 0


def run_image(args, insets_mm: list[float]) -> int:
    image_path = resolve_image(args.image)
    if image_path is None:
        print("오류: 이미지가 지정되지 않았습니다. 예) python path.py --image sample.png")
        return 1
    img = imread_any(image_path)
    if img is None:
        print(f"오류: 이미지를 열 수 없습니다 → {image_path}")
        return 1
    res = plan_from_image(img, args, insets_mm)
    print(f"입력 이미지      : {image_path}")
    if not res.ok:
        print(f"경로 생성 실패: {res.message}")
        return 1
    print("\n".join(res.lines))
    out_dir = args.out or image_path.parent / "output"
    save_result(res, img, out_dir, image_path.stem)
    if not args.no_window:
        cv2.namedWindow("OringPathPlanner_V5", cv2.WINDOW_NORMAL)
        cv2.imshow("OringPathPlanner_V5", fit_to_screen(res.vis))
        print("아무 키나 누르면 창이 닫힙니다.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0


# ================================================================ 메인
def main() -> int:
    ap = argparse.ArgumentParser(description="O-ring 국부 검사 로봇 경로 생성 (V4, 카메라 실시간/이미지)")
    ap.add_argument("--image", type=Path, default=None, help="이미지 파일 모드 (생략 시 카메라 모드)")
    ap.add_argument("--camera", type=int, default=None, help="카메라 번호 (생략 시 검색 후 선택)")
    ap.add_argument("--list-cameras", action="store_true", help="연결된 카메라 목록만 출력")
    ap.add_argument("--cam-width", type=int, default=1920, help="카메라 요청 가로 해상도")
    ap.add_argument("--cam-height", type=int, default=1080, help="카메라 요청 세로 해상도")
    ap.add_argument("--calib", type=Path, help="화소↔mm 대응점 파일(calib.json)")
    ap.add_argument("--length-mm", type=float, default=348.0, help="보정 파일 없을 때 외곽 긴 변 실치수(mm)")
    ap.add_argument("--inset-mm", type=str, default="7.5", help="외곽→경로 안쪽 거리(mm), 여러 줄: 4,7.5,11")
    ap.add_argument("--corner-r-mm", type=float, default=None, help="외곽 모서리 반경(mm, 생략 시 자동)")
    ap.add_argument("--fov-w", type=float, default=5.9, help="국부 시야 가로(mm)")
    ap.add_argument("--fov-h", type=float, default=4.4, help="국부 시야 세로(mm)")
    ap.add_argument("--overlap", type=float, default=0.15, help="인접 촬영 겹침 비율(0~0.9)")
    ap.add_argument("--bump-mm", type=float, default=0.8, help="외곽 피팅 잔차 경고 기준(mm)")
    ap.add_argument("--thresh", type=int, default=0, help="이진화 임계값(0=자동 Otsu)")
    ap.add_argument("--out", type=Path, default=None, help="저장 폴더(기본: 스크립트 폴더/output)")
    ap.add_argument("--no-window", action="store_true", help="이미지 모드에서 결과 창 표시 안 함")
    args = ap.parse_args()

    if not 0.0 <= args.overlap < 0.9:
        print("오류: --overlap 은 0 이상 0.9 미만이어야 합니다.")
        return 1
    try:
        insets_mm = [float(v) for v in args.inset_mm.split(",") if v.strip()]
    except ValueError:
        print("오류: --inset-mm 형식 예) 7.5 또는 4,7.5,11")
        return 1
    if not insets_mm:
        print("오류: --inset-mm 값이 비어 있습니다.")
        return 1

    if args.list_cameras:
        cams = scan_cameras()
        names = camera_names()
        if not cams:
            print("연결된 카메라가 없습니다.")
        for i, w, h in cams:
            name = names[i] if i < len(names) else "(이름 확인: pip install pygrabber)"
            print(f"[{i}] {name}  기본 {w}x{h}")
        return 0

    if args.image is not None:
        return run_image(args, insets_mm)
    return run_camera(args, insets_mm)


if __name__ == "__main__":
    sys.exit(main())
