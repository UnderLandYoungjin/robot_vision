"""
OringDefectPoC_V3
O-ring 국부(확대) 이미지 결함 검출 PoC (Proof of Concept, 개념 검증)

V3 변경:
  - 화면 좌우 분할: 왼쪽 원본 영상, 오른쪽 검사 결과 시각화 (실시간·이미지 뷰어 공통)

V2 변경 (실시간 검사):
  - 인자 없이 실행하면 카메라 검색 → 선택 → 실시간 검사
  - 영상 표시와 검사를 분리(별도 스레드) → 화면은 끊김 없이, 검사는 가능한 최대 속도
  - 처리 해상도 축소(--scale), 방향 각도 캐시(N프레임마다 재계산)로 속도 향상
  - 판정 안정화: 최근 N회 중 M회 이상 NG일 때 NG 표시 (깜빡임 방지)
  - NG 자동 저장(--save-ng), 일시정지(Space), FPS 표시

판정 기준 (2026.11 협의 기준, 목표 사양):
  - 선형 결함(크랙·실밥·스크래치): 선으로 검출되면 길이 무관 NG
  - 덩어리형 결함(이물·찍힘·결손): 최대 크기 0.3mm 초과 NG
  - 그 이하 점(먼지): OK

처리 방식:
  1) O-ring 진행 방향(반사 띠 방향) 자동 추정 → 영상을 회전해 띠를 수평으로 정렬
  2) 수평 방향 긴 커널의 Top-hat / Black-hat 으로 '띠는 남기고 국부 이상만' 추출
  3) 잡음 수준(MAD, Median Absolute Deviation 중앙값 절대편차) 기반 자동 임계값
  4) 영역별 세장비(길이÷폭)로 선/덩어리 분류, 크기는 mm 환산

사용 예:
  python oring_defect_poc.py                         (카메라 검색·선택 → 실시간 검사)
  python oring_defect_poc.py --camera 1 --scale 0.5 --save-ng
  python oring_defect_poc.py --list-cameras
  python oring_defect_poc.py --folder samples        (이미지 폴더 뷰어)
  python oring_defect_poc.py --folder D:/oring_imgs
  python oring_defect_poc.py --image 05.jpg
  python oring_defect_poc.py --batch                 (창 없이 폴더 전체 판정 → output 저장)

조작키 (실시간):
  Space : 일시정지/재개     D : 먼지 표시     M : 응답 맵     S : 현재 화면 저장     Q / Esc : 종료

조작키 (이미지 뷰어):
  N / → : 다음 이미지     P / ← : 이전 이미지
  D     : 먼지(OK) 표시 켜기/끄기
  M     : 응답 맵(검출 근거) 보기 전환
  S     : 결과 저장       Q / Esc : 종료

Python 3.12.3 / OpenCV 4.x / numpy
"""

import argparse
import csv
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np

IMG_EXT = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
WIN = "OringDefectPoC_V3"


# ================================================================ 파라미터
@dataclass
class Params:
    um_per_px: float = 8.0        # 물체 쪽 1화소 크기(µm) — 장비 배율 확정 후 고정값 입력
    k_sigma: float = 4.0          # 성장 임계값 = 중앙값 + k × 잡음
    k_seed: float = 12.0          # 씨앗 임계값 — 이 이상 강한 화소가 있는 영역만 결함 후보
    line_aspect: float = 6.0      # 세장비 이상이면 '선'
    line_min_mm: float = 0.15     # 이보다 짧은 선형 영역은 점(먼지)으로 처리
    blob_mm: float = 0.30         # 덩어리 NG 기준
    bg_len_mm: float = 1.5        # 배경 추정 길이(띠 방향) — 검출할 덩어리 크기보다 충분히 커야 함
    min_area_px: int = 12         # 이보다 작은 점은 무시
    sat_foreign: int = 60         # 채도 이상이면 '유색 이물' 표기

    @property
    def mm_per_px(self) -> float:
        return self.um_per_px / 1000.0


# ================================================================ 입출력
def imread_any(path: Path) -> np.ndarray | None:
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR) if data.size else None


def imwrite_any(path: Path, img: np.ndarray) -> None:
    ok, buf = cv2.imencode(path.suffix or ".png", img)
    if ok:
        buf.tofile(str(path))


def list_images(folder: Path) -> list[Path]:
    return sorted(p for p in folder.iterdir() if p.suffix.lower() in IMG_EXT)


def ask_folder(initial: Path) -> Path | None:
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        d = filedialog.askdirectory(title="검사 이미지 폴더 선택", initialdir=str(initial))
        root.destroy()
        return Path(d) if d else None
    except Exception:
        return None


# ================================================================ 결과 구조
@dataclass
class Defect:
    kind: str           # LINE / BLOB / DUST
    ng: bool
    length_mm: float
    width_mm: float
    area_mm2: float
    elong: float
    colored: bool
    contour: np.ndarray  # 원본 좌표


@dataclass
class Result:
    ng: bool
    defects: list
    angle_deg: float
    threshold: float
    response: np.ndarray    # 원본 좌표 응답 맵(0~255)


# ================================================================ 핵심 처리
def ring_angle(gray: np.ndarray) -> float:
    """구조 텐서로 반사 띠(O-ring 진행) 방향 각도(deg) 추정."""
    small = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    g = cv2.GaussianBlur(small, (0, 0), 3).astype(np.float32)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    jxx, jyy, jxy = float((gx * gx).sum()), float((gy * gy).sum()), float((gx * gy).sum())
    grad_dir = 0.5 * np.degrees(np.arctan2(2 * jxy, jxx - jyy))   # 밝기 변화 방향
    return grad_dir + 90.0                                          # 띠 방향 = 수직


def rotate_bound(img: np.ndarray, angle_deg: float, border=0):
    """띠 방향이 수평이 되도록 회전(잘림 없이). 회전 행렬과 역행렬 반환."""
    h, w = img.shape[:2]
    c = (w / 2.0, h / 2.0)
    M = cv2.getRotationMatrix2D(c, angle_deg, 1.0)
    cos, sin = abs(M[0, 0]), abs(M[0, 1])
    nw, nh = int(h * sin + w * cos), int(h * cos + w * sin)
    M[0, 2] += nw / 2.0 - c[0]
    M[1, 2] += nh / 2.0 - c[1]
    out = cv2.warpAffine(img, M, (nw, nh), flags=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_REPLICATE if border is None else cv2.BORDER_CONSTANT,
                         borderValue=0 if border is None else border)
    Minv = cv2.invertAffineTransform(M)
    return out, M, Minv


def inspect(img: np.ndarray, p: Params, fixed_angle: float | None = None) -> Result:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)

    # 1) 띠 방향 → 수평 정렬 (회전 각도 부호 자동 확인)
    ang = ring_angle(gray) if fixed_angle is None else fixed_angle
    cands = (ang,) if fixed_angle is not None else (ang, -ang)
    best = None
    for cand in cands:
        rot, M, Minv = rotate_bound(gray, cand, border=None)
        rs = cv2.resize(rot, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA).astype(np.float32)
        score = float(np.var(rs.mean(axis=1)))          # 수평 정렬일수록 행 평균 분산 큼
        if best is None or score > best[0]:
            best = (score, cand, rot, M, Minv)
    _, ang, rot, M, Minv = best
    valid = cv2.warpAffine(np.full(gray.shape, 255, np.uint8), M, (rot.shape[1], rot.shape[0]))
    valid = cv2.erode(valid, np.ones((9, 9), np.uint8))
    color_rot = cv2.warpAffine(img, M, (rot.shape[1], rot.shape[0]), borderMode=cv2.BORDER_REPLICATE)

    # 2) 띠 방향(수평) 1차원 평균으로 배경 추정 → 띠·밝기 기울기는 그대로 따라가고 국부 이상만 차이로 남음
    k = int(round(p.bg_len_mm / p.mm_per_px))
    k = int(np.clip(k, 15, max(15, rot.shape[1] // 2)))
    k += (k + 1) % 2
    rot_f = rot.astype(np.float32)
    bg = cv2.blur(rot_f, (k, 1), borderType=cv2.BORDER_REFLECT)
    # 2회 반복: 1차 차이가 큰 화소(결함 후보)를 배경으로 대체한 뒤 다시 평균 → 결함이 배경을 끌어올리는 영향 제거
    diff0 = np.abs(rot_f - bg)
    robust = np.where(diff0 > 3 * (np.median(diff0) * 1.4826 + 1e-6), bg, rot_f)
    bg = cv2.blur(robust, (k, 1), borderType=cv2.BORDER_REFLECT)
    resp = np.abs(rot_f - bg)
    # 띠 양 끝(원본 영상 위·아래 가장자리)은 평균 창이 잘려 배경 추정이 부정확 → 검사 제외
    valid = cv2.erode(valid, cv2.getStructuringElement(cv2.MORPH_RECT, (k // 2 * 2 + 1, 1)))
    resp = cv2.GaussianBlur(resp, (0, 0), 1.2)

    # 3) 자동 임계값 (잡음 = MAD) + 히스테리시스: 약한 영역은 강한 씨앗과 연결될 때만 채택
    vals = resp[valid > 0]
    med = float(np.median(vals))
    mad = float(np.median(np.abs(vals - med))) * 1.4826 + 1e-6
    thr = max(med + p.k_sigma * mad, 5.0)
    thr_seed = max(med + p.k_seed * mad, 12.0)
    weak = ((resp > thr) & (valid > 0)).astype(np.uint8)
    strong = (resp > thr_seed) & (valid > 0)
    n_lab, labels = cv2.connectedComponents(weak, connectivity=8)
    keep = np.zeros(n_lab, bool)
    keep[np.unique(labels[strong])] = True
    keep[0] = False
    binm = keep[labels].astype(np.uint8) * 255

    # 4) 영역 분류
    hsv = cv2.cvtColor(color_rot, cv2.COLOR_BGR2HSV)
    contours, _ = cv2.findContours(binm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    defects = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < p.min_area_px:
            continue
        x0, y0, bw, bh = cv2.boundingRect(c)
        m_local = np.zeros((bh + 2, bw + 2), np.uint8)
        cv2.drawContours(m_local, [c - [x0 - 1, y0 - 1]], -1, 255, -1)
        dist = cv2.distanceTransform(m_local, cv2.DIST_L2, 3)
        dvals = dist[m_local > 0]
        # 폭 = 거리변환 상위값×2 (선은 가늘어 작고, 덩어리는 커짐), 길이 = 면적÷폭 (휘어진 실밥도 실제 길이로 계산)
        width_px = max(2.0 * float(np.percentile(dvals, 90)), 1.5)
        length_px = max(area / width_px, 1.0)
        (_, _), (rw, rh), _ = cv2.minAreaRect(c)
        rect_len = max(rw, rh)
        elong = length_px / width_px
        length_mm = length_px * p.mm_per_px
        eq_diam_px = float(np.sqrt(4.0 * area / np.pi))           # 면적 환산 지름 — 엉성한 먼지 뭉치 과대평가 방지
        size_mm = max(eq_diam_px, 0.7 * rect_len) * p.mm_per_px
        if elong >= p.line_aspect and length_mm >= p.line_min_mm:
            kind, ng = "LINE", True
        elif size_mm > p.blob_mm:
            kind, ng = "BLOB", True
        else:
            kind, ng = "DUST", False

        m = np.zeros(binm.shape, np.uint8)
        cv2.drawContours(m, [c], -1, 255, -1)
        colored = cv2.mean(hsv[:, :, 1], mask=m)[0] > p.sat_foreign

        pts = c.reshape(-1, 1, 2).astype(np.float32)
        back = cv2.transform(pts, Minv).astype(np.int32)
        defects.append(Defect(kind, ng, length_mm if kind == "LINE" else size_mm,
                              width_px * p.mm_per_px, area * p.mm_per_px ** 2,
                              float(elong), bool(colored), back))

    resp_orig = cv2.warpAffine(np.clip(resp * (255.0 / max(thr * 3, 1)), 0, 255).astype(np.uint8),
                               Minv, (img.shape[1], img.shape[0]))
    defects.sort(key=lambda d: (not d.ng, d.kind != "LINE", -d.length_mm))
    return Result(any(d.ng for d in defects), defects, float(ang), thr, resp_orig)


# ================================================================ 표시
def draw(img: np.ndarray, res: Result, p: Params, show_dust: bool, show_map: bool, name: str) -> np.ndarray:
    if show_map:
        vis = cv2.applyColorMap(res.response, cv2.COLORMAP_JET)
    else:
        vis = img.copy()
    t = max(1, img.shape[1] // 600)
    n_ng = 0
    for d in res.defects:
        if d.kind == "DUST" and not show_dust:
            continue
        col = (0, 0, 255) if d.kind == "LINE" else (0, 140, 255) if d.kind == "BLOB" else (160, 160, 160)
        cv2.drawContours(vis, [d.contour], -1, col, t if d.ng else 1, cv2.LINE_AA)
        if d.ng:
            n_ng += 1
            x, y, w, h = cv2.boundingRect(d.contour)
            pad = 6 * t
            cv2.rectangle(vis, (x - pad, y - pad), (x + w + pad, y + h + pad), col, t)
            tag = f"#{n_ng} {d.kind}{'(color)' if d.colored else ''} {d.length_mm:.2f}mm"
            cv2.putText(vis, tag, (x - pad, max(y - pad - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45 * t, col, t, cv2.LINE_AA)

    # 0.3mm 기준 원 + 0.01mm 기준선 (우하단)
    h, w = vis.shape[:2]
    r = int(round(p.blob_mm / p.mm_per_px / 2))
    cx, cy = w - r - 20, h - r - 40
    cv2.circle(vis, (cx, cy), max(r, 2), (0, 255, 255), 1, cv2.LINE_AA)
    lw = max(int(round(0.01 / p.mm_per_px)), 1)
    cv2.line(vis, (cx - r, cy + r + 15), (cx + r, cy + r + 15), (0, 255, 255), lw)
    cv2.putText(vis, f"{p.blob_mm:.2f}mm / 0.01mm", (cx - r - 40, cy + r + 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4 * t, (0, 255, 255), 1, cv2.LINE_AA)

    bar = 34 * t
    cv2.rectangle(vis, (0, 0), (w, bar), (0, 0, 0), -1)
    verdict, vcol = ("NG", (0, 0, 255)) if res.ng else ("OK", (0, 220, 0))
    lines = sum(1 for d in res.defects if d.kind == "LINE")
    blobs = sum(1 for d in res.defects if d.kind == "BLOB")
    dust = sum(1 for d in res.defects if d.kind == "DUST")
    cv2.putText(vis, verdict, (8, int(26 * t)), cv2.FONT_HERSHEY_SIMPLEX, 0.9 * t, vcol, 2 * t, cv2.LINE_AA)
    info = f"{name}  LINE {lines}  BLOB {blobs}  dust {dust}  {p.um_per_px:.1f}um/px  angle {res.angle_deg:.1f}"
    cv2.putText(vis, info, (int(70 * t), int(23 * t)), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * t,
                (255, 255, 255), 1, cv2.LINE_AA)
    return vis


def side_by_side(original: np.ndarray, result: np.ndarray, gap: int = 6) -> np.ndarray:
    """왼쪽 원본 | 오른쪽 결과. 두 영상 높이를 맞춰 가로로 붙인다."""
    h = result.shape[0]
    if original.shape[0] != h:
        s = h / original.shape[0]
        original = cv2.resize(original, (int(original.shape[1] * s), h), interpolation=cv2.INTER_AREA)
    left = original.copy()
    t = max(1, left.shape[1] // 600)
    bar = 34 * t
    cv2.rectangle(left, (0, 0), (left.shape[1], bar), (0, 0, 0), -1)
    cv2.putText(left, "ORIGINAL", (8, int(24 * t)), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * t,
                (255, 255, 255), max(1, t), cv2.LINE_AA)
    sep = np.full((h, gap, 3), 60, np.uint8)
    return cv2.hconcat([left, sep, result])


def fit_screen(img, max_w=1800, max_h=900):
    h, w = img.shape[:2]
    s = min(max_w / w, max_h / h, 1.0)
    return img if s >= 1 else cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)


def report(name: str, res: Result) -> None:
    ngs = [d for d in res.defects if d.ng]
    print(f"[{'NG' if res.ng else 'OK'}] {name}  선 {sum(d.kind=='LINE' for d in ngs)}  "
          f"덩어리 {sum(d.kind=='BLOB' for d in ngs)}  먼지 {sum(d.kind=='DUST' for d in res.defects)}")
    for i, d in enumerate(ngs, 1):
        kind = "선형(크랙/실밥)" if d.kind == "LINE" else "덩어리(이물/찍힘)"
        print(f"    #{i} {kind}{' 유색' if d.colored else ''}  길이 {d.length_mm:.3f}mm  "
              f"폭 {d.width_mm:.3f}mm  세장비 {d.elong:.1f}")


def save(out_dir: Path, name: str, img, res: Result, p: Params) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(name).stem
    vis = draw(img, res, p, True, False, name)
    imwrite_any(out_dir / f"{stem}_result.png", vis)
    imwrite_any(out_dir / f"{stem}_compare.png", side_by_side(img, vis))
    imwrite_any(out_dir / f"{stem}_map.png", cv2.applyColorMap(res.response, cv2.COLORMAP_JET))
    with (out_dir / f"{stem}_defects.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["no", "kind", "ng", "length_mm", "width_mm", "area_mm2", "elong", "colored", "x", "y"])
        for i, d in enumerate(res.defects, 1):
            x, y, _, _ = cv2.boundingRect(d.contour)
            w.writerow([i, d.kind, int(d.ng), f"{d.length_mm:.4f}", f"{d.width_mm:.4f}",
                        f"{d.area_mm2:.5f}", f"{d.elong:.2f}", int(d.colored), x, y])


# ================================================================ 트랙바
TB = [  # (이름, 최대, 파라미터→값, 값→파라미터)
    ("um/px x10", 500, lambda p: int(p.um_per_px * 10), lambda p, v: replace(p, um_per_px=max(v, 1) / 10)),
    ("k x10", 150, lambda p: int(p.k_sigma * 10), lambda p, v: replace(p, k_sigma=max(v, 5) / 10)),
    ("seed k", 40, lambda p: int(p.k_seed), lambda p, v: replace(p, k_seed=float(max(v, 2)))),
    ("line asp x10", 200, lambda p: int(p.line_aspect * 10), lambda p, v: replace(p, line_aspect=max(v, 20) / 10)),
    ("blob um", 2000, lambda p: int(p.blob_mm * 1000), lambda p, v: replace(p, blob_mm=max(v, 10) / 1000)),
    ("bg len x0.1mm", 100, lambda p: int(p.bg_len_mm * 10), lambda p, v: replace(p, bg_len_mm=max(v, 3) / 10)),
]


def make_trackbars(p: Params) -> None:
    for name, mx, get, _ in TB:
        cv2.createTrackbar(name, WIN, min(get(p), mx), mx, lambda v: None)


def read_trackbars(p: Params) -> Params:
    for name, _, _, setter in TB:
        p = setter(p, cv2.getTrackbarPos(name, WIN))
    return p


# ================================================================ 실행 모드
def run_viewer(paths: list[Path], p: Params, out_dir: Path) -> int:
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    make_trackbars(p)
    idx, show_dust, show_map = 0, False, False
    cache_key, img, res = None, None, None
    print("N/P 이미지 이동, D 먼지 표시, M 응답맵, S 저장, Q 종료")
    while True:
        p = read_trackbars(p)
        key_now = (idx, p)
        if key_now != cache_key:
            img = imread_any(paths[idx])
            if img is None:
                print(f"읽기 실패: {paths[idx]}")
                paths.pop(idx)
                if not paths:
                    return 1
                idx %= len(paths)
                continue
            res = inspect(img, p)
            report(paths[idx].name, res)
            cache_key = key_now
        vis = draw(img, res, p, show_dust, show_map, f"[{idx+1}/{len(paths)}] {paths[idx].name}")
        cv2.imshow(WIN, fit_screen(side_by_side(img, vis)))
        k = cv2.waitKeyEx(50)
        if k in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            break
        if k in (ord("n"), ord("N"), 2555904, 65363):
            idx = (idx + 1) % len(paths)
        elif k in (ord("p"), ord("P"), 2424832, 65361):
            idx = (idx - 1) % len(paths)
        elif k in (ord("d"), ord("D")):
            show_dust = not show_dust
        elif k in (ord("m"), ord("M")):
            show_map = not show_map
        elif k in (ord("s"), ord("S")):
            save(out_dir, paths[idx].name, img, res, p)
            print(f"저장 → {out_dir.resolve()}")
    cv2.destroyAllWindows()
    return 0


def run_batch(paths: list[Path], p: Params, out_dir: Path) -> int:
    rows = []
    for path in paths:
        img = imread_any(path)
        if img is None:
            print(f"읽기 실패: {path}")
            continue
        res = inspect(img, p)
        report(path.name, res)
        save(out_dir, path.name, img, res, p)
        ngs = [d for d in res.defects if d.ng]
        rows.append([path.name, "NG" if res.ng else "OK",
                     sum(d.kind == "LINE" for d in ngs), sum(d.kind == "BLOB" for d in ngs),
                     f"{max((d.length_mm for d in ngs), default=0):.3f}"])
    with (out_dir / "summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["image", "result", "line_ng", "blob_ng", "max_size_mm"])
        w.writerows(rows)
    print(f"\n총 {len(rows)}장  NG {sum(r[1]=='NG' for r in rows)}  OK {sum(r[1]=='OK' for r in rows)}")
    print(f"결과 저장 → {out_dir.resolve()}")
    return 0


def camera_names() -> list[str]:
    try:
        from pygrabber.dshow_graph import FilterGraph
        return list(FilterGraph().get_input_devices())
    except Exception:
        return []


def scan_cameras(max_index: int = 8) -> list[tuple[int, int, int]]:
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


def choose_camera() -> int | None:
    print("카메라 검색 중...")
    cams, names = scan_cameras(), camera_names()
    if not cams:
        print("연결된 카메라가 없습니다. USB 연결과 다른 프로그램의 카메라 사용 여부를 확인하세요.")
        return None
    for i, w, h in cams:
        name = names[i] if i < len(names) else "(이름 표시: pip install pygrabber)"
        print(f"  [{i}] {name}  기본 {w}x{h}")
    if len(cams) == 1:
        print(f"카메라 1대 → [{cams[0][0]}] 자동 선택")
        return cams[0][0]
    valid = {c[0] for c in cams}
    while True:
        sel = input(f"카메라 번호 입력 {sorted(valid)} (Enter = {cams[0][0]}): ").strip()
        if sel == "":
            return cams[0][0]
        if sel.isdigit() and int(sel) in valid:
            return int(sel)


def open_camera(index: int, width: int, height: int):
    backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY] if sys.platform.startswith("win") else [cv2.CAP_ANY]
    for be in backends:
        cap = cv2.VideoCapture(index, be)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, f = cap.read()
        if ok and f is not None:
            return cap
        cap.release()
    return None


def scale_result(res: Result, s: float, full_shape) -> Result:
    """축소 해상도 결과를 원본 해상도 좌표로 환산."""
    if s == 1.0:
        return res
    for d in res.defects:
        d.contour = (d.contour.astype(np.float32) / s).astype(np.int32)
    res.response = cv2.resize(res.response, (full_shape[1], full_shape[0]), interpolation=cv2.INTER_LINEAR)
    return res


class Inspector(threading.Thread):
    """최신 프레임만 가져가 검사하는 작업 스레드 (밀린 프레임은 버림)."""

    def __init__(self, scale: float, angle_every: int):
        super().__init__(daemon=True)
        self.scale = scale
        self.angle_every = max(angle_every, 1)
        self.params = Params()
        self.lock = threading.Lock()
        self.frame = None
        self.result = None          # (frame, Result)
        self.fps = 0.0
        self.running = True
        self.count = 0
        self.angle = None

    def submit(self, frame, params: Params):
        with self.lock:
            self.frame = frame
            self.params = params

    def latest(self):
        with self.lock:
            return self.result

    def run(self):
        t_last = time.time()
        while self.running:
            with self.lock:
                frame, p = self.frame, self.params
                self.frame = None
            if frame is None:
                time.sleep(0.003)
                continue
            s = self.scale
            small = frame if s == 1.0 else cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            ps = replace(p, um_per_px=p.um_per_px / s)
            use_angle = None if (self.angle is None or self.count % self.angle_every == 0) else self.angle
            try:
                res = inspect(small, ps, fixed_angle=use_angle)
            except Exception as e:
                print(f"검사 오류: {e}")
                continue
            self.angle = res.angle_deg
            self.count += 1
            res = scale_result(res, s, frame.shape)
            now = time.time()
            self.fps = 0.8 * self.fps + 0.2 * (1.0 / max(now - t_last, 1e-3))
            t_last = now
            with self.lock:
                self.result = (frame, res)


def run_camera(index: int | None, p: Params, out_dir: Path, args) -> int:
    if index is None:
        index = choose_camera()
        if index is None:
            return 1
    cap = open_camera(index, args.cam_width, args.cam_height)
    if cap is None:
        print(f"오류: 카메라 {index}번을 열 수 없습니다.")
        return 1
    rw, rh = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"카메라 {index}번: {rw}x{rh}, 처리 배율 {args.scale}")
    print("Space 일시정지, D 먼지, M 응답맵, S 저장, Q 종료")

    worker = Inspector(args.scale, args.angle_every)
    worker.start()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    make_trackbars(p)

    history = deque(maxlen=args.stable_n)
    show_dust, show_map, paused = False, False, False
    last_seen_id, last_save = None, 0.0
    cam_fps, t_prev = 0.0, time.time()
    frame = None
    try:
        while True:
            if not paused:
                ok, f = cap.read()
                if not ok or f is None:
                    print("오류: 프레임 읽기 실패")
                    break
                frame = f
                now = time.time()
                cam_fps = 0.9 * cam_fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
                t_prev = now
                p = read_trackbars(p)
                worker.submit(frame, p)

            latest = worker.latest()
            if latest is None:
                wait = frame.copy()
                cv2.putText(wait, "inspecting...", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                disp = side_by_side(frame, wait)
            else:
                rframe, res = latest
                if id(res) != last_seen_id:           # 새 검사 결과가 나왔을 때만 이력 갱신
                    last_seen_id = id(res)
                    history.append(res.ng)
                    if args.save_ng and res.ng and time.time() - last_save > args.save_interval:
                        name = time.strftime("ng_%Y%m%d_%H%M%S.png")
                        imwrite_any(out_dir / name.replace(".png", "_raw.png"), rframe)
                        save(out_dir, name, rframe, res, p)
                        last_save = time.time()
                        print(f"NG 자동 저장: {name}")
                base = frame if not paused else rframe
                disp = draw(base, res, p, show_dust, show_map, f"CAM{index}")
                stable_ng = sum(history) >= args.stable_m
                h, w = disp.shape[:2]
                t = max(1, w // 600)
                label, col = ("NG", (0, 0, 255)) if stable_ng else ("OK", (0, 200, 0))
                cv2.rectangle(disp, (w - 150 * t, 40 * t), (w - 10, 100 * t), col, -1)
                cv2.putText(disp, label, (w - 130 * t, 88 * t), cv2.FONT_HERSHEY_SIMPLEX, 1.6 * t,
                            (255, 255, 255), 3 * t, cv2.LINE_AA)
                cv2.putText(disp, f"cam {cam_fps:.1f}fps  inspect {worker.fps:.1f}fps  "
                                  f"{'PAUSED' if paused else ''}",
                            (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * t, (0, 255, 255), 1, cv2.LINE_AA)
                disp = side_by_side(base, disp)
            cv2.imshow(WIN, fit_screen(disp))

            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k == ord(" "):
                paused = not paused
            elif k in (ord("d"), ord("D")):
                show_dust = not show_dust
            elif k in (ord("m"), ord("M")):
                show_map = not show_map
            elif k in (ord("s"), ord("S")) and latest is not None:
                name = time.strftime("cam_%Y%m%d_%H%M%S.png")
                imwrite_any(out_dir / name.replace(".png", "_raw.png"), latest[0])
                save(out_dir, name, latest[0], latest[1], p)
                report(name, latest[1])
                print(f"저장 → {out_dir.resolve()}")
    finally:
        worker.running = False
        worker.join(timeout=1.0)
        cap.release()
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="O-ring 결함 검출 PoC V1")
    ap.add_argument("--folder", type=Path, help="검사 이미지 폴더")
    ap.add_argument("--image", type=Path, help="이미지 1장")
    ap.add_argument("--camera", type=int, help="카메라 번호 (생략 시 검색 후 선택)")
    ap.add_argument("--list-cameras", action="store_true", help="카메라 목록만 출력")
    ap.add_argument("--cam-width", type=int, default=1920)
    ap.add_argument("--cam-height", type=int, default=1080)
    ap.add_argument("--scale", type=float, default=0.5, help="검사 처리 해상도 배율(1.0=원본, 0.5=절반, 빠름)")
    ap.add_argument("--angle-every", type=int, default=10, help="띠 방향 재계산 주기(프레임)")
    ap.add_argument("--stable-n", type=int, default=5, help="판정 안정화: 최근 N회")
    ap.add_argument("--stable-m", type=int, default=3, help="판정 안정화: N회 중 M회 이상 NG면 NG")
    ap.add_argument("--save-ng", action="store_true", help="NG 프레임 자동 저장")
    ap.add_argument("--save-interval", type=float, default=2.0, help="NG 자동 저장 최소 간격(초)")
    ap.add_argument("--batch", action="store_true", help="창 없이 전체 판정 후 저장")
    ap.add_argument("--um-per-px", type=float, default=8.0, help="물체 쪽 1화소 크기(µm)")
    ap.add_argument("--blob-mm", type=float, default=0.30, help="덩어리 NG 기준(mm)")
    ap.add_argument("--k", type=float, default=4.0, help="성장 임계값 민감도(작을수록 민감)")
    ap.add_argument("--seed-k", type=float, default=12.0, help="씨앗 임계값(작을수록 민감)")
    ap.add_argument("--out", type=Path, help="저장 폴더(기본: 스크립트 폴더/output)")
    args = ap.parse_args()

    p = Params(um_per_px=args.um_per_px, blob_mm=args.blob_mm, k_sigma=args.k, k_seed=args.seed_k)
    script_dir = Path(__file__).resolve().parent
    out_dir = args.out or script_dir / "output"
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.list_cameras:
        cams, names = scan_cameras(), camera_names()
        if not cams:
            print("연결된 카메라가 없습니다.")
        for i, w, h in cams:
            print(f"[{i}] {names[i] if i < len(names) else ''}  기본 {w}x{h}")
        return 0
    if not (0.1 <= args.scale <= 1.0):
        print("오류: --scale 은 0.1~1.0")
        return 1
    if args.stable_m > args.stable_n:
        print("오류: --stable-m 은 --stable-n 이하여야 합니다.")
        return 1

    # 이미지 지정이 없으면 실시간 카메라 모드
    if args.image is None and args.folder is None and not args.batch:
        return run_camera(args.camera, p, out_dir, args)

    if args.image:
        img_path = args.image if args.image.exists() else script_dir / args.image
        paths = [img_path]
    else:
        folder = args.folder or script_dir / "samples"
        if not folder.exists() or not list_images(folder):
            folder = ask_folder(script_dir)
            if folder is None:
                print("오류: 이미지 폴더가 없습니다. 같은 폴더에 samples 폴더를 만들어 이미지를 넣어 주세요.")
                return 1
        paths = list_images(folder)
    if not paths or not paths[0].exists():
        print("오류: 검사할 이미지가 없습니다.")
        return 1

    return run_batch(paths, p, out_dir) if args.batch else run_viewer(paths, p, out_dir)


if __name__ == "__main__":
    sys.exit(main())
