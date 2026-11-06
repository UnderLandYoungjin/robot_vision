"""
OringDefectPoC_V10
O-ring 국부(확대) 이미지 결함 검출 PoC (Proof of Concept, 개념 검증)

V10 변경:
  - PDF 검사 이미지 3장: 원본 / 검사 결과 / 응답 맵(검출 근거, 결함 윤곽 표시)
  - 응답 맵 이미지도 보관 (report/images/{SN}_{일시}/map.jpg, 200~500KB)

V9 변경:
  - 성적서 한국어·중국어 동시 출력 (같은 내용, 언어별 파일)
        PDF  : report/pdf/{SN}_{일시}_KO.pdf / _CN.pdf
        Excel: report/InspectionReport_{YYYYMMDD}_KO.xlsx / _CN.xlsx
        --lang ko / cn / both(기본)
  - 검사 이미지 저장 시 원본 비트맵(BMP, 무손실) 함께 저장 — 재검사·알고리즘 검증용
        성적서: report/images/{SN}_{일시}/original.bmp,  S 키 저장: output/{이름}_original.bmp
        --no-bmp 로 끄기

V8 변경:
  - 성적서(PDF·Excel)는 [검사 기록] 버튼 클릭 또는 R 키를 눌렀을 때만 생성 (자동 생성 기본 꺼짐)
      실시간 자동 생성이 필요하면 --auto-report, 배치에서 생성하려면 --report
  - Excel 파일이 열려 있어 저장이 막히면 InspectionReport_YYYYMMDD_시각.xlsx 로 따로 저장 (기록 유실 방지)

V7 변경:
  - 창 크기 자동 맞춤: 모니터 해상도·Windows 배율(DPI, Dots Per Inch)을 읽어 화면 안에 들어오게 창 크기 조정
  - F 키: 창 크기 다시 맞추기 (창을 마우스로 늘리고 줄여도 비율 유지)

V6 변경:
  - 불량품(NG)마다 1장짜리 PDF 성적서 자동 생성 (실시간: OK→NG 전환 시 1회, 배치: 기본 생성)
  - Excel 이미지 표시 크기 확대 (상단 XL_* 상수로 조절), Excel에서 PDF 성적서 바로 열기 링크
  - PDF 1장 고정: 결함 표는 NG 우선 최대 PDF_MAX_ROWS 행, 나머지는 Excel 결함상세 참조

V5 변경:
  - 불량 유형 명칭 표기: 스크래치(SCRATCH) / 파임(DENT) / 이물(FOREIGN, 유색 덩어리) / 먼지(허용)
      선형 결함 → 스크래치, 덩어리 결함 → 파임 (채도 높으면 이물)
  - 성적서(PDF·Excel)에 '불량 유형' 항목 추가
  - 보관용 이미지(원본·결과)는 JPEG 화질 자동 조절로 장당 200~500KB 수준 저장
  - Excel 결과 이미지가 '결과 이미지' 열에 정확히 들어가도록 위치 고정

V4 변경 (검사 성적서 출력):
  - 검사 결과를 PDF (Portable Document Format) 성적서 / Excel 검사 이력으로 자동 출력
  - 수록 항목: SN (Serial Number, 제품 일련번호), 검사 일시, 검사 소요시간(ms),
               합격/불합격 판정, 결함별 실측 픽셀 개수(면적·길이·폭 px), mm 환산 크기,
               원본·결과 이미지, 결함 부위 확대 이미지
  - 출력 폴더: output/report/
        PDF  : report/pdf/{SN}_{일시}.pdf            (1제품 1장)
        Excel: report/InspectionReport_{YYYYMMDD}.xlsx (일자별 누적, 시트 2개: 검사이력 / 결함상세)
        이력 : report/log_{YYYYMMDD}.json             (Excel 재생성용 원본 기록)
  - 실시간·이미지 뷰어: R 키 → SN 입력 창 → PDF·Excel 출력
  - 배치: --report 옵션 시 이미지마다 성적서 생성 (SN = 파일명, --sn 지정 시 접두어)
  - 실시간 NG 자동 성적서: (NG 자동 성적서 기본)

V3 변경: 화면 좌우 분할 (왼쪽 원본, 오른쪽 검사 결과)
V2 변경: 실시간 검사 (카메라 검색·선택, 검사 스레드 분리, 판정 안정화, NG 자동 저장)

판정 기준 (2026.11 협의 기준, 목표 사양):
  - 선형 결함(크랙·실밥·스크래치): 선으로 검출되면 길이 무관 NG
  - 덩어리형 결함(이물·찍힘·결손): 최대 크기 0.3mm 초과 NG
  - 그 이하 점(먼지): OK

사용 예:
  python oring_defect_poc.py                              (카메라 검색·선택 → 실시간 검사)
  python oring_defect_poc.py --camera 1                   (버튼/R 키로 성적서 생성)
  python oring_defect_poc.py --folder samples             (이미지 폴더 뷰어)
  python oring_defect_poc.py --batch --report             (폴더 전체 판정 + PDF·Excel)
  python oring_defect_poc.py --batch --report --report-format excel
  python oring_defect_poc.py --image 05.jpg --batch --sn LOT2611A

조작키 (실시간):
  Space 일시정지   D 먼지 표시   M 응답 맵   S 화면 저장   R 성적서(PDF·Excel)   Q/Esc 종료
조작키 (이미지 뷰어):
  N/→ 다음   P/← 이전   D 먼지   M 응답 맵   S 저장   R 성적서(PDF·Excel)   Q/Esc 종료

필요 패키지:
  pip install opencv-python numpy openpyxl reportlab pillow
  (선택) pip install pygrabber   ← 카메라 이름 표시

Python 3.12.3 / OpenCV 4.x / numpy
"""

import argparse
import csv
import json
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

IMG_EXT = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
WIN = "OringDefectPoC_V10"
SAVE_BMP = True                         # 원본 BMP 함께 저장 (--no-bmp 로 끄기)

# ---- Excel 이미지 표시 크기(px) — 여기 숫자만 바꾸면 크기 조절
XL_RESULT_W, XL_RESULT_H = 480, 300     # 검사이력 시트 결과 이미지
XL_CROP = 200                           # 결함상세 시트 결함 확대 이미지 (정사각)
PDF_MAX_ROWS = 6                        # PDF 1장에 들어가는 결함 행 수
PRODUCT_NAME = "SLIT VALVE O-ring"
REPORT_TITLE = "O-ring 외관 검사 성적서"


# ================================================================ 파라미터
@dataclass(frozen=True)
class Params:
    um_per_px: float = 8.0        # 물체 쪽 1화소 크기(µm) — 장비 배율 확정 후 고정값 입력
    k_sigma: float = 4.0          # 성장 임계값 = 중앙값 + k × 잡음
    k_seed: float = 12.0          # 씨앗 임계값 — 이 이상 강한 화소가 있는 영역만 결함 후보
    line_aspect: float = 6.0      # 세장비 이상이면 '선'
    line_min_mm: float = 0.15     # 이보다 짧은 선형 영역은 점(먼지)으로 처리
    blob_mm: float = 0.30         # 덩어리 NG 기준
    bg_len_mm: float = 1.5        # 배경 추정 길이(띠 방향)
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


def imwrite_any(path: Path, img: np.ndarray) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(path.suffix or ".png", img)
    if ok:
        buf.tofile(str(path))
    return bool(ok)


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


def ask_sn(default: str) -> str | None:
    """SN 입력 창. 취소 시 None. tkinter 사용 불가 시 콘솔 입력."""
    try:
        import tkinter as tk
        from tkinter import simpledialog
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        sn = simpledialog.askstring("SN 입력", "제품 SN (Serial Number, 일련번호):",
                                    initialvalue=default, parent=root)
        root.destroy()
    except Exception:
        try:
            sn = input(f"SN 입력 (Enter = {default}): ").strip() or default
        except EOFError:
            sn = default
    if sn is None:
        return None
    sn = sn.strip() or default
    return "".join(c if c not in '\\/:*?"<>|' else "_" for c in sn)   # 파일명 불가 문자 치환


# ================================================================ 불량 유형
def defect_type(kind: str, colored: bool) -> tuple[str, str]:
    """(화면 표기 영문, 성적서 표기 한글)."""
    if kind == "LINE":
        return "SCRATCH", "스크래치"
    if kind == "BLOB":
        return ("FOREIGN", "이물") if colored else ("DENT", "파임")
    return "DUST", "먼지(허용)"


def type_summary(defects, lang: int = 1) -> str:
    """NG 결함 유형별 건수 요약. 예) '스크래치 2, 파임 1'."""
    cnt: dict[str, int] = {}
    for d in defects:
        if d.ng:
            name = defect_type(d.kind, d.colored)[lang]
            cnt[name] = cnt.get(name, 0) + 1
    return ", ".join(f"{k} {v}" for k, v in cnt.items()) or ("-" if lang else "-")


def imwrite_jpeg_target(path: Path, img: np.ndarray, lo: int = 200_000, hi: int = 500_000) -> int:
    """보관용 JPEG 저장 — 화질·크기를 자동 조절해 lo~hi 바이트 범위로 맞춤. 저장 크기 반환."""
    path.parent.mkdir(parents=True, exist_ok=True)
    work = img
    buf = None
    for _ in range(8):
        for q in (95, 90, 85, 80, 75, 70, 65, 60):
            ok, b = cv2.imencode(".jpg", work, [cv2.IMWRITE_JPEG_QUALITY, q])
            if not ok:
                continue
            buf = b
            if b.size <= hi:
                break
        if buf is not None and buf.size <= hi:
            break
        work = cv2.resize(work, None, fx=0.8, fy=0.8, interpolation=cv2.INTER_AREA)   # 화질만으로 안 되면 축소
    if buf is None:
        return 0
    # 작은 영상은 lo 미만일 수 있음 (확대 저장은 의미 없으므로 최고 화질로 저장)
    buf.tofile(str(path))
    return int(buf.size)


# ================================================================ 결과 구조
@dataclass
class Defect:
    kind: str           # LINE / BLOB / DUST
    ng: bool
    length_mm: float    # LINE = 길이, BLOB/DUST = 최대 크기
    width_mm: float
    area_mm2: float
    elong: float
    colored: bool
    contour: np.ndarray  # 원본 좌표
    area_px: int = 0     # 실측 픽셀 개수 (결함 영역 화소 수)
    length_px: float = 0.0
    width_px: float = 0.0


@dataclass
class Result:
    ng: bool
    defects: list
    angle_deg: float
    threshold: float
    response: np.ndarray    # 원본 좌표 응답 맵(0~255)
    cycle_ms: float = 0.0   # 검사 소요시간


# ================================================================ 핵심 처리
def ring_angle(gray: np.ndarray) -> float:
    """구조 텐서로 반사 띠(O-ring 진행) 방향 각도(deg) 추정."""
    small = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    g = cv2.GaussianBlur(small, (0, 0), 3).astype(np.float32)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    jxx, jyy, jxy = float((gx * gx).sum()), float((gy * gy).sum()), float((gx * gy).sum())
    grad_dir = 0.5 * np.degrees(np.arctan2(2 * jxy, jxx - jyy))
    return grad_dir + 90.0


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
    t0 = time.perf_counter()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)

    # 1) 띠 방향 → 수평 정렬
    ang = ring_angle(gray) if fixed_angle is None else fixed_angle
    cands = (ang,) if fixed_angle is not None else (ang, -ang)
    best = None
    for cand in cands:
        rot, M, Minv = rotate_bound(gray, cand, border=None)
        rs = cv2.resize(rot, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA).astype(np.float32)
        score = float(np.var(rs.mean(axis=1)))
        if best is None or score > best[0]:
            best = (score, cand, rot, M, Minv)
    _, ang, rot, M, Minv = best
    valid = cv2.warpAffine(np.full(gray.shape, 255, np.uint8), M, (rot.shape[1], rot.shape[0]))
    valid = cv2.erode(valid, np.ones((9, 9), np.uint8))
    color_rot = cv2.warpAffine(img, M, (rot.shape[1], rot.shape[0]), borderMode=cv2.BORDER_REPLICATE)

    # 2) 띠 방향 1차원 평균으로 배경 추정 (2회 반복, 결함 영향 제거)
    k = int(round(p.bg_len_mm / p.mm_per_px))
    k = int(np.clip(k, 15, max(15, rot.shape[1] // 2)))
    k += (k + 1) % 2
    rot_f = rot.astype(np.float32)
    bg = cv2.blur(rot_f, (k, 1), borderType=cv2.BORDER_REFLECT)
    diff0 = np.abs(rot_f - bg)
    robust = np.where(diff0 > 3 * (np.median(diff0) * 1.4826 + 1e-6), bg, rot_f)
    bg = cv2.blur(robust, (k, 1), borderType=cv2.BORDER_REFLECT)
    resp = np.abs(rot_f - bg)
    valid = cv2.erode(valid, cv2.getStructuringElement(cv2.MORPH_RECT, (k // 2 * 2 + 1, 1)))
    resp = cv2.GaussianBlur(resp, (0, 0), 1.2)

    # 3) 자동 임계값 (잡음 = MAD, Median Absolute Deviation 중앙값 절대편차) + 히스테리시스
    vals = resp[valid > 0]
    if vals.size == 0:
        vals = resp.ravel()
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
        width_px = max(2.0 * float(np.percentile(dvals, 90)), 1.5)
        length_px = max(area / width_px, 1.0)
        (_, _), (rw, rh), _ = cv2.minAreaRect(c)
        rect_len = max(rw, rh)
        elong = length_px / width_px
        length_mm = length_px * p.mm_per_px
        eq_diam_px = float(np.sqrt(4.0 * area / np.pi))
        size_px = max(eq_diam_px, 0.7 * rect_len)
        size_mm = size_px * p.mm_per_px
        if elong >= p.line_aspect and length_mm >= p.line_min_mm:
            kind, ng = "LINE", True
        elif size_mm > p.blob_mm:
            kind, ng = "BLOB", True
        else:
            kind, ng = "DUST", False

        m = np.zeros(binm.shape, np.uint8)
        cv2.drawContours(m, [c], -1, 255, -1)
        colored = cv2.mean(hsv[:, :, 1], mask=m)[0] > p.sat_foreign
        pix = int(cv2.countNonZero(m))                     # 실측 픽셀 개수

        pts = c.reshape(-1, 1, 2).astype(np.float32)
        back = cv2.transform(pts, Minv).astype(np.int32)
        defects.append(Defect(kind, ng, length_mm if kind == "LINE" else size_mm,
                              width_px * p.mm_per_px, pix * p.mm_per_px ** 2,
                              float(elong), bool(colored), back,
                              area_px=pix,
                              length_px=float(length_px if kind == "LINE" else size_px),
                              width_px=float(width_px)))

    resp_orig = cv2.warpAffine(np.clip(resp * (255.0 / max(thr * 3, 1)), 0, 255).astype(np.uint8),
                               Minv, (img.shape[1], img.shape[0]))
    defects.sort(key=lambda d: (not d.ng, d.kind != "LINE", -d.length_mm))
    cycle_ms = (time.perf_counter() - t0) * 1000.0
    return Result(any(d.ng for d in defects), defects, float(ang), thr, resp_orig, cycle_ms)


# ================================================================ 표시
def defect_color(kind: str):
    return (0, 0, 255) if kind == "LINE" else (0, 140, 255) if kind == "BLOB" else (160, 160, 160)


def draw(img: np.ndarray, res: Result, p: Params, show_dust: bool, show_map: bool, name: str) -> np.ndarray:
    vis = cv2.applyColorMap(res.response, cv2.COLORMAP_JET) if show_map else img.copy()
    t = max(1, img.shape[1] // 600)
    n_ng = 0
    for d in res.defects:
        if d.kind == "DUST" and not show_dust:
            continue
        col = defect_color(d.kind)
        cv2.drawContours(vis, [d.contour], -1, col, t if d.ng else 1, cv2.LINE_AA)
        if d.ng:
            n_ng += 1
            x, y, w, h = cv2.boundingRect(d.contour)
            pad = 6 * t
            cv2.rectangle(vis, (x - pad, y - pad), (x + w + pad, y + h + pad), col, t)
            tag = f"#{n_ng} {defect_type(d.kind, d.colored)[0]} {d.length_mm:.2f}mm"
            cv2.putText(vis, tag, (x - pad, max(y - pad - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45 * t, col, t, cv2.LINE_AA)

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
    info = (f"{name}  SCRATCH {lines}  DENT {blobs}  dust {dust}  {p.um_per_px:.1f}um/px  "
            f"angle {res.angle_deg:.1f}  {res.cycle_ms:.0f}ms")
    cv2.putText(vis, info, (int(70 * t), int(23 * t)), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * t,
                (255, 255, 255), 1, cv2.LINE_AA)
    return vis


def side_by_side(original: np.ndarray, result: np.ndarray, gap: int = 6) -> np.ndarray:
    """왼쪽 원본 | 오른쪽 결과."""
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


def _screen_size() -> tuple[int, int]:
    """실제 모니터 해상도(px). Windows 배율(125%·150%) 영향 제거."""
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(2)      # 배율 무시, 실제 화소 기준
        except Exception:
            ctypes.windll.user32.SetProcessDPIAware()
        u = ctypes.windll.user32
        return int(u.GetSystemMetrics(0)), int(u.GetSystemMetrics(1))
    except Exception:
        try:
            import tkinter as tk
            r = tk.Tk()
            w, h = r.winfo_screenwidth(), r.winfo_screenheight()
            r.destroy()
            return w, h
        except Exception:
            return 1600, 900


SCREEN_W, SCREEN_H = _screen_size()
TRACKBAR_SPACE = 260          # 트랙바 6개 + 제목줄 + 작업표시줄 여유(px)


def fit_screen(img, max_w=None, max_h=None):
    max_w = max_w or int(SCREEN_W * 0.96)
    max_h = max_h or int(SCREEN_H - TRACKBAR_SPACE)
    h, w = img.shape[:2]
    s = min(max_w / w, max_h / h, 1.0)
    return img if s >= 1 else cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)


class Display:
    """창 표시 — 화면 크기가 바뀔 때(첫 화면, F 키)만 창 크기 재설정, 이후 사용자 조절 유지."""

    BTN_TEXT = "RECORD (R)"

    def __init__(self):
        self.size = None
        self.btn = None
        self.clicked = False
        cv2.setMouseCallback(WIN, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP and self.btn is not None:
            x0, y0, x1, y1 = self.btn
            if x0 <= x <= x1 and y0 <= y <= y1:
                self.clicked = True

    def take_click(self) -> bool:
        c, self.clicked = self.clicked, False
        return c

    def show(self, img, refit: bool = False):
        disp = fit_screen(img).copy()
        h, w = disp.shape[:2]
        # [검사 기록] 버튼 — 왼쪽 아래
        bw, bh = 190, 46
        x0, y0 = 12, h - bh - 12
        self.btn = (x0, y0, x0 + bw, y0 + bh)
        cv2.rectangle(disp, (x0, y0), (x0 + bw, y0 + bh), (180, 90, 20), -1)
        cv2.rectangle(disp, (x0, y0), (x0 + bw, y0 + bh), (255, 255, 255), 2)
        cv2.putText(disp, self.BTN_TEXT, (x0 + 14, y0 + 31), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    (255, 255, 255), 2, cv2.LINE_AA)
        if refit or self.size != (w, h):
            self.size = (w, h)
            cv2.resizeWindow(WIN, w, h)
            cv2.moveWindow(WIN, max((SCREEN_W - w) // 2, 0), 0)
        cv2.imshow(WIN, disp)


def create_window():
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)


def report(name: str, res: Result) -> None:
    ngs = [d for d in res.defects if d.ng]
    print(f"[{'NG' if res.ng else 'OK'}] {name}  스크래치 {sum(d.kind=='LINE' for d in ngs)}  "
          f"파임·이물 {sum(d.kind=='BLOB' for d in ngs)}  먼지 {sum(d.kind=='DUST' for d in res.defects)}  "
          f"{res.cycle_ms:.1f}ms")
    for i, d in enumerate(ngs, 1):
        kind = defect_type(d.kind, d.colored)[1]
        print(f"    #{i} {kind}  길이 {d.length_mm:.3f}mm  "
              f"폭 {d.width_mm:.3f}mm  세장비 {d.elong:.1f}  픽셀 {d.area_px}px")


def save(out_dir: Path, name: str, img, res: Result, p: Params) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(name).stem
    vis = draw(img, res, p, True, False, name)
    if SAVE_BMP:
        imwrite_any(out_dir / f"{stem}_original.bmp", img)
    imwrite_any(out_dir / f"{stem}_result.png", vis)
    imwrite_any(out_dir / f"{stem}_compare.png", side_by_side(img, vis))
    imwrite_any(out_dir / f"{stem}_map.png", cv2.applyColorMap(res.response, cv2.COLORMAP_JET))
    with (out_dir / f"{stem}_defects.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["no", "type", "kind", "ng", "length_mm", "width_mm", "area_mm2", "area_px",
                    "length_px", "width_px", "elong", "colored", "x", "y"])
        for i, d in enumerate(res.defects, 1):
            x, y, _, _ = cv2.boundingRect(d.contour)
            w.writerow([i, defect_type(d.kind, d.colored)[1], d.kind, int(d.ng), f"{d.length_mm:.4f}", f"{d.width_mm:.4f}",
                        f"{d.area_mm2:.5f}", d.area_px, f"{d.length_px:.1f}", f"{d.width_px:.1f}",
                        f"{d.elong:.2f}", int(d.colored), x, y])


# ================================================================ 성적서 (PDF / Excel)
KIND_KR = {"LINE": "스크래치", "BLOB": "파임", "DUST": "먼지(허용)"}


def thumb(img: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    h, w = img.shape[:2]
    s = min(max_w / w, max_h / h)
    return cv2.resize(img, (max(1, int(w * s)), max(1, int(h * s))),
                      interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_CUBIC)


def crop_defect(img: np.ndarray, d: Defect, color, pad: int = 25) -> np.ndarray:
    """결함 중심 정사각 크롭 (윤곽선 표시)."""
    h, w = img.shape[:2]
    x, y, bw, bh = cv2.boundingRect(d.contour)
    side = max(bw, bh) + 2 * pad
    cx, cy = x + bw // 2, y + bh // 2
    x0, y0 = max(cx - side // 2, 0), max(cy - side // 2, 0)
    x1, y1 = min(x0 + side, w), min(y0 + side, h)
    x0, y0 = max(x1 - side, 0), max(y1 - side, 0)
    view = img.copy()
    cv2.drawContours(view, [d.contour], -1, color, 1, cv2.LINE_AA)
    crop = view[y0:y1, x0:x1]
    return thumb(crop, 400, 400)


# ================================================================ 성적서 언어 (한국어 KO / 중국어 CN)
TXT = {
    "KO": {
        "title": "O-ring 외관 검사 성적서",
        "sub": "O-ring Visual Inspection Report  |  목표 사양(협의) 기준 PoC 판정",
        "product_name": "SLIT VALVE O-ring",
        "pass": "합격", "fail": "불합격", "pass_full": "합격 (OK)", "fail_full": "불합격 (NG)",
        "good": "양호 (불량 없음)", "good_short": "양호",
        "product": "제품명", "sn": "SN (Serial Number, 일련번호)", "type": "불량 유형", "verdict": "판정",
        "time": "검사 일시", "cycle": "검사 소요시간",
        "n_line": "스크래치(NG)", "n_blob": "파임·이물(NG)", "n_dust": "먼지(허용)", "max": "최대 결함 크기",
        "ngpx": "NG 결함 픽셀 합계", "allpx": "전체 검출 픽셀 합계", "res": "화소 분해능", "imgsize": "영상 크기",
        "cnt": "건",
        "sec_img": "검사 이미지 (원본 / 검사 결과 / 응답 맵)", "orig": "원본 (Original)",
        "map": "응답 맵 (Response Map, 검출 근거)",
        "result": "검사 결과 (Result)",
        "sec_def": "결함 상세 (실측 픽셀 개수 · mm 환산)",
        "dhead": ["No", "결함 이미지", "불량 유형", "판정", "크기\n(mm)", "폭\n(mm)", "픽셀 개수\n(px)",
                  "길이\n(px)", "폭\n(px)", "세장비", "위치 (x, y)"],
        "nodef": "검출된 결함 없음",
        "omit": "※ 표시 생략 {n}건 (Excel 결함상세 시트 참조)",
        "crit": ("판정 기준(목표 사양, 협의): 스크래치(선형 결함) 길이 무관 NG / 파임·이물(덩어리 결함) "
                 "{blob:.2f} mm 초과 NG / 그 이하 점(먼지) OK.  검출 파라미터: k={k:.1f}, seed k={seed:.1f}.  "
                 "mm 환산값 = 픽셀 × {um:.2f} µm/px (장비 배율 확정 전 가정값)."),
        "sign": ["검사", "확인", "승인"],
        "types": {"SCRATCH": "스크래치", "DENT": "파임", "FOREIGN": "이물", "DUST": "먼지(허용)"},
        "x_font": "맑은 고딕",
        "x_sheet1": "검사이력", "x_sheet2": "결함상세", "x_sheet3": "판정기준",
        "x_title": "{title} — 검사 이력 ({d})",
        "x_summary": ("총 {n}건   합격(OK) {ok}   불합격(NG) {ng}   합격률 {rate:.1f}%   "
                      "※ 판정 기준은 목표 사양(협의), mm 환산은 화소 분해능 가정값 기준"),
        "x_head1": ["No", "SN\n(Serial Number)", "검사 일시", "검사 소요시간\n(ms)", "판정", "불량 유형",
                    "스크래치\nNG(건)", "파임·이물\nNG(건)", "먼지\n(건)", "최대 결함\n크기(mm)",
                    "NG 픽셀 합계\n(px)", "전체 검출 픽셀\n(px)", "화소 분해능\n(µm/px)", "결과 이미지",
                    "성적서\n(PDF)", "입력"],
        "x_head2": ["SN\n(Serial Number)", "검사 일시", "판정\n(제품)", "결함 No", "불량 유형", "판정\n(결함)",
                    "크기(mm)", "폭(mm)", "면적(mm²)", "픽셀 개수\n(px)", "길이(px)", "폭(px)", "세장비",
                    "유색", "위치 x", "위치 y", "결함 이미지"],
        "x_open": "PDF 열기",
        "x_crit": [("항목", "기준 (목표 사양, 협의)"),
                   ("스크래치 (선형 결함)", "선으로 검출되면 길이 무관 NG"),
                   ("파임 (덩어리 결함)", "최대 크기 0.3 mm 초과 NG (검사 시 설정값은 검사이력 참조)"),
                   ("이물 (유색 덩어리 결함)", "파임과 같은 크기 기준, 색(채도)이 있으면 이물로 표기"),
                   ("먼지 (점)", "기준 이하 점은 OK"),
                   ("픽셀 개수 (px)", "결함 영역에 포함된 실제 화소 수"),
                   ("mm 환산", "픽셀 × 화소 분해능(µm/px) ÷ 1000 — 장비 배율 확정 전 가정값"),
                   ("SN (Serial Number)", "제품 일련번호"),
                   ("NG / OK", "NG (No Good, 불합격) / OK (합격)")],
    },
    "CN": {
        "title": "O型圈外观检验报告",
        "sub": "O-ring Visual Inspection Report  |  按目标规格(协商)的 PoC 判定",
        "product_name": "SLIT VALVE O型圈",
        "pass": "合格", "fail": "不合格", "pass_full": "合格 (OK)", "fail_full": "不合格 (NG)",
        "good": "良好 (无不良)", "good_short": "良好",
        "product": "产品名称", "sn": "SN (Serial Number, 序列号)", "type": "不良类型", "verdict": "判定",
        "time": "检验时间", "cycle": "检验耗时",
        "n_line": "划痕(NG)", "n_blob": "凹坑·异物(NG)", "n_dust": "灰尘(允许)", "max": "最大缺陷尺寸",
        "ngpx": "NG缺陷像素合计", "allpx": "全部检出像素合计", "res": "像素分辨率", "imgsize": "图像尺寸",
        "cnt": "个",
        "sec_img": "检验图像 (原图 / 检验结果 / 响应图)", "orig": "原图 (Original)",
        "map": "响应图 (Response Map, 检测依据)",
        "result": "检验结果 (Result)",
        "sec_def": "缺陷明细 (实测像素数 · mm换算)",
        "dhead": ["No", "缺陷图像", "不良类型", "判定", "尺寸\n(mm)", "宽度\n(mm)", "像素数\n(px)",
                  "长度\n(px)", "宽度\n(px)", "长宽比", "位置 (x, y)"],
        "nodef": "未检出缺陷",
        "omit": "※ 省略显示 {n} 项 (请参考 Excel 缺陷明细表)",
        "crit": ("判定标准(目标规格, 协商): 划痕(线状缺陷) 不论长度均为 NG / 凹坑·异物(块状缺陷) 超过 "
                 "{blob:.2f} mm 为 NG / 以下的点(灰尘) 为 OK.  检测参数: k={k:.1f}, seed k={seed:.1f}.  "
                 "mm换算值 = 像素 × {um:.2f} µm/px (设备倍率确定前的假定值)."),
        "sign": ["检验", "确认", "批准"],
        "types": {"SCRATCH": "划痕", "DENT": "凹坑", "FOREIGN": "异物", "DUST": "灰尘(允许)"},
        "x_font": "Microsoft YaHei",
        "x_sheet1": "检验记录", "x_sheet2": "缺陷明细", "x_sheet3": "判定标准",
        "x_title": "{title} — 检验记录 ({d})",
        "x_summary": ("共 {n} 件   合格(OK) {ok}   不合格(NG) {ng}   合格率 {rate:.1f}%   "
                      "※ 判定标准为目标规格(协商), mm换算基于假定的像素分辨率"),
        "x_head1": ["No", "SN\n(序列号)", "检验时间", "检验耗时\n(ms)", "判定", "不良类型",
                    "划痕\nNG(个)", "凹坑·异物\nNG(个)", "灰尘\n(个)", "最大缺陷\n尺寸(mm)",
                    "NG像素合计\n(px)", "全部检出像素\n(px)", "像素分辨率\n(µm/px)", "结果图像",
                    "检验报告\n(PDF)", "输入"],
        "x_head2": ["SN\n(序列号)", "检验时间", "判定\n(产品)", "缺陷 No", "不良类型", "判定\n(缺陷)",
                    "尺寸(mm)", "宽度(mm)", "面积(mm²)", "像素数\n(px)", "长度(px)", "宽度(px)", "长宽比",
                    "有色", "位置 x", "位置 y", "缺陷图像"],
        "x_open": "打开PDF",
        "x_crit": [("项目", "标准 (目标规格, 协商)"),
                   ("划痕 (线状缺陷)", "检出为线状时不论长度均为 NG"),
                   ("凹坑 (块状缺陷)", "最大尺寸超过 0.3 mm 为 NG (检验时设定值请参考检验记录)"),
                   ("异物 (有色块状缺陷)", "与凹坑相同的尺寸标准, 有颜色(饱和度)时标记为异物"),
                   ("灰尘 (点)", "标准以下的点为 OK"),
                   ("像素数 (px)", "缺陷区域内的实际像素数量"),
                   ("mm换算", "像素 × 像素分辨率(µm/px) ÷ 1000 — 设备倍率确定前的假定值"),
                   ("SN (Serial Number)", "产品序列号"),
                   ("NG / OK", "NG (No Good, 不合格) / OK (合格)")],
    },
}


def defect_code(d: dict) -> str:
    return d.get("code") or defect_type(d["kind"], d["colored"])[0]


def defect_name(d: dict, lang: str) -> str:
    return TXT[lang]["types"][defect_code(d)]


def rec_types(rec: dict, lang: str) -> str:
    """NG 결함 유형별 건수. 예) KO '스크래치 2, 파임 1' / CN '划痕 2, 凹坑 1'."""
    cnt: dict[str, int] = {}
    for d in rec["defects"]:
        if d["ng"]:
            n = defect_name(d, lang)
            cnt[n] = cnt.get(n, 0) + 1
    return ", ".join(f"{k} {v}" for k, v in cnt.items()) or "-"


def place_image(ws, path: str, col0: int, row0: int, w: int, h: int, off: int = 4) -> None:
    """Excel 이미지를 지정 셀(0 기준 열·행)에 정확히 고정 — 열 위치 밀림 방지."""
    from openpyxl.drawing.image import Image as XLImage
    from openpyxl.drawing.spreadsheet_drawing import AnchorMarker, OneCellAnchor
    from openpyxl.drawing.xdr import XDRPositiveSize2D
    from openpyxl.utils.units import pixels_to_EMU
    im = XLImage(path)
    s = min(w / im.width, h / im.height)
    iw, ih = int(im.width * s), int(im.height * s)
    im.width, im.height = iw, ih
    marker = AnchorMarker(col=col0, colOff=pixels_to_EMU(off), row=row0, rowOff=pixels_to_EMU(off))
    im.anchor = OneCellAnchor(_from=marker, ext=XDRPositiveSize2D(pixels_to_EMU(iw), pixels_to_EMU(ih)))
    ws.add_image(im)


class Reporter:
    """검사 기록 누적 + PDF 성적서 / Excel 이력 생성."""

    MAX_CROPS = 12      # 한 제품당 결함 확대 이미지 최대 수 (NG 우선)

    def __init__(self, out_dir: Path, fmt: str = "both", langs: tuple = ("KO", "CN")):
        self.root = out_dir / "report"
        self.fmt = fmt
        self.langs = langs
        self._fonts: dict[str, str] = {}
        self.day = datetime.now().strftime("%Y%m%d")
        self.log_path = self.root / f"log_{self.day}.json"
        self.records: list[dict] = []
        self.lock = threading.Lock()
        if self.log_path.exists():
            try:
                self.records = json.loads(self.log_path.read_text(encoding="utf-8"))
            except Exception:
                self.records = []

    # ---------------------------------------------------------- 기록
    def next_sn(self) -> str:
        return f"SN{self.day}-{len(self.records) + 1:04d}"

    def add(self, sn: str, img: np.ndarray, res: Result, p: Params, source: str = "") -> dict:
        now = datetime.now()
        stamp = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        key = f"{sn}_{stamp}"
        img_dir = self.root / "images" / key
        vis = draw(img, res, p, True, False, sn)

        imwrite_jpeg_target(img_dir / "original.jpg", img)        # 장당 200~500KB
        if SAVE_BMP:
            imwrite_any(img_dir / "original.bmp", img)            # 무손실 원본 (재검사용)
        imwrite_jpeg_target(img_dir / "result.jpg", vis)
        imwrite_jpeg_target(img_dir / "map.jpg", draw(img, res, p, True, True, sn))   # 응답 맵 + 결함 윤곽
        imwrite_any(img_dir / "result_thumb.jpg", thumb(vis, XL_RESULT_W * 2, XL_RESULT_H * 2))

        ngs = [d for d in res.defects if d.ng]
        dust = [d for d in res.defects if not d.ng]
        rows = []
        for i, d in enumerate(res.defects, 1):
            x, y, w, h = cv2.boundingRect(d.contour)
            crop_rel = ""
            if i <= self.MAX_CROPS and (d.ng or len(rows) < self.MAX_CROPS):
                crop = crop_defect(img, d, defect_color(d.kind))
                crop_path = img_dir / f"defect_{i:02d}.jpg"
                imwrite_any(crop_path, crop)
                crop_rel = str(crop_path.relative_to(self.root)).replace("\\", "/")
            rows.append({
                "no": i, "kind": d.kind, "type": defect_type(d.kind, d.colored)[1],
                "code": defect_type(d.kind, d.colored)[0], "ng": d.ng,
                "size_mm": round(d.length_mm, 4), "width_mm": round(d.width_mm, 4),
                "area_mm2": round(d.area_mm2, 5),
                "area_px": int(d.area_px), "length_px": round(d.length_px, 1),
                "width_px": round(d.width_px, 1), "elong": round(d.elong, 2),
                "colored": d.colored, "x": int(x), "y": int(y), "w": int(w), "h": int(h),
                "crop": crop_rel,
            })

        rec = {
            "seq": len(self.records) + 1,
            "sn": sn, "key": key, "source": source,
            "time": now.strftime("%Y-%m-%d %H:%M:%S"),
            "cycle_ms": round(res.cycle_ms, 1),
            "verdict": "NG" if res.ng else "OK",
            "types": type_summary(res.defects),
            "n_line": sum(d.kind == "LINE" for d in ngs),
            "n_blob": sum(d.kind == "BLOB" for d in ngs),
            "n_dust": len(dust),
            "max_size_mm": round(max((d.length_mm for d in ngs), default=0.0), 4),
            "total_ng_px": int(sum(d.area_px for d in ngs)),
            "total_px": int(sum(d.area_px for d in res.defects)),
            "img_w": int(img.shape[1]), "img_h": int(img.shape[0]),
            "um_per_px": p.um_per_px, "blob_mm": p.blob_mm,
            "k_sigma": p.k_sigma, "k_seed": p.k_seed,
            "original": f"images/{key}/original.jpg",
            "original_bmp": f"images/{key}/original.bmp" if SAVE_BMP else "",
            "result": f"images/{key}/result.jpg",
            "map": f"images/{key}/map.jpg",
            "result_thumb": f"images/{key}/result_thumb.jpg",
            "defects": rows,
        }
        with self.lock:
            self.records.append(rec)
            self.root.mkdir(parents=True, exist_ok=True)
            self.log_path.write_text(json.dumps(self.records, ensure_ascii=False, indent=1), encoding="utf-8")
        return rec

    def export(self, rec: dict, excel: bool = True) -> list[Path]:
        """fmt·언어 설정에 따라 PDF·Excel 출력 (한국어/중국어 각각). 실패해도 검사는 계속."""
        out = []
        for lang in self.langs:
            if self.fmt in ("pdf", "both"):
                try:
                    out.append(self.write_pdf(rec, lang))
                except Exception as e:
                    print(f"PDF({lang}) 출력 실패: {e}  (pip install reportlab pillow)")
        if excel and self.fmt in ("excel", "both"):
            out += self.write_excel_all()
        return out

    def write_excel_all(self) -> list[Path]:
        out = []
        for lang in self.langs:
            try:
                out.append(self.write_excel(lang))
            except Exception as e:
                print(f"Excel({lang}) 출력 실패: {e}  (pip install openpyxl pillow)")
        return out

    # ---------------------------------------------------------- PDF
    FONT_CANDS = {
        "KO": ["C:/Windows/Fonts/malgun.ttf", "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
               "/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf"],
        "CN": ["C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/msyh.ttf", "C:/Windows/Fonts/simhei.ttf",
               "C:/Windows/Fonts/simsun.ttc", "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
               "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", "/usr/share/fonts/truetype/arphic/uming.ttc"],
    }
    CID_FALLBACK = {"KO": "HYGothic-Medium", "CN": "STSong-Light"}

    def pdf_font(self, lang: str) -> str:
        if lang in self._fonts:
            return self._fonts[lang]
        from reportlab.lib.fonts import addMapping
        from reportlab.pdfbase import pdfmetrics
        from reportlab.pdfbase.ttfonts import TTFont
        name = f"F_{lang}"
        for f in self.FONT_CANDS[lang]:
            if not Path(f).exists():
                continue
            try:
                if f.lower().endswith(".ttc"):
                    pdfmetrics.registerFont(TTFont(name, f, subfontIndex=0))
                else:
                    pdfmetrics.registerFont(TTFont(name, f))
                for b in (0, 1):
                    for i in (0, 1):
                        addMapping(name, b, i, name)        # <b> 태그 오류 방지
                self._fonts[lang] = name
                return name
            except Exception:
                pass
        from reportlab.pdfbase.cidfonts import UnicodeCIDFont
        cid = self.CID_FALLBACK[lang]
        pdfmetrics.registerFont(UnicodeCIDFont(cid))
        self._fonts[lang] = cid
        return cid

    def write_pdf(self, rec: dict, lang: str = "KO") -> Path:
        from reportlab.lib import colors
        from reportlab.lib.pagesizes import A4
        from reportlab.lib.styles import ParagraphStyle
        from reportlab.lib.units import mm
        from reportlab.platypus import (Image, Paragraph, SimpleDocTemplate, Spacer, Table,
                                        TableStyle, KeepTogether)

        T = TXT[lang]
        font = self.pdf_font(lang)
        pdf_dir = self.root / "pdf"
        pdf_dir.mkdir(parents=True, exist_ok=True)
        path = pdf_dir / f"{rec['key']}_{lang}.pdf"

        st_title = ParagraphStyle("t", fontName=font, fontSize=17, leading=22, alignment=1)
        st_sub = ParagraphStyle("s", fontName=font, fontSize=8.5, leading=11, alignment=1,
                                textColor=colors.HexColor("#555555"))
        st_h = ParagraphStyle("h", fontName=font, fontSize=11, leading=15, spaceBefore=4, spaceAfter=3,
                              textColor=colors.HexColor("#1F3864"))
        st_c = ParagraphStyle("c", fontName=font, fontSize=8.5, leading=11)
        st_cc = ParagraphStyle("cc", parent=st_c, alignment=1)
        st_note = ParagraphStyle("n", fontName=font, fontSize=7.5, leading=10,
                                 textColor=colors.HexColor("#666666"))

        navy = colors.HexColor("#1F3864")
        light = colors.HexColor("#E9EEF6")
        grid = colors.HexColor("#9AA5B8")
        is_ng = rec["verdict"] == "NG"
        vcol = colors.HexColor("#C00000") if is_ng else colors.HexColor("#008A3E")

        def img_flow(rel: str, max_w, max_h):
            fp = self.root / rel
            im = imread_any(fp)
            if im is None:
                return Paragraph("-", st_cc)
            h, w = im.shape[:2]
            s = min(max_w / w, max_h / h)
            return Image(str(fp), width=w * s, height=h * s)

        story = [Paragraph(T["title"], st_title), Paragraph(T["sub"], st_sub), Spacer(1, 5 * mm)]

        verdict_tbl = Table([[Paragraph(f'<font color="white" size="26">{rec["verdict"]}</font>',
                                        ParagraphStyle("v", fontName=font, alignment=1, leading=30))],
                             [Paragraph(f'<font color="white">{T["fail"] if is_ng else T["pass"]}</font>',
                                        ParagraphStyle("v2", fontName=font, alignment=1, fontSize=10))]],
                            colWidths=[38 * mm], rowHeights=[22 * mm, 9 * mm])
        verdict_tbl.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), vcol),
                                         ("VALIGN", (0, 0), (-1, -1), "MIDDLE")]))
        types_txt = rec_types(rec, lang) if is_ng else T["good"]
        cnt = T["cnt"]
        info = [
            [T["product"], T["product_name"], T["sn"], rec["sn"]],
            [T["type"], Paragraph(f'<font color="{"#C00000" if is_ng else "#008A3E"}">{types_txt}</font>', st_c),
             T["verdict"], T["fail_full"] if is_ng else T["pass_full"]],
            [T["time"], rec["time"], T["cycle"], f'{rec["cycle_ms"]:.1f} ms'],
            [T["n_line"], f'{rec["n_line"]} {cnt}', T["n_blob"], f'{rec["n_blob"]} {cnt}'],
            [T["n_dust"], f'{rec["n_dust"]} {cnt}', T["max"], f'{rec["max_size_mm"]:.3f} mm'],
            [T["ngpx"], f'{rec["total_ng_px"]:,} px', T["allpx"], f'{rec["total_px"]:,} px'],
            [T["res"], f'{rec["um_per_px"]:.2f} µm/px', T["imgsize"], f'{rec["img_w"]} × {rec["img_h"]} px'],
        ]
        info_p = [[c if isinstance(c, Paragraph) else Paragraph(str(c), st_c) for c in r] for r in info]
        info_tbl = Table(info_p, colWidths=[27 * mm, 40 * mm, 35 * mm, 38 * mm])
        info_tbl.setStyle(TableStyle([
            ("GRID", (0, 0), (-1, -1), 0.5, grid),
            ("BACKGROUND", (0, 0), (0, -1), light), ("BACKGROUND", (2, 0), (2, -1), light),
            ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
            ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
        ]))
        top = Table([[verdict_tbl, info_tbl]], colWidths=[42 * mm, 140 * mm])
        top.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                                 ("LEFTPADDING", (0, 0), (-1, -1), 0)]))
        story += [top, Spacer(1, 4 * mm)]

        story.append(Paragraph(T["sec_img"], st_h))
        iw, ih = 58 * mm, 52 * mm
        map_cell = img_flow(rec["map"], iw, ih) if rec.get("map") else Paragraph("-", st_cc)
        imgs = Table([[img_flow(rec["original"], iw, ih), img_flow(rec["result"], iw, ih), map_cell],
                      [Paragraph(T["orig"], st_cc), Paragraph(T["result"], st_cc), Paragraph(T["map"], st_cc)]],
                     colWidths=[60.6 * mm] * 3)
        imgs.setStyle(TableStyle([("ALIGN", (0, 0), (-1, -1), "CENTER"),
                                  ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                                  ("LEFTPADDING", (0, 0), (-1, -1), 2), ("RIGHTPADDING", (0, 0), (-1, -1), 2),
                                  ("BOX", (0, 0), (0, -1), 0.5, grid), ("BOX", (1, 0), (1, -1), 0.5, grid),
                                  ("BOX", (2, 0), (2, -1), 0.5, grid)]))
        story += [imgs, Spacer(1, 4 * mm)]

        story.append(Paragraph(T["sec_def"], st_h))
        head = T["dhead"]
        data = [[Paragraph(f'<font color="white">{h.replace(chr(10), "<br/>")}</font>', st_cc) for h in head]]
        shown = [d for d in rec["defects"] if d["ng"]] + [d for d in rec["defects"] if not d["ng"]]
        shown = shown[:PDF_MAX_ROWS]
        for d in shown:
            crop = img_flow(d["crop"], 20 * mm, 20 * mm) if d["crop"] else Paragraph("-", st_cc)
            data.append([Paragraph(str(d["no"]), st_cc), crop, Paragraph(defect_name(d, lang), st_c),
                         Paragraph(f'<font color="{"#C00000" if d["ng"] else "#008A3E"}">'
                                   f'{"NG" if d["ng"] else "OK"}</font>', st_cc),
                         Paragraph(f'{d["size_mm"]:.3f}', st_cc), Paragraph(f'{d["width_mm"]:.3f}', st_cc),
                         Paragraph(f'{d["area_px"]:,}', st_cc), Paragraph(f'{d["length_px"]:.1f}', st_cc),
                         Paragraph(f'{d["width_px"]:.1f}', st_cc), Paragraph(f'{d["elong"]:.1f}', st_cc),
                         Paragraph(f'{d["x"]}, {d["y"]}', st_cc)])
        if not shown:
            data.append([Paragraph(T["nodef"], st_cc)] + [""] * (len(head) - 1))
        sty = [("GRID", (0, 0), (-1, -1), 0.5, grid),
               ("BACKGROUND", (0, 0), (-1, 0), navy),
               ("ALIGN", (0, 0), (-1, -1), "CENTER"),
               ("VALIGN", (0, 0), (-1, -1), "MIDDLE")]
        if not shown:
            sty.append(("SPAN", (0, 1), (-1, 1)))
        for r in range(2, len(data), 2):
            sty.append(("BACKGROUND", (0, r), (-1, r), colors.HexColor("#F6F8FB")))
        dt = Table(data, colWidths=[w * mm for w in (9, 23, 27, 11, 14, 13, 18, 14, 12, 13, 21)], repeatRows=1)
        dt.setStyle(TableStyle(sty))
        story.append(dt)
        if len(rec["defects"]) > len(shown):
            story.append(Paragraph(T["omit"].format(n=len(rec["defects"]) - len(shown)), st_note))

        crit = T["crit"].format(blob=rec["blob_mm"], k=rec["k_sigma"], seed=rec["k_seed"], um=rec["um_per_px"])
        story += [Spacer(1, 2 * mm), Paragraph(crit, st_note), Spacer(1, 3 * mm)]

        sign = Table([T["sign"], ["", "", ""]], colWidths=[22 * mm] * 3, rowHeights=[6 * mm, 14 * mm])
        sign.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.5, grid), ("FONTNAME", (0, 0), (-1, -1), font),
                                  ("FONTSIZE", (0, 0), (-1, -1), 8), ("ALIGN", (0, 0), (-1, -1), "CENTER"),
                                  ("BACKGROUND", (0, 0), (-1, 0), light)]))
        story.append(KeepTogether(Table([["", sign]], colWidths=[108 * mm, 68 * mm])))

        def footer(canvas, doc):
            canvas.saveState()
            canvas.setFont(font, 7)
            canvas.setFillColor(colors.HexColor("#777777"))
            canvas.drawString(15 * mm, 9 * mm, f"{WIN}  |  {rec['sn']}  |  {rec['time']}")
            canvas.drawRightString(A4[0] - 15 * mm, 9 * mm, f"Page {doc.page}")
            canvas.restoreState()

        doc = SimpleDocTemplate(str(path), pagesize=A4, leftMargin=14 * mm, rightMargin=14 * mm,
                                topMargin=12 * mm, bottomMargin=15 * mm,
                                title=f"{T['title']} {rec['sn']}", author="Wuxi Excelsior")
        doc.build(story, onFirstPage=footer, onLaterPages=footer)
        with self.lock:
            rec[f"pdf_{lang}"] = str(path.relative_to(self.root)).replace("\\", "/")
            self.log_path.write_text(json.dumps(self.records, ensure_ascii=False, indent=1), encoding="utf-8")
        return path

    # ---------------------------------------------------------- Excel
    def write_excel(self, lang: str = "KO") -> Path:
        """일자별 기록(JSON)으로 Excel 전체 재생성 — openpyxl 은 기존 파일 이미지를 보존하지 못하므로."""
        from openpyxl import Workbook
        from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
        from openpyxl.utils import get_column_letter

        T = TXT[lang]
        path = self.root / f"InspectionReport_{self.day}_{lang}.xlsx"
        with self.lock:
            records = list(self.records)

        f_name = T["x_font"]
        thin = Side(style="thin", color="9AA5B8")
        border = Border(left=thin, right=thin, top=thin, bottom=thin)
        head_fill = PatternFill("solid", fgColor="1F3864")
        head_font = Font(name=f_name, bold=True, color="FFFFFF", size=10)
        body_font = Font(name=f_name, size=10)
        center = Alignment(horizontal="center", vertical="center", wrap_text=True)
        ok_fill, ng_fill = PatternFill("solid", fgColor="E2F0D9"), PatternFill("solid", fgColor="FCE4E4")
        ok_font = Font(name=f_name, bold=True, color="008A3E", size=11)
        ng_font = Font(name=f_name, bold=True, color="C00000", size=11)

        wb = Workbook()

        # ---- 시트 1: 검사이력
        ws = wb.active
        ws.title = T["x_sheet1"]
        ws["A1"] = T["x_title"].format(title=T["title"], d=f"{self.day[:4]}.{self.day[4:6]}.{self.day[6:]}")
        ws["A1"].font = Font(name=f_name, bold=True, size=14, color="1F3864")
        n_ok = sum(r["verdict"] == "OK" for r in records)
        n_ng = len(records) - n_ok
        rate = (n_ok / len(records) * 100) if records else 0
        ws["A2"] = T["x_summary"].format(n=len(records), ok=n_ok, ng=n_ng, rate=rate)
        ws["A2"].font = Font(name=f_name, size=9, color="555555")

        head = T["x_head1"]
        widths = [6, 22, 20, 13, 9, 20, 10, 11, 8, 12, 14, 14, 12, XL_RESULT_W / 7 + 2, 12, 22]
        img_col = 13                     # 0 기준: 결과 이미지 열
        hr = 4
        for c, (h, w) in enumerate(zip(head, widths), 1):
            cell = ws.cell(hr, c, h)
            cell.fill, cell.font, cell.alignment, cell.border = head_fill, head_font, center, border
            ws.column_dimensions[get_column_letter(c)].width = w
        ws.row_dimensions[hr].height = 32
        for i, r in enumerate(records):
            row = hr + 1 + i
            is_ng = r["verdict"] == "NG"
            vals = [r["seq"], r["sn"], r["time"], r["cycle_ms"], r["verdict"],
                    rec_types(r, lang) if is_ng else T["good_short"],
                    r["n_line"], r["n_blob"], r["n_dust"], r["max_size_mm"], r["total_ng_px"], r["total_px"],
                    r["um_per_px"], "", "", r.get("source", "")]
            for c, v in enumerate(vals, 1):
                cell = ws.cell(row, c, v)
                cell.font, cell.alignment, cell.border = body_font, center, border
            ws.cell(row, 4).number_format = "0.0"
            ws.cell(row, 10).number_format = "0.000"
            ws.cell(row, 11).number_format = "#,##0"
            ws.cell(row, 12).number_format = "#,##0"
            for c in (5, 6):
                vc = ws.cell(row, c)
                vc.fill, vc.font = (ng_fill, ng_font) if is_ng else (ok_fill, ok_font)
            ws.row_dimensions[row].height = XL_RESULT_H * 0.75 + 8
            tp = self.root / r["result_thumb"]
            if tp.exists():
                place_image(ws, str(tp), img_col, row - 1, XL_RESULT_W, XL_RESULT_H)
            pdf_rel = r.get(f"pdf_{lang}") or (r.get("pdf") if lang == "KO" else None)
            if pdf_rel:
                lc = ws.cell(row, img_col + 2, T["x_open"])
                lc.hyperlink = pdf_rel
                lc.font = Font(name=f_name, size=10, color="0563C1", underline="single")
        ws.freeze_panes = ws.cell(hr + 1, 3)
        if records:
            ws.auto_filter.ref = f"A{hr}:{get_column_letter(len(head))}{hr + len(records)}"

        # ---- 시트 2: 결함상세
        ws2 = wb.create_sheet(T["x_sheet2"])
        head2 = T["x_head2"]
        widths2 = [22, 20, 9, 8, 14, 9, 10, 9, 11, 12, 10, 9, 8, 7, 8, 8, XL_CROP / 7 + 2]
        for c, (h, w) in enumerate(zip(head2, widths2), 1):
            cell = ws2.cell(1, c, h)
            cell.fill, cell.font, cell.alignment, cell.border = head_fill, head_font, center, border
            ws2.column_dimensions[get_column_letter(c)].width = w
        ws2.row_dimensions[1].height = 32
        row = 2
        for r in records:
            for d in r["defects"]:
                vals = [r["sn"], r["time"], r["verdict"], d["no"], defect_name(d, lang),
                        "NG" if d["ng"] else "OK", d["size_mm"], d["width_mm"], d["area_mm2"],
                        d["area_px"], d["length_px"], d["width_px"], d["elong"],
                        "Y" if d["colored"] else "", d["x"], d["y"], ""]
                for c, v in enumerate(vals, 1):
                    cell = ws2.cell(row, c, v)
                    cell.font, cell.alignment, cell.border = body_font, center, border
                for c, fmt in ((7, "0.000"), (8, "0.000"), (9, "0.0000"), (10, "#,##0"),
                               (11, "0.0"), (12, "0.0"), (13, "0.0")):
                    ws2.cell(row, c).number_format = fmt
                for c in (3, 6):
                    vc = ws2.cell(row, c)
                    vc.fill, vc.font = (ng_fill, ng_font) if vc.value == "NG" else (ok_fill, ok_font)
                cp = self.root / d["crop"] if d["crop"] else None
                if cp is not None and cp.exists():
                    place_image(ws2, str(cp), len(head2) - 1, row - 1, XL_CROP, XL_CROP)
                    ws2.row_dimensions[row].height = XL_CROP * 0.75 + 8
                else:
                    ws2.row_dimensions[row].height = 20
                row += 1
        ws2.freeze_panes = "C2"
        if row > 2:
            ws2.auto_filter.ref = f"A1:{get_column_letter(len(head2))}{row - 1}"

        # ---- 시트 3: 판정기준
        ws3 = wb.create_sheet(T["x_sheet3"])
        for i, (a, b) in enumerate(T["x_crit"], 1):
            for c, v in ((1, a), (2, b)):
                cell = ws3.cell(i, c, v)
                cell.border = border
                cell.font = head_font if i == 1 else body_font
                cell.alignment = Alignment(vertical="center", wrap_text=True)
                if i == 1:
                    cell.fill = head_fill
        ws3.column_dimensions["A"].width = 34
        ws3.column_dimensions["B"].width = 70

        for w in (ws, ws2):
            w.page_setup.orientation = "landscape"
            w.page_setup.fitToWidth = 1
            w.page_setup.fitToHeight = 0
            w.sheet_properties.pageSetUpPr.fitToPage = True
            w.print_title_rows = f"{hr}:{hr}" if w is ws else "1:1"

        self.root.mkdir(parents=True, exist_ok=True)
        try:
            wb.save(str(path))
        except PermissionError:
            alt = path.with_name(f"{path.stem}_{datetime.now().strftime('%H%M%S')}.xlsx")
            wb.save(str(alt))
            print(f"※ {path.name} 파일이 열려 있어 별도 파일로 저장했습니다. Excel을 닫으면 다음 기록부터 원래 파일에 저장됩니다.")
            return alt
        return path


def make_report(reporter: Reporter, sn: str, img, res: Result, p: Params, source: str,
                excel: bool = True) -> None:
    rec = reporter.add(sn, img, res, p, source)
    paths = reporter.export(rec, excel=excel)
    print(f"성적서 [{rec['verdict']}] SN={sn}  {rec['cycle_ms']:.1f}ms")
    for pth in paths:
        print(f"    → {pth.resolve()}")


# ================================================================ 트랙바
TB = [
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
def run_viewer(paths: list[Path], p: Params, out_dir: Path, reporter: Reporter, sn_prefix: str) -> int:
    create_window()
    make_trackbars(p)
    display, refit = Display(), False
    idx, show_dust, show_map = 0, False, False
    cache_key, img, res = None, None, None
    print("N/P 이미지 이동, D 먼지 표시, M 응답맵, F 창 맞춤, S 저장, R 성적서(PDF·Excel), Q 종료")
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
        display.show(side_by_side(img, vis), refit)
        refit = False
        k = cv2.waitKeyEx(50)
        if k in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            break
        if k in (ord("n"), ord("N"), 2555904, 65363):
            idx = (idx + 1) % len(paths)
        elif k in (ord("p"), ord("P"), 2424832, 65361):
            idx = (idx - 1) % len(paths)
        elif k in (ord("d"), ord("D")):
            show_dust = not show_dust
        elif k in (ord("f"), ord("F")):
            refit = True
        elif k in (ord("m"), ord("M")):
            show_map = not show_map
        elif k in (ord("s"), ord("S")):
            save(out_dir, paths[idx].name, img, res, p)
            print(f"저장 → {out_dir.resolve()}")
        elif k in (ord("r"), ord("R")) or display.take_click():
            sn = ask_sn(f"{sn_prefix}{paths[idx].stem}")
            if sn:
                make_report(reporter, sn, img, res, p, paths[idx].name)
    cv2.destroyAllWindows()
    return 0


def run_batch(paths: list[Path], p: Params, out_dir: Path, reporter: Reporter | None, sn_prefix: str) -> int:
    rows = []
    for path in paths:
        img = imread_any(path)
        if img is None:
            print(f"읽기 실패: {path}")
            continue
        res = inspect(img, p)
        report(path.name, res)
        save(out_dir, path.name, img, res, p)
        if reporter is not None:
            make_report(reporter, f"{sn_prefix}{path.stem}", img, res, p, path.name, excel=False)
        ngs = [d for d in res.defects if d.ng]
        rows.append([path.name, "NG" if res.ng else "OK", f"{res.cycle_ms:.1f}",
                     sum(d.kind == "LINE" for d in ngs), sum(d.kind == "BLOB" for d in ngs),
                     f"{max((d.length_mm for d in ngs), default=0):.3f}", sum(d.area_px for d in ngs)])
    with (out_dir / "summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["image", "result", "cycle_ms", "line_ng", "blob_ng", "max_size_mm", "ng_pixels"])
        w.writerows(rows)
    if reporter is not None and reporter.fmt in ("excel", "both") and rows:
        for pth in reporter.write_excel_all():
            print(f"Excel → {pth.resolve()}")
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
    """축소 해상도 결과를 원본 해상도 좌표·픽셀 수로 환산."""
    if s == 1.0:
        return res
    for d in res.defects:
        d.contour = (d.contour.astype(np.float32) / s).astype(np.int32)
        d.area_px = int(round(d.area_px / (s * s)))
        d.length_px /= s
        d.width_px /= s
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
        self.result = None          # (frame, Result, Params)
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
            t0 = time.perf_counter()
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
            res.cycle_ms = (time.perf_counter() - t0) * 1000.0     # 축소 포함 전체 소요시간
            now = time.time()
            self.fps = 0.8 * self.fps + 0.2 * (1.0 / max(now - t_last, 1e-3))
            t_last = now
            with self.lock:
                self.result = (frame, res, p)


def run_camera(index: int | None, p: Params, out_dir: Path, args, reporter: Reporter) -> int:
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
    print("[RECORD] 버튼 또는 R 키 = 성적서(PDF·Excel) 생성 | Space 일시정지, D 먼지, M 응답맵, F 창 맞춤, S 저장, Q 종료")

    worker = Inspector(args.scale, args.angle_every)
    worker.start()
    create_window()
    make_trackbars(p)
    display, refit = Display(), False

    history = deque(maxlen=args.stable_n)
    show_dust, show_map, paused = False, False, False
    last_seen_id, last_save, last_report = None, 0.0, 0.0
    stable_prev = False
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
                rframe, res, rp = latest
                if id(res) != last_seen_id:
                    last_seen_id = id(res)
                    history.append(res.ng)
                    prev_stable = stable_prev
                    stable_now = sum(history) >= args.stable_m
                    stable_prev = stable_now
                    if args.save_ng and res.ng and time.time() - last_save > args.save_interval:
                        name = time.strftime("ng_%Y%m%d_%H%M%S.png")
                        imwrite_any(out_dir / name.replace(".png", "_raw.png"), rframe)
                        save(out_dir, name, rframe, res, rp)
                        last_save = time.time()
                        print(f"NG 자동 저장: {name}")
                    if (args.auto_report and res.ng and stable_now and not prev_stable
                            and time.time() - last_report > args.report_interval):   # 새 불량품 1회
                        make_report(reporter, f"{args.sn}{reporter.next_sn()}" if args.sn else reporter.next_sn(),
                                    rframe, res, rp, f"CAM{index}")
                        last_report = time.time()
                base = frame if not paused else rframe
                disp = draw(base, res, rp, show_dust, show_map, f"CAM{index}")
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
            display.show(disp, refit)
            refit = False

            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k == ord(" "):
                paused = not paused
            elif k in (ord("d"), ord("D")):
                show_dust = not show_dust
            elif k in (ord("f"), ord("F")):
                refit = True
            elif k in (ord("m"), ord("M")):
                show_map = not show_map
            elif k in (ord("s"), ord("S")) and latest is not None:
                name = time.strftime("cam_%Y%m%d_%H%M%S.png")
                imwrite_any(out_dir / name.replace(".png", "_raw.png"), latest[0])
                save(out_dir, name, latest[0], latest[1], latest[2])
                report(name, latest[1])
                print(f"저장 → {out_dir.resolve()}")
            elif (k in (ord("r"), ord("R")) or display.take_click()) and latest is not None:
                snap = latest                           # 키 누른 시점의 결과 고정
                default = f"{args.sn}{reporter.next_sn()}" if args.sn else reporter.next_sn()
                sn = ask_sn(default)
                if sn:
                    make_report(reporter, sn, snap[0], snap[1], snap[2], f"CAM{index}")
    finally:
        worker.running = False
        worker.join(timeout=1.0)
        cap.release()
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="O-ring 결함 검출 PoC V10 (PDF·Excel 성적서)")
    ap.add_argument("--folder", type=Path, help="검사 이미지 폴더")
    ap.add_argument("--image", type=Path, help="이미지 1장")
    ap.add_argument("--camera", type=int, help="카메라 번호 (생략 시 검색 후 선택)")
    ap.add_argument("--list-cameras", action="store_true", help="카메라 목록만 출력")
    ap.add_argument("--cam-width", type=int, default=1920)
    ap.add_argument("--cam-height", type=int, default=1080)
    ap.add_argument("--scale", type=float, default=0.5, help="검사 처리 해상도 배율(1.0=원본, 0.5=절반)")
    ap.add_argument("--angle-every", type=int, default=10, help="띠 방향 재계산 주기(프레임)")
    ap.add_argument("--stable-n", type=int, default=5, help="판정 안정화: 최근 N회")
    ap.add_argument("--stable-m", type=int, default=3, help="판정 안정화: N회 중 M회 이상 NG면 NG")
    ap.add_argument("--save-ng", action="store_true", help="NG 프레임 자동 저장")
    ap.add_argument("--save-interval", type=float, default=2.0, help="NG 자동 저장·자동 성적서 최소 간격(초)")
    ap.add_argument("--batch", action="store_true", help="창 없이 전체 판정 후 저장")
    ap.add_argument("--report", action="store_true", help="배치 모드에서 이미지마다 성적서 생성")
    ap.add_argument("--auto-report", action="store_true", help="실시간 NG 발생 시 성적서 자동 생성 (기본: 버튼/R 키로만)")
    ap.add_argument("--report-interval", type=float, default=3.0, help="자동 성적서 최소 간격(초)")
    ap.add_argument("--report-format", choices=["pdf", "excel", "both"], default="both", help="성적서 출력 형식")
    ap.add_argument("--lang", choices=["ko", "cn", "both"], default="both", help="성적서 언어 (한국어/중국어/둘 다)")
    ap.add_argument("--no-bmp", action="store_true", help="원본 BMP 저장 끄기")
    ap.add_argument("--sn", default="", help="SN 접두어 (배치: 접두어+파일명, 실시간: 접두어+자동번호)")
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
    global SAVE_BMP
    SAVE_BMP = not args.no_bmp
    langs = {"ko": ("KO",), "cn": ("CN",), "both": ("KO", "CN")}[args.lang]
    reporter = Reporter(out_dir, args.report_format, langs)

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

    if args.image is None and args.folder is None and not args.batch:
        return run_camera(args.camera, p, out_dir, args, reporter)

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

    if args.batch:
        return run_batch(paths, p, out_dir, reporter if args.report else None, args.sn)
    return run_viewer(paths, p, out_dir, reporter, args.sn)


if __name__ == "__main__":
    sys.exit(main())
