#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BSV Slit Valve O-ring 정밀 검사 설비 — 실시간 3D 시뮬레이터 V4
(콘셉트 이미지 형상 반영: 양 끝 10단 입·출력 랙, 하부 상향 파지, 중앙 상부 POMEAS 고정 카메라 + 정렬 지그,
 전면 하부 제7축 주행, 우측 전장함)

설비 외형(목표 예시): 약 4,200(L) × 1,600(W) × 2,200(H) mm

사이클 (입력 랙 제품 1개당)
  1) 입력 랙 해당 단 앞으로 주행 → 포크형 엔드이펙터를 제품 아래로 수평 진입
  2) 포크 내장 카메라로 제품 하면 SN(Serial Number, 일련번호) 판독
  3) 상향 리프트 → 그리퍼 파지 → 인출
  4) 정렬 지그에 안착 → 위치 보정 → 재파지
  5) POMEAS 카메라 초점 높이로 들어 올려 O-ring 윤곽 24점 순차 정밀검사
  6) 출력 랙 양품(OK) 단 또는 불량(NG) 단에 하부에서 밀어 넣어 적재

실행 : python bsv_robot_cell_sim_v4.py
설치 : pip install pyvista     (numpy, vtk 자동 설치)
환경 : Python 3.12

조작
  마우스 왼쪽 드래그 회전 / 휠 확대·축소 / Shift+드래그 이동
  Space 재생·일시정지   Up/Down 재생 속도 ×2·÷2   k 투명 도어·커버   n 공장 환경   q 종료
  화면 왼쪽 아래 체크박스: 재생 / 커버

옵션
  --no-factory : 공장 환경 없이 설비만 표시
  --snap DIR   : 창 없이 사이클 주요 시점 스크린샷 저장 (검증용)
  --vtk-timer  : VTK 타이머 방식 재생 (기본은 직접 루프 — Windows에서 타이머 미동작 대응)

※ 로봇 동작은 시각화용 간이 기구학(어깨 측방 오프셋 + 2링크). 실제 CS66 역기구학·충돌검사 아님.
※ 랙 단 피치·포크 두께·도달범위 등은 개념값 — 그리퍼 실물 치수와 로봇 제조사 시뮬레이터로 재확인 필요.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    import pyvista as pv
except ImportError:  # pragma: no cover
    sys.exit("pyvista가 필요합니다:  pip install pyvista")

# ===========================================================================
# 색상
# ===========================================================================
COL = {
    "steel": (0.25, 0.26, 0.28),
    "steel_m": (0.42, 0.43, 0.45),
    "alu": (0.80, 0.81, 0.83),
    "alu_d": (0.60, 0.61, 0.64),
    "white": (0.93, 0.94, 0.95),
    "white_d": (0.82, 0.84, 0.86),
    "navy": (0.07, 0.11, 0.24),
    "led": (0.20, 0.55, 1.00),
    "glass": (0.72, 0.84, 0.94),
    "rail": (0.10, 0.10, 0.11),
    "rail_cover": (0.78, 0.79, 0.81),
    "guide": (0.72, 0.73, 0.75),
    "robot": (0.94, 0.94, 0.93),
    "robot_j": (0.22, 0.23, 0.25),
    "robot_b": (0.20, 0.40, 0.80),
    "grip": (0.20, 0.30, 0.62),
    "grip_j": (0.55, 0.57, 0.60),
    "chain": (0.12, 0.12, 0.13),
    "product": (0.86, 0.88, 0.92),
    "oring": (0.04, 0.04, 0.04),
    "yellow": (0.96, 0.72, 0.10),
    "cam": (0.08, 0.09, 0.11),
    "lens": (0.25, 0.27, 0.30),
    "ringlight": (0.25, 0.55, 1.00),
    "beam": (0.55, 0.75, 1.00),
    "ok": (0.15, 0.62, 0.30),
    "ng": (0.85, 0.15, 0.15),
    "bolt": (0.74, 0.74, 0.72),
    "screen": (0.10, 0.25, 0.50),
    "floor": (0.80, 0.82, 0.83),
    "floor2": (0.76, 0.78, 0.79),
    "fyellow": (0.95, 0.78, 0.10),
    "fgreen": (0.30, 0.62, 0.40),
    "wall": (0.90, 0.91, 0.92),
    "column": (0.62, 0.64, 0.67),
    "machine": (0.90, 0.91, 0.92),
    "machine_d": (0.30, 0.34, 0.40),
    "person": (0.20, 0.35, 0.55),
    "skin": (0.85, 0.70, 0.58),
}


def nrm(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


# ===========================================================================
# 간이 장면 그래프 (면 목록 → 재질별 PolyData)
# ===========================================================================
@dataclass
class Face:
    v: np.ndarray
    color: tuple
    alpha: float
    spec: float
    tag: str
    edge: bool


class Scene:
    def __init__(self):
        self.faces: list[Face] = []

    def poly(self, v, color, alpha=1.0, spec=0.15, tag="", edges=True):
        self.faces.append(Face(np.asarray(v, float), color, alpha, spec, tag, edges))

    def obox(self, c, R, h, color, **kw):
        c = np.asarray(c, float)
        R = np.asarray(R, float)
        h = np.asarray(h, float)
        for a in range(3):
            b, cc = (a + 1) % 3, (a + 2) % 3
            for sgn in (-1.0, 1.0):
                base = c + sgn * h[a] * R[a]
                eb, ec = R[b] * h[b], R[cc] * h[cc]
                self.poly([base - eb - ec, base + eb - ec, base + eb + ec, base - eb + ec],
                          color, **kw)

    def box(self, p0, p1, color, **kw):
        p0 = np.asarray(p0, float)
        p1 = np.asarray(p1, float)
        lo, hi = np.minimum(p0, p1), np.maximum(p0, p1)
        self.obox((lo + hi) / 2, np.eye(3), (hi - lo) / 2, color, **kw)

    def bar(self, a, b, w, t, color, up=(0, 0, 1), **kw):
        a = np.asarray(a, float)
        b = np.asarray(b, float)
        ax = nrm(b - a)
        up = nrm(up)
        if abs(ax @ up) > 0.99:
            up = np.array([1.0, 0, 0]) if abs(ax[0]) < 0.9 else np.array([0, 1.0, 0])
        side = nrm(np.cross(ax, up))
        u2 = np.cross(side, ax)
        L = float(np.linalg.norm(b - a))
        self.obox((a + b) / 2, np.array([ax, side, u2]), (L / 2, w / 2, t / 2), color, **kw)

    def cyl(self, a, b, r, color, n=24, caps=True, side_edges=False, **kw):
        a = np.asarray(a, float)
        b = np.asarray(b, float)
        ax = nrm(b - a)
        ref = np.array([0, 0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0, 0])
        p = nrm(np.cross(ax, ref))
        q = np.cross(ax, p)
        ang = np.linspace(0, 2 * np.pi, n + 1)
        ra = [a + r * (np.cos(t) * p + np.sin(t) * q) for t in ang]
        rb = [b + r * (np.cos(t) * p + np.sin(t) * q) for t in ang]
        for k in range(n):
            self.poly([ra[k], ra[k + 1], rb[k + 1], rb[k]], color,
                      **dict(kw, edges=side_edges))
        if caps:
            self.poly(ra[:-1], color, **dict(kw, edges=kw.get("edges", True)))
            self.poly(rb[:-1], color, **dict(kw, edges=kw.get("edges", True)))

    def cone(self, apex, base_c, r, color, n=32, **kw):
        apex = np.asarray(apex, float)
        base_c = np.asarray(base_c, float)
        ax = nrm(base_c - apex)
        ref = np.array([0, 0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0, 0])
        p = nrm(np.cross(ax, ref))
        q = np.cross(ax, p)
        ang = np.linspace(0, 2 * np.pi, n + 1)
        ring = [base_c + r * (np.cos(t) * p + np.sin(t) * q) for t in ang]
        for k in range(n):
            self.poly([apex, ring[k], ring[k + 1]], color, **dict(kw, edges=False))

    def hexbolt(self, p, axis=(0, 0, 1), s=10.0, h=8.0, **kw):
        p = np.asarray(p, float)
        self.cyl(p, p + nrm(axis) * h, s, COL["bolt"], n=6, side_edges=True, spec=0.45, **kw)


def scene_to_groups(sc: Scene, tags=None, excl=()):
    groups: dict = {}
    for F in sc.faces:
        if F.tag in excl or (tags is not None and F.tag not in tags):
            continue
        key = (tuple(round(x, 3) for x in F.color), round(F.alpha, 3), round(F.spec, 2),
               bool(F.edge), F.tag == "led")
        pts, fc, cnt = groups.setdefault(key, ([], [], [0]))
        n = len(F.v)
        pts.append(F.v)
        fc.append(np.concatenate([[n], np.arange(cnt[0], cnt[0] + n)]))
        cnt[0] += n
    return {k: pv.PolyData(np.vstack(p), np.concatenate(f).astype(np.int64))
            for k, (p, f, _) in groups.items()}


def add_group(pl, key, mesh):
    color, alpha, spec, edge, emis = key
    kw = dict(color=color, opacity=alpha, show_edges=edge and alpha >= 0.99,
              edge_color=tuple(c * 0.45 for c in color), line_width=0.7,
              specular=min(1.0, spec * 1.5), specular_power=25, smooth_shading=False)
    if emis:
        kw.update(ambient=0.85, diffuse=0.3, show_edges=False)
    return pl.add_mesh(mesh, **kw)


# ===========================================================================
# 설계 파라미터 (mm) — X: 설비 길이(좌→우), Y: 깊이(전면 −), Z: 높이
# ===========================================================================
ENC_X0, ENC_X1 = -2100.0, 1500.0       # 본체 (3개 베이 × 1,200)
CAB_X0, CAB_X1 = 1500.0, 2100.0        # 우측 전장함 → 전체 4,200
ENC_Y = 800.0                          # 깊이 1,600
Z_PLINTH = 250.0                       # 하부 플린스 상단
Z_HEADER = 1950.0                      # 상부 헤더 하단
Z_TOP = 2200.0
BAYS = [-2100.0, -900.0, 300.0, 1500.0]

RY = -330.0                            # 제7축 중심 Y (전면 하부)
RAIL_X0, RAIL_X1 = -1960.0, 1360.0
CARR_MIN, CARR_MAX = -1800.0, 1200.0   # 캐리지 중심 이동범위 = 유효 스트로크 3,000
STATIONS = [-1800.0, -300.0, 1200.0]
Z_BEAM = (120.0, 270.0)
Z_TOPPL = (270.0, 278.0)
Z_PAD = (278.0, 290.0)
Z_RAIL = (290.0, 400.0)
Z_GUIDE = (400.0, 415.0)
Z_CARR = (415.0, 455.0)
Z_BASE = 560.0                         # 로봇 베이스면 (라이저 105)
Z_SH = Z_BASE + 154.0
LINK_OFF = 200.0                       # 어깨 측방 오프셋
L1, L2 = 420.0, 390.0                  # 상완·전완 (CS66급 도달 914 mm 내 개념값)
W2F = 118.0                            # 손목 중심 → 툴 플랜지

PROD_L, PROD_W, PROD_T = 360.0, 60.0, 20.0
TIERS = 10
TIER_Z0, TIER_P = 520.0, 105.0         # 최하단 제품 하면 높이, 단 피치
RACK_Y = 50.0                          # 랙 제품 중심 Y
XI, XO = -1775.0, 1150.0               # 입력 랙 / 출력 랙 제품 중심 X
NG_TIERS = (8, 9)                      # 출력 랙 상부 2단 = 불량(NG)

CX, CY = -150.0, 150.0                 # 정밀 카메라 광축 위치
Z_LENS = 1450.0                        # 링조명 하면
WD = 87.0                              # 작동거리 (POMEAS LZG-63104-A 표기값 기준)
Z_FOCUS = Z_LENS - WD                  # 제품 상면이 와야 할 높이
Z_JIG = 1250.0                         # 정렬 지그 제품 지지면

# 포크형 엔드이펙터 (툴 좌표: t=+Y 진입방향, b=−X, c=+Z)
T_PROD = 150.0                         # 플랜지 → 제품 중심 (t 방향)
C_PROD = 45.0                          # 플랜지 → 제품 중심 (c 방향, 포크 상면 35 + 두께/2)
GAP = 25.0                             # 진입 시 포크 상면과 제품 하면 간격
R_TOOL = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # 행: t, b, c


def tier_z(k):
    return TIER_Z0 + TIER_P * k


def find_font():
    for p in (r"C:\Windows\Fonts\malgun.ttf",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
              "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
              "/System/Library/Fonts/AppleSDGothicNeo.ttc"):
        if Path(p).exists():
            return p
    return None


# ===========================================================================
# 정적 형상
# ===========================================================================
def build_steel_base(sc: Scene):
    """제7축 하부 강재 베이스: 트윈 스파인 + 상판(폐단면) + 삼각 아웃리거 스테이션 3개소"""
    st = COL["steel"]
    T = dict(tag="steel")
    x0, x1 = RAIL_X0 - 60, RAIL_X1 + 60
    for s in (-1, 1):
        yc = RY + s * 90
        sc.box((x0, yc - 40, Z_BEAM[0]), (x1, yc + 40, Z_BEAM[1]), st, **T)
    sc.box((x0, RY - 135, Z_TOPPL[0]), (x1, RY + 135, Z_TOPPL[1]), st, **T)
    for px in np.linspace(RAIL_X0 + 150, RAIL_X1 - 150, 9):
        sc.box((px - 70, RY - 130, Z_PAD[0]), (px + 70, RY + 130, Z_PAD[1]), COL["steel_m"],
               spec=0.35, tag="steel")
        for dx in (-50, 50):
            for s in (-1, 1):
                sc.hexbolt((px + dx, RY + s * 120, Z_PAD[1]), s=7, h=6, tag="steel")
    xs = np.linspace(x0 + 40, x1 - 40, 9)
    for i in range(8):
        ya = RY - 90 if i % 2 == 0 else RY + 90
        sc.bar((xs[i], ya, Z_BEAM[0] - 5), (xs[i + 1], 2 * RY - ya, Z_BEAM[0] - 5), 50, 8, st, **T)
    for xs_ in STATIONS:
        y0, y1 = RY - 380, RY + 380
        sc.box((xs_ - 45, y0, 30), (xs_ + 45, y1, 110), st, **T)
        for s in (-1, 1):
            yc = RY + s * 90
            sc.box((xs_ - 40, yc - 40, 110), (xs_ + 40, yc + 40, Z_BEAM[0]), st, **T)
            sc.bar((xs_, RY + s * 350, 110), (xs_, RY + s * 132, Z_BEAM[1] - 25), 60, 60, st,
                   up=(1, 0, 0), **T)
            yf = RY + s * 330
            sc.cyl((xs_, yf, 0), (xs_, yf, 12), 40, COL["bolt"], n=20, tag="steel")
            sc.cyl((xs_, yf, 12), (xs_, yf, 30), 10, COL["bolt"], n=10, tag="steel")
            ya_ = RY + s * 380
            sc.box((xs_ - 60, min(ya_, ya_ + s * 90), 0), (xs_ + 60, max(ya_, ya_ + s * 90), 12),
                   st, **T)
            for dx in (-35, 35):
                sc.hexbolt((xs_ + dx, ya_ + s * 50, 12), s=11, h=9, tag="steel")


def build_rail_static(sc: Scene):
    r = COL["rail"]
    sc.box((RAIL_X0, RY - 110, Z_RAIL[0]), (RAIL_X1, RY + 110, Z_RAIL[1]), COL["rail_cover"],
           spec=0.35, tag="rail")
    sc.box((RAIL_X0, RY - 120, Z_RAIL[0]), (RAIL_X1, RY + 120, Z_RAIL[0] + 10), r, tag="rail")
    for s in (-1, 1):
        sc.box((RAIL_X0, RY + s * 75 - 14, Z_GUIDE[0]), (RAIL_X1, RY + s * 75 + 14, Z_GUIDE[1]),
               COL["guide"], spec=0.5, tag="rail")
    for x in (RAIL_X0 - 30, RAIL_X1):
        sc.box((x, RY - 110, Z_RAIL[0]), (x + 30, RY + 110, Z_GUIDE[1] + 20), COL["yellow"],
               tag="rail")
    # 케이블 체인 트로프 (레일 후방)
    sc.box((RAIL_X0 + 100, RY + 130, 282), (RAIL_X1 - 100, RY + 230, 290), COL["alu_d"], tag="rail")
    for yy in (RY + 130, RY + 224):
        sc.box((RAIL_X0 + 100, yy, 290), (RAIL_X1 - 100, yy + 6, 380), COL["alu_d"], tag="rail")


def build_rack(sc: Scene, xc, label_col_fn, title):
    """10단 개방형 랙: 제품 양 끝만 받침(노랑) → 중앙 하부 개방 (포크 진입·SN 판독)"""
    al = COL["alu"]
    T = dict(tag="rack", spec=0.35)
    y_f, y_b = RACK_Y - 75, RACK_Y + 85
    xl, xr = xc - PROD_L / 2 - 40, xc + PROD_L / 2 + 40
    for x in (xl, xr):
        for y in (y_f, y_b):
            sc.box((x - 20, y - 20, Z_PLINTH), (x + 20, y + 20, tier_z(TIERS - 1) + 160), al, **T)
        sc.box((x - 20, y_f, Z_PLINTH), (x + 20, y_b, Z_PLINTH + 40), al, **T)
        sc.box((x - 20, y_f, tier_z(TIERS - 1) + 120), (x + 20, y_b, tier_z(TIERS - 1) + 160), al,
               **T)
    for y in (y_f, y_b):
        sc.box((xl, y - 20, tier_z(TIERS - 1) + 120), (xr, y + 20, tier_z(TIERS - 1) + 160), al, **T)
    sc.box((xl, y_b - 20, Z_PLINTH), (xr, y_b + 20, Z_PLINTH + 40), al, **T)
    # 후면 X 브레이스
    zt = tier_z(TIERS - 1) + 120
    sc.bar((xl, y_b, Z_PLINTH + 40), (xr, y_b, zt), 30, 6, COL["alu_d"], up=(0, 1, 0), **T)
    sc.bar((xr, y_b, Z_PLINTH + 40), (xl, y_b, zt), 30, 6, COL["alu_d"], up=(0, 1, 0), **T)
    for k in range(TIERS):
        z = tier_z(k)
        for s in (-1, 1):
            xe = xc + s * (PROD_L / 2 - 12)
            x_in, x_out = sorted((xe - s * 0, xc + s * (PROD_L / 2 + 20)))
            # 받침 블록 (노랑) + 위치결정 립
            sc.box((min(xe, x_out), y_f + 10, z - 22), (max(xe, x_out), y_b - 10, z), COL["yellow"],
                   tag="rack")
            lip = xc + s * (PROD_L / 2 + 4)
            sc.box((min(lip, lip + s * 16), y_f + 10, z), (max(lip, lip + s * 16), y_b - 10, z + 12),
                   COL["yellow"], tag="rack")
            sc.box((min(xc + s * (PROD_L / 2 + 20), xc + s * (PROD_L / 2 + 40)), y_f + 10, z - 22),
                   (max(xc + s * (PROD_L / 2 + 20), xc + s * (PROD_L / 2 + 40)), y_b - 10, z - 2),
                   COL["alu_d"], tag="rack")
        col = label_col_fn(k)
        if col is not None:
            sc.box((xl - 24, y_f - 2, z - 18), (xl - 20, y_f + 30, z - 4), col, tag="rack")
    # 표지판
    sc.box((xc - 160, y_f - 25, tier_z(TIERS - 1) + 170), (xc + 160, y_f - 20, tier_z(TIERS - 1) + 270),
           COL["led"], tag="rack")
    return title


def build_portal_camera(sc: Scene):
    """정밀 카메라 독립 강재 포털 (X방향 문형, 기둥 2개는 광축 좌우 ±650 — 전면 시야·로봇 동선 회피)
    + POMEAS 광학 헤드"""
    st = COL["steel"]
    T = dict(tag="portal")
    zt = 1900.0
    for xc_ in (CX - 650.0, CX + 650.0):
        sc.box((xc_ - 50, CY - 50, 16), (xc_ + 50, CY + 50, zt - 80), st, **T)
        sc.box((xc_ - 110, CY - 90, 0), (xc_ + 110, CY + 90, 16), st, **T)
        for dx in (-80, 80):
            for dy in (-60, 60):
                sc.hexbolt((xc_ + dx, CY + dy, 16), s=10, h=9, tag="portal")
        s = 1 if xc_ < CX else -1
        sc.bar((xc_ + s * 50, CY, zt - 380), (xc_ + s * 330, CY, zt - 80), 50, 50, st, up=(0, 1, 0), **T)
    sc.box((CX - 700, CY - 60, zt - 80), (CX + 700, CY + 60, zt), st, **T)
    # Z 전동 포커스 슬라이드 + 헤드
    sc.box((CX - 70, CY - 60, 1640), (CX + 70, CY - 45, zt - 80), st, **T)
    sc.box((CX - 40, CY - 45, 1660), (CX + 40, CY - 25, 1800), COL["alu_d"], **T)
    sc.box((CX - 45, CY - 40, 1760), (CX + 45, CY + 50, 1840), COL["cam"], tag="cam")   # 산업용 카메라
    sc.cyl((CX, CY, 1560), (CX, CY, 1760), 55, COL["cam"], n=36, tag="cam", spec=0.4)   # 줌 바디
    sc.cyl((CX, CY, 1500), (CX, CY, 1560), 45, COL["lens"], n=36, tag="cam", spec=0.6)
    sc.cyl((CX, CY, 1470), (CX, CY, 1500), 95, COL["cam"], n=40, tag="cam")
    sc.cyl((CX, CY, Z_LENS), (CX, CY, 1470), 90, COL["ringlight"], n=40, tag="led")
    sc.cone((CX, CY, Z_LENS - 1), (CX, CY, Z_FOCUS), 60, COL["beam"], alpha=0.18, tag="beam")


def build_jig(sc: Scene):
    """정렬·고정 지그 (XY 서보 스테이지 위, 제품 양 끝 받침 → 중앙 하부 개방)"""
    T = dict(tag="jig", spec=0.35)
    y0, y1 = CY - 120, CY + 120
    sc.box((CX - 90, y0 + 30, Z_PLINTH), (CX + 90, y1 - 30, 1080), COL["alu"], **T)    # 기둥
    sc.box((CX - 260, y0, 1080), (CX + 260, y1, 1100), COL["steel"], **T)
    sc.box((CX - 250, y0 + 20, 1100), (CX + 250, y1 - 20, 1130), COL["alu_d"], **T)     # X축
    sc.box((CX - 230, y0 + 30, 1130), (CX + 230, y1 - 30, 1160), COL["alu"], **T)        # Y축
    for s in (-1, 1):
        x_in = CX + s * (PROD_L / 2 - 14)
        x_out = CX + s * (PROD_L / 2 + 26)
        sc.box((min(x_in, x_out), CY - 45, 1160), (max(x_in, x_out), CY + 45, Z_JIG),
               COL["grip_j"], **T)
        sc.box((min(x_out - s * 16, x_out), CY - 45, Z_JIG), (max(x_out - s * 16, x_out), CY + 45,
                                                              Z_JIG + 12), COL["yellow"], tag="jig")
        sc.cyl((x_in + s * 10, CY + 38, Z_JIG), (x_in + s * 10, CY + 38, Z_JIG + 8), 4,
               COL["bolt"], n=10, tag="jig")
    sc.box((CX + 250, y0 + 20, 1100), (CX + 300, y1 - 20, 1160), COL["cam"], tag="jig")  # 서보


def build_enclosure(sc: Scene):
    W, Wd = COL["white"], COL["white_d"]
    T = dict(tag="enc", spec=0.3)
    x0, x1 = ENC_X0, CAB_X1
    # 레벨링 풋
    for x in (x0 + 60, -900.0, 300.0, CAB_X0, x1 - 60):
        for y in (-ENC_Y + 60, ENC_Y - 60):
            sc.cyl((x, y, 0), (x, y, 14), 38, COL["chain"], n=16, tag="enc")
            sc.cyl((x, y, 14), (x, y, 80), 11, COL["bolt"], n=12, tag="enc")
    # 하부 플린스 (흰색 판넬) + 청색 LED 라인
    sc.box((x0, -ENC_Y, 80), (ENC_X1, -ENC_Y + 20, Z_PLINTH), W, **T)
    sc.box((x0, ENC_Y - 20, 80), (ENC_X1, ENC_Y, Z_PLINTH), W, **T)
    for x in (x0, ENC_X1 - 20):
        sc.box((x, -ENC_Y, 80), (x + 20, ENC_Y, Z_PLINTH), W, **T)
    sc.box((x0 + 40, -ENC_Y - 3, 110), (ENC_X1 - 40, -ENC_Y, 118), COL["led"], tag="led")
    # 기둥 (백색 도장 각재)
    for x in BAYS:
        for y in (-ENC_Y, ENC_Y):
            sc.box((x - 40, y - 40 if y > 0 else y, Z_PLINTH), (x + 40, y if y > 0 else y + 40, Z_HEADER),
                   W, **T)
    for x in (x0, ENC_X1):
        for z in (Z_PLINTH, Z_HEADER - 40):
            sc.box((x - 40 if x > x0 else x, -ENC_Y, z), (x + 40 if x > x0 else x + 40, ENC_Y, z + 40), W, **T)
    for y in (-ENC_Y, ENC_Y - 40):
        sc.box((x0, y, Z_PLINTH), (ENC_X1, y + 40, Z_PLINTH + 30), W, **T)
    # 상부 헤더 (전면 남색 밴드 + LED), 지붕
    sc.box((x0, -ENC_Y - 6, Z_HEADER), (ENC_X1, -ENC_Y + 30, Z_TOP), COL["navy"], tag="enc", spec=0.5)
    sc.box((x0 + 40, -ENC_Y - 9, Z_HEADER + 10), (ENC_X1 - 40, -ENC_Y - 6, Z_HEADER + 16), COL["led"],
           tag="led")
    sc.box((x0, ENC_Y - 30, Z_HEADER), (ENC_X1, ENC_Y, Z_TOP), Wd, **T)
    for x in (x0, ENC_X1 - 30):
        sc.box((x, -ENC_Y, Z_HEADER), (x + 30, ENC_Y, Z_TOP), Wd, **T)
    sc.box((x0, -ENC_Y, Z_TOP - 10), (ENC_X1, ENC_Y, Z_TOP), Wd, tag="roof", spec=0.3)
    # 내부 상부 LED 바 (청색)
    sc.box((x0 + 60, -ENC_Y + 40, Z_HEADER - 30), (ENC_X1 - 60, -ENC_Y + 60, Z_HEADER - 22),
           COL["led"], tag="led")
    # 유리 도어 (전면 3베이, 각 2짝) + 손잡이, 측·후면 유리
    G = dict(alpha=0.12, spec=0.8, tag="cover", edges=True)
    for i in range(3):
        xa, xb = BAYS[i] + 40, BAYS[i + 1] - 40
        xm = (xa + xb) / 2
        for (u0, u1) in ((xa, xm - 3), (xm + 3, xb)):
            sc.box((u0, -ENC_Y + 8, Z_PLINTH + 30), (u1, -ENC_Y + 14, Z_HEADER - 40), COL["glass"], **G)
            sc.box((u0, -ENC_Y + 2, Z_PLINTH + 30), (u1, -ENC_Y + 8, Z_PLINTH + 60), COL["alu_d"],
                   tag="cover")
            sc.box((u0, -ENC_Y + 2, Z_HEADER - 70), (u1, -ENC_Y + 8, Z_HEADER - 40), COL["alu_d"],
                   tag="cover")
        for xh in (xm - 50, xm + 40):
            sc.box((xh, -ENC_Y - 30, 950), (xh + 12, -ENC_Y + 2, 1350), COL["alu"], tag="cover", spec=0.6)
        sc.box((xa, ENC_Y - 14, Z_PLINTH + 30), (xb, ENC_Y - 8, Z_HEADER - 40), COL["glass"], **G)
    for x in (x0 + 10, ENC_X1 - 16):
        sc.box((x, -ENC_Y + 40, Z_PLINTH + 30), (x + 6, ENC_Y - 40, Z_HEADER - 40), COL["glass"], **G)


def build_cabinet(sc: Scene):
    W = COL["white"]
    T = dict(tag="cab", spec=0.3)
    x0, x1 = CAB_X0, CAB_X1
    sc.box((x0, -ENC_Y, 80), (x1, ENC_Y, Z_TOP), W, **T)
    sc.box((x0 + 10, -ENC_Y - 8, Z_HEADER), (x1 - 10, -ENC_Y, Z_TOP - 10), COL["navy"], tag="cab")
    sc.box((x0 + 30, -ENC_Y - 4, 260), (x1 - 30, -ENC_Y, 1000), COL["white_d"], tag="cab")   # 하부 도어
    sc.box((x1 - 70, -ENC_Y - 20, 560), (x1 - 55, -ENC_Y - 4, 760), COL["chain"], tag="cab")
    # HMI
    sc.box((x0 + 70, -ENC_Y - 25, 1320), (x1 - 70, -ENC_Y, 1700), COL["chain"], tag="cab")
    sc.box((x0 + 85, -ENC_Y - 27, 1335), (x1 - 85, -ENC_Y - 25, 1685), COL["screen"], tag="led")
    # 버튼 + 비상정지
    zb = 1180.0
    for i, col in enumerate((COL["ok"], COL["ok"], COL["ng"])):
        x = x0 + 120 + i * 90
        sc.cyl((x, -ENC_Y, zb), (x, -ENC_Y - 25, zb), 22, col, n=20, tag="cab", spec=0.5)
    sc.cyl((x1 - 120, -ENC_Y, zb), (x1 - 120, -ENC_Y - 15, zb), 42, COL["yellow"], n=24, tag="cab")
    sc.cyl((x1 - 120, -ENC_Y - 15, zb), (x1 - 120, -ENC_Y - 45, zb), 32, COL["ng"], n=24, tag="cab",
           spec=0.5)
    sc.box((x0 + 150, -ENC_Y - 6, 1060), (x1 - 150, -ENC_Y, 1090), COL["chain"], tag="cab")  # 키 슬롯
    # 신호등
    xs, ys = (x0 + x1) / 2 - 150, -300.0
    sc.cyl((xs, ys, Z_TOP), (xs, ys, Z_TOP + 120), 14, COL["bolt"], n=12, tag="cab")
    z = Z_TOP + 120
    for col in (COL["ok"], COL["yellow"], COL["ng"]):
        sc.cyl((xs, ys, z), (xs, ys, z + 80), 40, col, n=24, tag="led")
        z += 80
    sc.cyl((xs, ys, z), (xs, ys, z + 20), 42, COL["white_d"], n=24, tag="cab")


def build_floor_patch(sc: Scene):
    m = 900.0
    sc.poly([(ENC_X0 - m, -ENC_Y - m, 0), (CAB_X1 + m, -ENC_Y - m, 0),
             (CAB_X1 + m, ENC_Y + m, 0), (ENC_X0 - m, ENC_Y + m, 0)], COL["floor"], edges=False,
            tag="floor")
    add_safety_line(sc, ENC_X0 - 500, CAB_X1 + 500, -ENC_Y - 500, ENC_Y + 500, tag="floor")


def add_safety_line(sc, x0, x1, y0, y1, w=80.0, tag="floor"):
    yl = COL["fyellow"]
    for (a, b, c, d) in (((x0, y0), (x1, y0), (x1, y0 + w), (x0, y0 + w)),
                         ((x0, y1 - w), (x1, y1 - w), (x1, y1), (x0, y1)),
                         ((x0, y0), (x0 + w, y0), (x0 + w, y1), (x0, y1)),
                         ((x1 - w, y0), (x1, y0), (x1, y1), (x1 - w, y1))):
        sc.poly([(*a, 3), (*b, 3), (*c, 3), (*d, 3)], yl, edges=False, tag=tag)


def build_person(sc: Scene, x, y):
    c = COL["person"]
    T = dict(tag="fac")
    for dy in (-90, 90):
        sc.cyl((x, y + dy, 0), (x, y + dy, 850), 60, (0.18, 0.20, 0.25), n=12, **T)
    sc.box((x - 110, y - 190, 850), (x + 110, y + 190, 1450), c, **T)
    for dy in (-230, 230):
        sc.cyl((x, y + dy, 1420), (x + 60, y + dy, 900), 45, c, n=10, **T)
    sc.cyl((x, y, 1450), (x, y, 1510), 50, COL["skin"], n=12, **T)
    sc.cyl((x, y, 1510), (x, y, 1700), 95, COL["skin"], n=16, **T)
    sc.cyl((x, y, 1690), (x, y, 1740), 105, (0.05, 0.05, 0.05), n=16, **T)


def build_factory(sc: Scene):
    """클린 생산동 가상 환경 (밝은 에폭시 바닥, 벽, 기둥, 인접 설비)"""
    FX0, FX1, FY0, FY1 = -9000.0, 13000.0, -8000.0, 9000.0
    for i, x in enumerate(np.arange(FX0, FX1, 2000.0)):
        for j, y in enumerate(np.arange(FY0, FY1, 2000.0)):
            sc.poly([(x, y, 0), (x + 2000, y, 0), (x + 2000, y + 2000, 0), (x, y + 2000, 0)],
                    COL["floor"] if (i + j) % 2 == 0 else COL["floor2"], edges=False, tag="fac")
    for yy in (-3600.0, -2400.0):
        sc.poly([(FX0, yy, 3), (FX1, yy, 3), (FX1, yy + 100, 3), (FX0, yy + 100, 3)], COL["fyellow"],
                edges=False, tag="fac")
    sc.poly([(FX0, -3500, 2), (FX1, -3500, 2), (FX1, -2400, 2), (FX0, -2400, 2)], COL["fgreen"],
            alpha=0.5, edges=False, tag="fac")
    add_safety_line(sc, ENC_X0 - 500, CAB_X1 + 500, -ENC_Y - 500, ENC_Y + 500, tag="fac")
    sc.box((FX0, FY1, 0), (FX1, FY1 + 200, 6000), COL["wall"], tag="fac", spec=0.05)
    sc.box((FX0 - 200, FY0, 0), (FX0, FY1 + 200, 6000), COL["wall"], tag="fac", spec=0.05)
    sc.box((FX0, FY1 - 5, 3500), (FX1, FY1, 4300), (0.60, 0.74, 0.86), tag="fac")
    for cx in (-5000.0, 2000.0, 9000.0):
        sc.box((cx - 200, FY1 - 400, 0), (cx + 200, FY1, 6000), COL["column"], tag="fac")
    for mx in (4200.0, 7600.0):
        sc.box((mx, -800, 80), (mx + 2600, 800, 2100), COL["machine"], tag="fac")
        sc.box((mx + 200, -803, 700), (mx + 2400, -800, 1800), (0.60, 0.74, 0.86), tag="fac")
        sc.box((mx, -800, 0), (mx + 2600, 800, 80), COL["machine_d"], tag="fac")
        sc.box((mx, -806, 1900), (mx + 2600, -800, 2100), COL["navy"], tag="fac")
    sc.box((-6500, 4500, 850), (-4000, 5300, 900), (0.50, 0.52, 0.55), tag="fac")
    for (ax_, ay_) in ((-6450, 4550), (-4050, 4550), (-6450, 5250), (-4050, 5250)):
        sc.box((ax_ - 25, ay_ - 25, 0), (ax_ + 25, ay_ + 25, 850), (0.40, 0.42, 0.45), tag="fac")
    build_person(sc, -3300, 300)


def add_header_text(pl):
    """헤더 전면 문자 (3D 문자 — ASCII만 지원)"""
    items = [("", 90, (ENC_X0 + 420, -ENC_Y - 8, 2110)),
             ("BSV Slit Valve", 95, (-650, -ENC_Y - 8, 2125)),
             ("O-ring Automatic Visual Inspection System", 42, (-650, -ENC_Y - 8, 2010)),
             ("O-ring Precision Inspection", 55, (900, -ENC_Y - 8, 2060)),
             ("INPUT RACK (10)", 48, (XI, RACK_Y - 104, tier_z(TIERS - 1) + 220)),
             ("OUTPUT RACK (OK/NG 10)", 40, (XO, RACK_Y - 104, tier_z(TIERS - 1) + 220)),
             ("POMEAS", 30, (CX, CY - 58, 1700))]
    actors = []
    for s, h, c in items:
        try:
            m = pv.Text3D(s, depth=3, height=h, center=c, normal=(0, -1, 0))
            actors.append(pl.add_mesh(m, color="white", ambient=0.6, specular=0.2))
        except Exception:
            pass
    return actors


# ===========================================================================
# 로봇 기구학 (간이)
# ===========================================================================
def fk(xc, q1, a1, a2):
    f = np.array([math.cos(q1), math.sin(q1), 0.0])
    l = np.array([-math.sin(q1), math.cos(q1), 0.0])
    z = np.array([0.0, 0.0, 1.0])
    S0 = np.array([xc, RY, Z_SH])
    S = S0 + LINK_OFF * l
    E = S + L1 * (math.cos(a1) * f + math.sin(a1) * z)
    W = E + L2 * (math.cos(a2) * f + math.sin(a2) * z)
    return S0, S, E, W, f, l


def ik(xc, W, elbow="up"):
    d = np.array([W[0] - xc, W[1] - RY])
    rho = float(np.hypot(*d))
    if rho <= LINK_OFF + 1:
        raise ValueError(f"손목이 어깨축에 너무 가까움 W={np.round(W)}, xc={xc}")
    q1 = math.atan2(d[1], d[0]) - math.asin(LINK_OFF / rho)
    f = np.array([math.cos(q1), math.sin(q1)])
    r = float(d @ f)
    h = W[2] - Z_SH
    D = math.hypot(r, h)
    if D > L1 + L2 - 1:
        raise ValueError(f"도달 불가 W={np.round(W)}, xc={xc}, D={D:.0f}")
    phi = math.atan2(h, r)
    al = math.acos(max(-1.0, min(1.0, (L1 * L1 + D * D - L2 * L2) / (2 * L1 * D))))
    a1 = phi - al if elbow == "down" else phi + al
    er, eh = L1 * math.cos(a1), L1 * math.sin(a1)
    return np.array([q1, a1, math.atan2(h - eh, r - er)])


def flange_for_product(center):
    """제품 중심(파지 상태) → 툴 플랜지 위치"""
    t, _, c = R_TOOL
    return np.asarray(center, float) - t * T_PROD - c * C_PROD


def wrist_from_flange(Fl):
    return np.asarray(Fl, float) - R_TOOL[0] * W2F


# ===========================================================================
# 사이클 프로그램
# ===========================================================================
@dataclass
class KF:
    xc: float
    q: np.ndarray
    grip: float
    dur: float
    hold: float
    label: str
    event: tuple | None


def oring_points(n=24, inset=12.0):
    hx, hy = PROD_L / 2 - inset, PROD_W / 2 - inset
    per = 4 * hx + 4 * hy
    pts = []
    for k in range(n):
        s = per * k / n
        if s < 2 * hx:
            pts.append((-hx + s, -hy))
        elif s < 2 * hx + 2 * hy:
            pts.append((hx, -hy + (s - 2 * hx)))
        elif s < 4 * hx + 2 * hy:
            pts.append((hx - (s - 2 * hx - 2 * hy), hy))
        else:
            pts.append((-hx, hy - (s - 4 * hx - 2 * hy)))
    return pts


class Program:
    def __init__(self, ko=True):
        self.ko = ko
        self.kfs: list[KF] = []
        self.out_plan: list[tuple[str, int]] = []
        self._build()
        self.t_start, acc = [], 0.0
        for k in self.kfs:
            self.t_start.append(acc)
            acc += k.dur + k.hold
        self.total = acc

    def L(self, ko, en):
        return ko if self.ko else en

    def add_fl(self, Fl, xc, dur=1.0, hold=0.0, grip=None, label="", event=None, elbow="up"):
        W = wrist_from_flange(Fl)
        xc = float(min(CARR_MAX, max(CARR_MIN, xc)))
        q = ik(xc, W, elbow)
        if grip is None:
            grip = self.kfs[-1].grip if self.kfs else 0.0
        self.kfs.append(KF(xc, q, grip, dur, hold, label, event))

    def _build(self):
        t, _, c = R_TOOL
        n_ok = 0
        n_ng = 0
        pts = oring_points()
        xi_c = XI + 300.0          # 입력 랙 작업 시 캐리지 위치
        xo_c = XO - 300.0          # 출력 랙 작업 시 캐리지 위치
        xj_c = CX + 300.0          # 지그·검사 시 캐리지 위치
        home_fl = np.array([XI, -250.0, 1150.0])
        self.add_fl(home_fl, xi_c, dur=0.1, hold=0.8, label=self.L("대기", "Home"))
        for k in range(TIERS):                        # 입력 랙 위에서부터 순서대로
            tier = TIERS - 1 - k
            zb = tier_z(tier)
            is_ng = k in (2, 6)
            P = lambda y, z_bottom: np.array([XI, y, z_bottom + PROD_T / 2])  # noqa: E731
            # 1) 진입 → SN 판독
            out_y = -170.0                            # 포크 끝이 랙 전면 밖에 있는 제품중심 Y
            self.add_fl(flange_for_product(P(out_y, zb - GAP)), xi_c, dur=2.2,
                        label=self.L(f"제품 {k + 1}: 입력 랙 {tier + 1}단 앞으로 이동",
                                     f"Part {k + 1}: to input tier {tier + 1}"))
            self.add_fl(flange_for_product(P(RACK_Y, zb - GAP)), xi_c, dur=1.0,
                        label=self.L("② 하부 진입 (포크 수평 삽입)", "Fork in under part"))
            self.add_fl(flange_for_product(P(RACK_Y, zb - GAP)), xi_c, dur=0.1, hold=0.9,
                        label=self.L("SN 판독 (포크 내장 카메라, 제품 하면)", "Read serial number"),
                        event=("sn", tier))
            # 2) 상향 리프트 → 파지 → 인출
            self.add_fl(flange_for_product(P(RACK_Y, zb)), xi_c, dur=0.5,
                        label=self.L("③ 상향 리프트", "Lift"))
            self.add_fl(flange_for_product(P(RACK_Y, zb)), xi_c, dur=0.35, grip=1.0,
                        label=self.L("③ 파지", "Grip"), event=("pick_in", tier))
            self.add_fl(flange_for_product(P(RACK_Y, zb + 18)), xi_c, dur=0.4,
                        label=self.L("받침 립 넘김", "Clear lip"))
            self.add_fl(flange_for_product(P(out_y, zb + 18)), xi_c, dur=0.9,
                        label=self.L("④ 인출", "Withdraw"))
            # 3) 정렬 지그 안착 → 보정 → 재파지
            self.add_fl(flange_for_product((XI, -170.0, 1330.0)), xi_c, dur=1.0,
                        label=self.L("④ 검사부로 이동", "To inspection"))
            self.add_fl(flange_for_product((CX, CY, Z_JIG + PROD_T / 2 + 90)), xj_c, dur=2.6,
                        label=self.L("④ 검사부로 이동", "To inspection"))
            self.add_fl(flange_for_product((CX, CY, Z_JIG + PROD_T / 2)), xj_c, dur=0.6,
                        label=self.L("정렬 지그 안착", "Place on jig"))
            self.add_fl(flange_for_product((CX, CY, Z_JIG + PROD_T / 2)), xj_c, dur=0.3, grip=0.0,
                        label=self.L("정렬 지그 안착", "Release"), event=("to_jig", None))
            self.add_fl(flange_for_product((CX, CY, Z_JIG + PROD_T / 2 - GAP)), xj_c, dur=0.4,
                        hold=0.6, label=self.L("지그 위치 보정 (핀 정렬)", "Jig alignment"))
            self.add_fl(flange_for_product((CX, CY, Z_JIG + PROD_T / 2)), xj_c, dur=0.4,
                        label=self.L("재파지", "Re-grip"))
            self.add_fl(flange_for_product((CX, CY, Z_JIG + PROD_T / 2)), xj_c, dur=0.3, grip=1.0,
                        label=self.L("재파지", "Re-grip"), event=("from_jig", None))
            # 4) 정밀검사 (제품 상면을 초점면에 두고 O-ring 윤곽 순차 이동)
            zc = Z_FOCUS - PROD_T / 2
            for j, (qx, qy) in enumerate(pts):
                self.add_fl(flange_for_product((CX - qx, CY - qy, zc)), xj_c,
                            dur=0.8 if j == 0 else 0.16, hold=0.10,
                            label=self.L(f"정밀검사 {j + 1}/{len(pts)} (POMEAS 고정 카메라, WD {WD:.0f})",
                                         f"Inspection {j + 1}/{len(pts)}"),
                            event=("insp", j))
            self.add_fl(flange_for_product((CX, -60.0, 1250.0)), xj_c, dur=0.7,
                        label=self.L("검사 완료 — 판정", "Judge"))
            # 5) 출력 랙 적재 (하부에서 밀어 넣기)
            if is_ng:
                otier, lab = NG_TIERS[n_ng], self.L("불량(NG) 단 적재", "Place NG")
                n_ng += 1
            else:
                otier, lab = n_ok, self.L("양품(OK) 단 적재", "Place OK")
                n_ok += 1
            self.out_plan.append(("NG" if is_ng else "OK", otier))
            zo = tier_z(otier)
            Q = lambda y, z_bottom: np.array([XO, y, z_bottom + PROD_T / 2])  # noqa: E731
            self.add_fl(flange_for_product(Q(out_y, zo + 18)), xo_c, dur=2.6,
                        label=self.L(f"{lab} — 출력 랙 {otier + 1}단으로 이동", lab))
            self.add_fl(flange_for_product(Q(RACK_Y, zo + 18)), xo_c, dur=0.9,
                        label=self.L(f"{lab} — 삽입", lab))
            self.add_fl(flange_for_product(Q(RACK_Y, zo)), xo_c, dur=0.4, label=lab)
            self.add_fl(flange_for_product(Q(RACK_Y, zo)), xo_c, dur=0.3, grip=0.0, label=lab,
                        event=("place_out", (otier, "NG" if is_ng else "OK")))
            self.add_fl(flange_for_product(Q(RACK_Y, zo - GAP)), xo_c, dur=0.4,
                        label=self.L("하강", "Lower"))
            self.add_fl(flange_for_product(Q(out_y, zo - GAP)), xo_c, dur=0.8,
                        label=self.L("포크 후퇴", "Withdraw fork"))
        self.add_fl(home_fl, xi_c, dur=3.0, hold=2.0,
                    label=self.L("10개 완료 — 처음부터 반복", "All done — repeat"))

    def sample(self, t):
        t = t % self.total
        i = max(0, int(np.searchsorted(self.t_start, t, side="right")) - 1)
        k = self.kfs[i]
        p = self.kfs[i - 1] if i > 0 else k
        tl = t - self.t_start[i]
        if tl >= k.dur or i == 0:
            u, done = 1.0, i
        else:
            u = tl / k.dur
            u = u * u * (3 - 2 * u)
            done = i - 1
        dq = k.q - p.q
        dq[0] = (dq[0] + math.pi) % (2 * math.pi) - math.pi
        return (p.xc + (k.xc - p.xc) * u, p.q + dq * u, p.grip + (k.grip - p.grip) * u,
                k.label, done)


# ===========================================================================
# 동적 형상
# ===========================================================================
def draw_product(sc, center, tag="dyn"):
    """제품: 길이 X, 폭 Y, 상면 +Z (O-ring 상면)"""
    c = np.asarray(center, float)
    sc.box(c - (PROD_L / 2, PROD_W / 2, PROD_T / 2), c + (PROD_L / 2, PROD_W / 2, PROD_T / 2),
           COL["product"], spec=0.45, tag=tag)
    zt = c[2] + PROD_T / 2
    i = 12.0
    hx, hy = PROD_L / 2 - i, PROD_W / 2 - i
    for s in (-1, 1):
        sc.box((c[0] - hx, c[1] + s * hy - 2, zt), (c[0] + hx, c[1] + s * hy + 2, zt + 1.5),
               COL["oring"], tag=tag)
        sc.box((c[0] + s * hx - 2, c[1] - hy, zt), (c[0] + s * hx + 2, c[1] + hy, zt + 1.5),
               COL["oring"], tag=tag)


def draw_carriage(sc, xc):
    for dx in (-90, 90):
        for s in (-1, 1):
            sc.box((xc + dx - 45, RY + s * 75 - 28, Z_GUIDE[1] - 5),
                   (xc + dx + 45, RY + s * 75 + 28, Z_CARR[0] + 18), COL["rail"], tag="dyn")
    sc.box((xc - 150, RY - 150, Z_CARR[0] + 18), (xc + 150, RY + 150, Z_CARR[1]), COL["alu_d"],
           spec=0.3, tag="dyn")
    sc.box((xc + 20, RY - 260, 330), (xc + 130, RY - 150, Z_CARR[1]), COL["rail"], tag="dyn")
    sc.box((xc - 85, RY - 85, Z_CARR[1]), (xc + 85, RY + 85, Z_BASE), COL["alu_d"], tag="dyn")


def draw_chain(sc, xc):
    x_f, Lc, link = -300.0, 1500.0, 34.0
    x_loop = (x_f + xc + Lc) / 2
    yl, yr = RY + 140, RY + 216
    zl, zu, rr = (292.0, 330.0), (350.0, 388.0), 48.0

    def lb(x, z0, z1):
        sc.box((x, yl, z0), (x + link - 4, yr, z1), COL["chain"], tag="dyn")

    x = x_f
    while x < x_loop - link:
        lb(x, *zl)
        x += link
    x = xc
    while x < x_loop - link:
        lb(x, *zu)
        x += link
    cz = (zl[0] + zu[1]) / 2
    for i in range(7):
        a0 = -math.pi / 2 + math.pi * i / 7
        a1 = -math.pi / 2 + math.pi * (i + 1) / 7
        p0 = np.array([x_loop - link + rr * math.cos(a0), (yl + yr) / 2, cz + rr * math.sin(a0)])
        p1 = np.array([x_loop - link + rr * math.cos(a1), (yl + yr) / 2, cz + rr * math.sin(a1)])
        sc.bar(p0, p1, yr - yl, 36, COL["chain"], up=(0, 1, 0), tag="dyn")
    sc.box((xc - 20, RY + 100, zu[1]), (xc + 20, RY + 220, Z_CARR[1]), COL["alu_d"], tag="dyn")


def draw_robot(sc, xc, q, grip, carry, flash=False):
    wh, jk = COL["robot"], COL["robot_j"]
    T = dict(tag="dyn", spec=0.25)
    S0, S, E, W, f, l = fk(xc, *q)
    sc.cyl((xc, RY, Z_BASE), (xc, RY, Z_BASE + 20), 80, jk, n=28, **T)
    sc.cyl((xc, RY, Z_BASE + 20), (xc, RY, Z_SH), 66, wh, n=28, **T)
    sc.cyl(S0, S + l * 55, 60, wh, n=28, **T)
    sc.cyl(S + l * 55, S + l * 62, 48, COL["robot_b"], n=28, **T)
    sc.cyl(S, E, 43, wh, n=24, **T)
    sc.cyl(E - l * 55, E + l * 55, 48, wh, n=24, **T)
    sc.cyl(E + l * 55, E + l * 61, 38, COL["robot_b"], n=24, **T)
    sc.cyl(E, W, 36, wh, n=24, **T)
    sc.cyl(W - l * 48, W + l * 48, 39, wh, n=24, **T)
    t, b, c = R_TOOL
    sc.cyl(W, W + t * 60, 37, wh, n=24, **T)
    sc.cyl(W + t * 60, W + t * 110, 35, wh, n=24, **T)
    sc.cyl(W + t * 110, W + t * W2F, 31, COL["robot_b"], n=24, **T)
    Fl = W + t * W2F
    R = np.array([t, b, c])
    # 그리퍼 본체 (청색) + 포크 판 + 조(jaw) 2개
    sc.obox(Fl + t * 25 + c * 5, R, (25, 45, 40), COL["grip"], tag="dyn", spec=0.35)
    sc.obox(Fl + t * 130 + c * 25, R, (80, 42, 10), COL["grip_j"], tag="dyn", spec=0.4)
    off = PROD_W / 2 + 6 + 18 * (1 - grip)
    for s in (-1, 1):
        sc.obox(Fl + t * (T_PROD + s * off) + c * 47, R, (6, 32, 12), COL["grip"], tag="dyn", spec=0.3)
    lens = Fl + t * T_PROD + c * 35
    sc.cyl(lens - c * 3, lens + c * 1, 11, COL["cam"], n=20, tag="dyn")
    sc.cyl(lens + c * 1, lens + c * 2, 17, (1.0, 1.0, 0.6) if flash else COL["ringlight"], n=24,
           tag="led" if flash else "dyn")
    if carry:
        draw_product(sc, Fl + t * T_PROD + c * C_PROD)
    return Fl


# ===========================================================================
# 시뮬레이터
# ===========================================================================
class CellSim:
    def __init__(self, pl, factory=True, font=None):
        self.pl = pl
        self.font = font
        self.ko = font is not None
        self.prog = Program(ko=self.ko)
        self.t = 0.0
        self.speed = 1.0
        self.playing = True
        self.last = time.perf_counter()
        self.dyn_actors: dict = {}
        self.cover_actors, self.factory_actors, self.patch_actors = [], [], []
        self.show_cover, self.show_factory = True, factory
        self._reset_state()
        self._build_static(factory)
        self._add_overlay()
        self.update(force=True)

    def _reset_state(self):
        self.inp = set(range(TIERS))
        self.out: dict[int, str] = {}
        self.carry = False
        self.on_jig = False
        self.done_idx = -1
        self.flash_until = -1.0
        self.n_ok = self.n_ng = 0

    def _apply_events(self, upto, t):
        for i in range(self.done_idx + 1, upto + 1):
            ev = self.prog.kfs[i].event
            if not ev:
                continue
            kind, arg = ev
            if kind == "pick_in":
                self.inp.discard(arg)
                self.carry = True
            elif kind == "to_jig":
                self.carry, self.on_jig = False, True
            elif kind == "from_jig":
                self.carry, self.on_jig = True, False
            elif kind == "place_out":
                self.carry = False
                self.out[arg[0]] = arg[1]
                if arg[1] == "OK":
                    self.n_ok += 1
                else:
                    self.n_ng += 1
            elif kind == "sn":
                self.flash_until = t + 0.9
        self.done_idx = upto

    def _build_static(self, factory):
        sc = Scene()
        build_steel_base(sc)
        build_rail_static(sc)
        build_rack(sc, XI, lambda k: None, "in")
        build_rack(sc, XO, lambda k: COL["ng"] if k in NG_TIERS else COL["ok"], "out")
        build_portal_camera(sc)
        build_jig(sc)
        build_enclosure(sc)
        build_cabinet(sc)
        for key, m in scene_to_groups(sc, excl=("cover",)).items():
            add_group(self.pl, key, m)
        for key, m in scene_to_groups(sc, tags={"cover"}).items():
            self.cover_actors.append(add_group(self.pl, key, m))
        add_header_text(self.pl)
        sf = Scene()
        build_factory(sf)
        for key, m in scene_to_groups(sf).items():
            a = add_group(self.pl, key, m)
            a.SetVisibility(factory)
            self.factory_actors.append(a)
        sp = Scene()
        build_floor_patch(sp)
        for key, m in scene_to_groups(sp).items():
            a = add_group(self.pl, key, m)
            a.SetVisibility(not factory)
            self.patch_actors.append(a)

    def _add_overlay(self):
        kw = dict(font_size=11, color="#1d2733")
        if self.font:
            kw["font_file"] = self.font
        self.txt = self.pl.add_text(" ", position=(0.01, 0.86), viewport=True, **kw)
        help_ko = ("Space 재생/정지  ·  ↑↓ 속도  ·  k 도어·커버  ·  n 공장  ·  마우스 드래그 회전·휠 확대\n"
                   "간이 기구학 시각화 — 실제 CS66 역기구학·충돌검사 아님  ·  치수는 개념값")
        help_en = ("Space play/pause  -  Up/Down speed  -  k doors  -  n factory  -  drag rotate\n"
                   "Simplified kinematics for visualization only")
        self.pl.add_text(help_ko if self.ko else help_en, position=(0.30, 0.01), viewport=True,
                         **dict(kw, font_size=9))
        try:
            self.pl.add_checkbox_button_widget(lambda v: self.set_play(v), value=True,
                                               position=(10, 10), size=34,
                                               color_on="#2e8b57", color_off="#b03030")
            self.pl.add_checkbox_button_widget(lambda v: self.set_cover(v), value=True,
                                               position=(10, 54), size=34,
                                               color_on="#6fa8dc", color_off="#888888")
            self.pl.add_text("재생" if self.ko else "Play", position=(52, 18), **kw)
            self.pl.add_text("도어" if self.ko else "Doors", position=(52, 62), **kw)
        except Exception:
            pass

    def _set_text(self, s):
        s = s.replace("|", "/")  # VTK: '|'가 있으면 지정 글꼴 대신 기본 글꼴로 그려져 한글이 깨짐
        if hasattr(self.txt, "SetInput"):
            self.txt.SetInput(s)
        elif hasattr(self.txt, "SetText"):
            self.txt.SetText(2, s)

    def set_play(self, v):
        self.playing = bool(v)
        self.last = time.perf_counter()

    def toggle_play(self):
        self.set_play(not self.playing)

    def set_cover(self, v):
        self.show_cover = bool(v)
        for a in self.cover_actors:
            a.SetVisibility(self.show_cover)
        self.pl.render()

    def toggle_factory(self):
        self.show_factory = not self.show_factory
        for a in self.factory_actors:
            a.SetVisibility(self.show_factory)
        for a in self.patch_actors:
            a.SetVisibility(not self.show_factory)
        self.pl.render()

    def faster(self):
        self.speed = min(16.0, self.speed * 2)

    def slower(self):
        self.speed = max(0.125, self.speed / 2)

    def set_time(self, t):
        self._reset_state()
        self.t = t
        self.update(force=True)

    def update(self, force=False):
        now = time.perf_counter()
        dt = min(0.5, now - self.last)
        self.last = now
        T = self.prog.total
        prev = self.t % T
        if self.playing and not force:
            self.t += dt * self.speed
        tl = self.t % T
        if tl < prev and not force:          # 한 사이클 종료 → 랙 상태 초기화
            self._reset_state()
        xc, q, grip, label, done = self.prog.sample(tl)
        self._apply_events(done, tl)
        sc = Scene()
        draw_carriage(sc, xc)
        draw_chain(sc, xc)
        draw_robot(sc, xc, q, grip, self.carry, flash=tl < self.flash_until)
        for k in self.inp:
            draw_product(sc, (XI, RACK_Y, tier_z(k) + PROD_T / 2))
        for k in self.out:
            draw_product(sc, (XO, RACK_Y, tier_z(k) + PROD_T / 2))
        if self.on_jig:
            draw_product(sc, (CX, CY, Z_JIG + PROD_T / 2))
        groups = scene_to_groups(sc)
        for key, a in self.dyn_actors.items():
            if key not in groups:
                a.SetVisibility(False)
        for key, m in groups.items():
            if key in self.dyn_actors:
                a = self.dyn_actors[key]
                a.mapper.dataset.copy_from(m)
                a.mapper.dataset.Modified()
                a.SetVisibility(True)
            else:
                self.dyn_actors[key] = add_group(self.pl, key, m)
        if self.ko:
            s = (f"BSV Slit Valve O-ring 정밀 검사 설비 — 실시간 시뮬레이션 V4\n"
                 f"단계: {label}\n"
                 f"캐리지 X = {xc:7.0f} mm   ·   속도 ×{self.speed:g}   ·   "
                 f"{'재생' if self.playing else '일시정지'}\n"
                 f"입력 랙 {len(self.inp)}  /  양품(OK) {self.n_ok}  /  불량(NG) {self.n_ng}"
                 f"   ·   사이클 {tl:5.1f} / {T:.0f} s")
        else:
            s = (f"BSV O-ring inspection system - real-time simulation V4\n"
                 f"Step: {label}\nCarriage X = {xc:7.0f} mm - speed x{self.speed:g}\n"
                 f"Input {len(self.inp)} / OK {self.n_ok} / NG {self.n_ng} - {tl:5.1f} / {T:.0f} s")
        self._set_text(s)

    def tick(self, *_):
        self.update()
        self.pl.render()


def default_camera(pl):
    pl.camera_position = [(-1500.0, -7600.0, 2900.0), (-150.0, 0.0, 1050.0), (0.0, 0.0, 1.0)]
    pl.camera.view_angle = 30.0


def make_plotter(off_screen=False, size=(1600, 900)):
    pl = pv.Plotter(off_screen=off_screen, window_size=size, lighting="light_kit",
                    title="BSV Slit Valve O-ring Inspection - Simulation V4")
    pl.set_background("#d9dee4", top="#f6f8fa")
    try:
        pl.enable_depth_peeling(number_of_peels=4)
    except Exception:
        pass
    return pl


def run_interactive(factory=True, use_vtk_timer=False):
    pl = make_plotter()
    try:
        pl.enable_anti_aliasing("ssaa")
    except Exception:
        pass
    sim = CellSim(pl, factory=factory, font=find_font())
    default_camera(pl)
    pl.add_key_event("space", sim.toggle_play)
    pl.add_key_event("Up", sim.faster)
    pl.add_key_event("Down", sim.slower)
    pl.add_key_event("k", lambda: sim.set_cover(not sim.show_cover))
    pl.add_key_event("n", sim.toggle_factory)
    print(f"사이클 {sim.prog.total:.0f} s (제품 10개), 키프레임 {len(sim.prog.kfs)}개 — 창을 닫거나 q 키로 종료")

    if use_vtk_timer:   # 옵션: VTK 타이머 방식 (일부 Windows 환경에서 타이머가 동작하지 않음)
        pl.add_timer_event(max_steps=10 ** 9, duration=33, callback=sim.tick)
        pl.show()
        return

    # 기본: 직접 루프 방식 — 매 프레임 갱신 후 창 이벤트(마우스·키) 처리. Windows/Linux 공통 동작.
    closed = {"v": False}

    def _on_exit(*_):
        closed["v"] = True

    pl.show(interactive_update=True, auto_close=False)
    try:
        pl.iren.add_observer("ExitEvent", _on_exit)
    except Exception:
        pass
    frame = 1.0 / 60.0
    while not closed["v"]:
        t0 = time.perf_counter()
        try:
            if pl.render_window is None or getattr(pl, "_closed", False):
                break
            sim.update()
            pl.render()
            pl.iren.process_events()
        except Exception:
            break
        rest = frame - (time.perf_counter() - t0)
        if rest > 0:
            time.sleep(rest)
    try:
        pl.close()
    except Exception:
        pass


def run_snapshots(out_dir: Path, factory=False):
    out_dir.mkdir(parents=True, exist_ok=True)
    pl = make_plotter(off_screen=True, size=(1920, 1080))
    sim = CellSim(pl, factory=factory, font=find_font())
    sim.playing = False
    prog = sim.prog
    picks, seen = [], set()
    for i, k in enumerate(prog.kfs):
        key = k.label.split(" —")[0].split(":")[0]
        if k.event and k.event[0] == "insp" and k.event[1] not in (0, 12):
            continue
        if key in seen and not (k.event and k.event[0] == "insp"):
            continue
        seen.add(key)
        picks.append((prog.t_start[i] + k.dur * 0.999, k.label))
        if len(picks) >= 24:
            break
    for j, (t, lab) in enumerate(picks):
        sim.set_time(t)
        default_camera(pl)
        pl.render()
        pl.screenshot(str(out_dir / f"snap_{j:02d}.png"))
        print(f"  snap {j:02d}  t={t:6.1f}s  {lab}")
    pl.close()


def main():
    ap = argparse.ArgumentParser(description="BSV O-ring 정밀 검사 설비 실시간 시뮬레이터 V4")
    ap.add_argument("--no-factory", action="store_true", help="공장 환경 숨김")
    ap.add_argument("--snap", type=str, default=None, help="창 없이 스크린샷 저장 폴더")
    ap.add_argument("--vtk-timer", action="store_true", help="VTK 타이머 방식으로 재생 (기본: 직접 루프)")
    a = ap.parse_args()
    if a.snap:
        run_snapshots(Path(a.snap), factory=not a.no_factory)
    else:
        run_interactive(factory=not a.no_factory, use_vtk_timer=a.vtk_timer)


if __name__ == "__main__":
    main()
