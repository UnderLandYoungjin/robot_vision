#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BSV Slit Valve O-ring 정밀 검사 설비 — 통합 3D 시뮬레이터 V7 (오픈형 철망 펜스 + 양개 전동 자동문)
=================================================================
V4 셀 시뮬레이터(설비 형상·사이클)  +  V1 검사 경로 시뮬레이터(6축 역기구학·45° 윤곽 추종) 통합

설비 (V4)
  - 양 끝 10단 입·출력 랙, 전면 하부 제7축(주행축) 위 6축 협동로봇, 포크형 엔드이펙터
  - 중앙 상부 POMEAS 고정 카메라(독립 포털) + 정렬 지그, 우측 전장함, 공장 환경
검사 경로 (V1)
  - 제품을 45° 기울여 O-ring 외측 → 내측 윤곽을 1.5 mm 간격으로 연속 추종
  - 검사점 Pᵢ가 항상 카메라 촬영점 Q(초점 위치)에 오도록 제품 자세 계산
        Rᵢ   = Rx(−45°)·Rz(ψᵢ)                 보는 측면이 로봇 쪽(−Y)을 향하도록 ψᵢ 결정
        Tᵢ   = Q − Rᵢ·Pᵢ                         제품 원점 위치
        Tᴮ_F = (Tᵂ_B)⁻¹ · Tᵂ_P · (Tᶠ_P)⁻¹        로봇 플랜지 지령값 (Tᵂ_B: 제7축 위치 반영 로봇 베이스)
  - 6축 DH(Denavit–Hartenberg, 링크 파라미터 표기법) 정·역기구학, J6 범위 중앙 정렬용 그리퍼 장착각 γ 자동 계산
  - 오른쪽 화면 = 고정 카메라 시점 (검사점이 화면 중앙 십자선에 오는지 확인)

V4 대비 변경 (6축 기구학으로 풀었을 때 드러난 간섭·특이점 해소 포함)
  - 간이 2링크 기구학 → 6축 DH 역기구학 (랙 입·출력, 지그, 검사 모든 동작), 전 사이클 한 자세군
  - 정밀검사: 평면 24점 → 45° 기울임 외측·내측 윤곽 연속 추종 (약 1,000점)
  - 기울기 자동 축소: 내측 윤곽 양 끝을 볼 때 제품 몸체가 링조명(Ø180, WD 87)에 닿음 → 해당 구간만 45° → 약 38°
  - 포크형 엔드이펙터: 플랜지 축 ∥ 제품 법선 (ψ 회전을 J6가 담당), 포크 길이 150 mm 오프셋
  - 정렬 지그 높이 1,250 → 900 mm (기울임 검사 시 제품·포크와 지그 간섭 회피)
  - 랙 1단 520 → 680 mm, 피치 105 → 88 mm (1단에서 손목·툴이 레일·캐리지·케이블체인과 간섭, 최상단 높이는 유지)
  - 출력 랙 X 1,150 → 900 mm (입력 랙과 같은 로봇 상대위치 → 반대 어깨 자세·특이점 통과 없음)
  - 장거리 이동: 전면 이송면으로 직선 후퇴 → 제7축 동시 직선 이동 → 목적지 직선 접근 (포털·랙 기둥 회피)
  - 검사 전 이송점에서 J6 360° 되감기 (검사 ψ 누적 약 530° → J6 ±360° 중앙 정렬)

V5 대비 변경 (V6, 검토 반영)
  - 관절 속도 제한: V5는 검사점 간격을 고정 시간(0.02 s)으로 두어 내측 코너에서 J6 약 550°/s, J1 약 290°/s가 나옴
    → 모든 샘플 시간 간격을 max(지령 시간, 관절 변화량 / JOINT_VMAX, 제7축 이동량 / RAIL_VMAX)로 늘림
    → 사이클 시간이 속도 한계를 반영한 값으로 바뀜 (JOINT_VMAX·RAIL_VMAX는 가정값 → 사양서 값으로 교체)
  - 자기간섭 검사 추가: 파지 제품·포크 vs 로봇 링크(베이스~J5) 최소 거리 (V5 evaluate에는 없었음)
  - 요약에 관절별 최대 속도 표시, h 키 = 화면 문자(HUD) 숨김 (V5에 함수만 있고 키 연결 없음)

V6 대비 변경 (V7) — 외함 → 오픈형 철망 펜스 + 对开电动地推门(duìkāi diàndòng dìtuīmén, 양개 전동 바닥주행 미닫이문)
  - 밀폐형 외함(흰 기둥·헤더·지붕·유리 슬라이딩 도어) 제거 → 설비 베이스(플린스)만 남기고 지붕 없는 오픈형
  - 설비 외곽 300 mm 바깥에 철망 안전펜스 (검정 프레임 + 철망 50 mm 피치, 높이 2,000) — 우측은 전장함이 펜스 역할
  - 전면 양개 자동문: 노란 구동 기둥 2개 + 상부 빔(간판), 한쪽 2장 텔레스코픽 철망 도어(바깥 도어 2배속)
      열림 → 도어가 고정 펜스 바깥쪽 레일 위로 겹쳐 수납, 닫힘 → 중앙에서 맞닿음 (선단 고무 안전엣지)
  - 사이클에 자동문 동작 포함: 시작 = 문 닫힘(인터록 확인) 후 로봇 기동, 10개 완료 → 대기 위치 복귀 후 문 열림(랙 교체)
  - 인터록 검증: 로봇·제7축이 움직이는 샘플에서 문 열림량 > 0 이면 요약에 경고 표시
  - 기둥 상부 상태등: 닫힘 = 초록(자동 운전), 동작 중 = 주황 점멸, 열림 = 노랑(로봇 정지)
  - 바닥 도어 레일·전면 황흑 경고 줄무늬, CSV에 door_open(0 닫힘 ~ 1 열림) 열 추가
  - 키 변경: k = 철망 펜스·도어 표시/숨김 (내부 보기), d = 자동문 열림 구간으로 이동

환경 : Python 3.12.3  /  pip install pyvista numpy scipy   (GIF 녹화 시 imageio 추가)

실행
  python bsv_robot_cell_sim_v7.py                 대화형 3D 시뮬레이션
  python bsv_robot_cell_sim_v7.py --no-factory    공장 환경 없이 설비만
  python bsv_robot_cell_sim_v7.py --csv path.csv  전체 사이클 경로·관절각 CSV 저장 후 실행
  python bsv_robot_cell_sim_v7.py --snap DIR      창 없이 주요 시점 스크린샷 저장 (검증용)
  python bsv_robot_cell_sim_v7.py --gif sim.gif   제품 1개 사이클 GIF 녹화
  python bsv_robot_cell_sim_v7.py --vtk-timer     VTK 타이머 방식 재생 (기본은 직접 루프)

조작
  Space 재생·일시정지   ← / → 0.25 s 이동(일시정지)   ↑ / ↓ 속도 ×2 · ÷2
  k 철망 펜스·도어 표시   d 자동문 열림 구간으로 이동   n 공장 환경   c 시점 초기화   h 화면 문자 표시   q 종료
  하단 슬라이더: 시간 직접 이동     왼쪽 아래 체크박스: 재생 / 펜스

주의 (개념 시뮬레이션)
  - DH 값은 6축 협동로봇 일반형 개념값 — ELITE CS66 실제 DH로 교체 필요 (DH 리스트)
  - 촬영점 Q, 작업거리, 포크 오프셋, O-ring 윤곽 치수, 랙 단 피치는 모두 예시·가정값
  - 간섭 검사는 렌즈 하부 원통·지그·제7축 주변 상자에 대한 단순 점 검사 (정밀 충돌 검사 아님)
"""
from __future__ import annotations

import argparse
import csv
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
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

D2R = math.pi / 180.0

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
    "done": (0.18, 0.62, 0.36),
    "todo": (0.56, 0.59, 0.65),
    "tcp": (0.42, 0.31, 0.63),
    "fence": (0.13, 0.13, 0.14),       # 철망 펜스·도어 프레임 (검정 알루미늄)
    "mesh": (0.58, 0.60, 0.62),        # 철망 선
    "rubber": (0.05, 0.05, 0.05),      # 도어 선단 안전엣지
    "amber": (1.00, 0.55, 0.05),
}


# ===========================================================================
# 수학 함수
# ===========================================================================
def hexc(rgb):
    return "#%02x%02x%02x" % tuple(int(round(c * 255)) for c in rgb)


def nrm(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def Rx(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def Rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def H(R=None, t=(0.0, 0.0, 0.0)):
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    M[:3, 3] = t
    return M


def inv_h(M):
    R, t = M[:3, :3], M[:3, 3]
    Mi = np.eye(4)
    Mi[:3, :3] = R.T
    Mi[:3, 3] = -R.T @ t
    return Mi


def smooth(u):
    return u * u * (3.0 - 2.0 * u)


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rounded_rect(a, b, r, ds):
    """반폭 a(X), b(Y), 코너 R r 의 둥근 사각 윤곽을 간격 ds로 샘플 (시계 방향) → (점, 외향 법선, 둘레)"""
    segs = [("L", (-(a - r), b), (a - r, b), (0, 1)), ("A", (a - r, b - r), 90, 0),
            ("L", (a, b - r), (a, -(b - r)), (1, 0)), ("A", (a - r, -(b - r)), 0, -90),
            ("L", (a - r, -b), (-(a - r), -b), (0, -1)), ("A", (-(a - r), -(b - r)), -90, -180),
            ("L", (-a, -(b - r)), (-a, b - r), (-1, 0)), ("A", (-(a - r), b - r), 180, 90)]
    lens = [math.hypot(s[2][0] - s[1][0], s[2][1] - s[1][1]) if s[0] == "L" else abs(s[3] - s[2]) * D2R * r
            for s in segs]
    total = sum(lens)
    n = max(8, int(round(total / ds)))
    pts, nr = [], []
    for k in range(n):
        s_ = k * total / n
        for sg, L in zip(segs, lens):
            if s_ <= L + 1e-9:
                u = s_ / L if L > 0 else 0.0
                if sg[0] == "L":
                    p = np.array(sg[1], float) + u * (np.array(sg[2], float) - np.array(sg[1], float))
                    nn = np.array(sg[3], float)
                else:
                    ang = (sg[2] + u * (sg[3] - sg[2])) * D2R
                    nn = np.array([math.cos(ang), math.sin(ang)])
                    p = np.array(sg[1], float) + r * nn
                pts.append(p)
                nr.append(nn)
                break
            s_ -= L
    return np.array(pts), np.array(nr), total


# ===========================================================================
# 간이 장면 그래프 (면 목록 → 재질별 PolyData)  — V4
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
                self.poly([base - eb - ec, base + eb - ec, base + eb + ec, base - eb + ec], color, **kw)

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
            self.poly([ra[k], ra[k + 1], rb[k + 1], rb[k]], color, **dict(kw, edges=side_edges))
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
# 설계 파라미터 (mm) — X: 설비 길이(좌→우), Y: 깊이(전면 −), Z: 높이   (V4 기준)
# ===========================================================================
ENC_X0, ENC_X1 = -2100.0, 1500.0       # 본체 (3개 베이 × 1,200)
CAB_X0, CAB_X1 = 1500.0, 2100.0        # 우측 전장함 → 전체 4,200
ENC_Y = 800.0                          # 깊이 1,600
Z_PLINTH = 250.0
Z_HEADER = 1950.0
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

PROD_L, PROD_W, PROD_T = 360.0, 60.0, 20.0
PROD_R = 8.0                           # 제품 외곽 코너 R
TIERS = 10
# 최하단 제품 하면 높이, 단 피치 (V4 520 / 105 → 680 / 88)
#   V4 값(520)은 6축 기구학으로 풀면 1단에서 손목·툴이 제7축 레일·캐리지·케이블체인과 간섭(최대 −92 mm, 모든 자세군)
#   → 1단을 160 mm 올림. 최상단(V4 1,465)은 도달 한계라 1,472로 유지 → 피치 88
#     (포크 진입 시 아래 단 제품과 33 mm, 립 넘김 후 위 단 받침과 28 mm)
TIER_Z0, TIER_P = 680.0, 88.0
RACK_Y = 50.0                          # 랙 제품 중심 Y
# 입력 랙 / 출력 랙 제품 중심 X (V4 출력 랙 1,150 → 900)
#   V4 배치는 출력 랙이 로봇 베이스 오른쪽(+300)에 와서 입력 랙·지그(왼쪽 −300/−150)와 반대 어깨 자세가 필요 →
#   이송 중 J1 축 위(특이점)를 지나며 자세 전환 → 제품·포크가 포털·랙 기둥을 휩쓺. 출력 랙을 250 mm 왼쪽으로 옮겨
#   캐리지 끝(1,200)에서 입력 랙과 같은 상대위치(−300) → 전 사이클 한 자세군, 이송은 이송면 직선 이동
XI, XO = -1775.0, 900.0
NG_TIERS = (8, 9)                      # 출력 랙 상부 2단 = 불량(NG)
NG_PARTS = (2, 6)                      # 시뮬레이션상 불량 판정 제품 순번(0부터)

CX, CY = -150.0, 150.0                 # 정밀 카메라 광축 위치
Z_LENS = 1450.0                        # 링조명 하면
WD = 87.0                              # 작동거리 (POMEAS LZG-63104-A 표기값 기준)
Z_FOCUS = Z_LENS - WD                  # 촬영점 높이
Q = np.array([CX, CY, Z_FOCUS])        # 카메라 촬영점 Q
Z_JIG = 900.0                          # 정렬 지그 제품 지지면 (V4 1,250 → 하향)

# ---- 검사 경로 (V1) ----
TILT_DEG = 45.0                        # 파지 기울기
OR_OUT = (170.0, 24.0, 20.0)           # O-ring 외측 윤곽: 반폭X, 반폭Y, 코너R
OR_IN = (166.0, 20.0, 16.0)            # O-ring 내측 윤곽 → 폭 4 mm
STEP_MM = 1.5                          # 검사점 간격 (시야 2 mm · 중첩 25% 가정)
TRANS_N = 60                           # 외측 → 내측 전환 프레임 수
DT_INSP = 0.02                         # 검사점 1개당 시간 [s] (시각화용)
DT_TRANS = 0.03
CAM_FOV = 20.0                         # 오른쪽 카메라 화면 시야 폭 (표시용)
LENS_R = 100.0                         # 렌즈·링조명 간섭 검사 반경 (링조명 Ø180 + 여유)
LENS_MARGIN = 5.0                      # 렌즈 하면과 최소 여유 → 부족 구간은 기울기 자동 축소
TILT_MIN = 20.0                        # 자동 축소 하한 [deg]
APPROACH_DZ = 40.0                     # 검사 시작·종료 시 초점면 아래 대기 거리

# ---- 포크형 엔드이펙터 (툴 좌표 = 플랜지 좌표를 Zf축으로 γ 회전, 축 방향은 제품 좌표와 동일) ----
L_FORK = 150.0                         # 플랜지 축 → 제품 중심 (포크 진입 방향 +Yp)
H_TOOL = 75.0                          # 플랜지 면 → 제품 상면 (포크 상면 55 + 제품 두께 20)
GAP = 25.0                             # 진입 시 포크 상면과 제품 하면 간격
OUT_Y = -170.0                         # 랙 앞 대기 시 제품 중심 Y
Z_VIA = 1330.0                         # 대기(Home) 높이 (입력 랙 앞, 제품 상면 기준)
XC_IN, XC_OUT = XI + 300.0, XO + 300.0  # 랙 작업 시 캐리지 위치 (두 랙 모두 제품이 베이스 왼쪽 300)
XC_JIG = CX + 150.0                    # 지그·검사 시 캐리지 위치 (검사 중 고정)
JIG_ABOVE_DZ = 120.0

# ---- 로봇 (표준 DH: d, a, alpha) — 6축 협동로봇 개념값, CS66 실제값으로 교체 필요 ----
DH = [(150.0, 0.0, math.pi / 2), (0.0, -420.0, 0.0), (0.0, -400.0, 0.0),
      (125.0, 0.0, math.pi / 2), (105.0, 0.0, -math.pi / 2), (100.0, 0.0, 0.0)]
JOINT_LIM = 360.0 * D2R
# PTP 목표 관절각 표기 기준 (J1~J5, 각 중심 ±180°). 이 레이아웃의 작업 자세(입·출력 랙 1~10단, 경유점, 지그, 검사)는
# 한 자세군(팔꿈치 J3<0, 손목 J5>0, J1이 랙 쪽)에서 모두 이 범위 안 → 관절 보간이 바닥 쪽으로 돌거나 사이클마다 감기지 않음
#   J1: 랙·지그 방향(≈180°) 중심, J2: −20° 중심 (+90° = 상완 수직 하향 → 그 너머 ±180° 쪽으로 넘어가지 않게)
#   J3: 0° 중심 (±180° = 전완이 상완 위로 완전히 접힘 → 통과 불가)
CANON_C = np.array([180.0, -20.0, 0.0, 80.0, 0.0]) * D2R
# 제7축 주변 간섭 상자 (xmin, xmax, ymin, ymax, ztop) — 캐리지는 xc 기준
RAIL_KEEPOUT = 60.0                    # PTP 목표 자세 선택 시 요구 여유 (이후 하강 43 mm 감안)
IK_TOL = 1e-3

# ---- 속도 제한 (V6) — 가정값. ELITE CS66 사양서·협동 운전 안전 설정값으로 교체 필요 ----
JOINT_VMAX = np.array([180.0, 180.0, 180.0, 180.0, 180.0, 180.0]) * D2R   # 관절별 최대 속도 [rad/s]
RAIL_VMAX = 1000.0                     # 제7축 최대 속도 [mm/s]
SELF_CLEAR_WARN = 20.0                 # 자기간섭 최소 거리 경고 기준 [mm]

# ---- 오픈형 철망 펜스 + 양개 전동 자동문 (V7) — 치수·시간은 개념값 ----
FENCE_OFF = 300.0                      # 설비 베이스 → 펜스 간격
FX0 = ENC_X0 - FENCE_OFF               # 펜스 좌측 X
FX1 = CAB_X0 - 25.0                    # 펜스 우측 X (전장함 좌측면에 붙임, 전장함이 펜스 역할)
FY0 = -ENC_Y - FENCE_OFF               # 전면 펜스 Y
FY1 = ENC_Y + FENCE_OFF                # 후면 펜스 Y
FENCE_H = 2000.0                       # 펜스 높이
FENCE_PANEL = 1000.0                   # 펜스 패널 최대 폭 (기둥 간격)
MESH_PITCH = 50.0                      # 철망 피치
DOOR_LP, DOOR_RP = -1300.0, 700.0      # 자동문 좌·우 구동 기둥 중심 X (기둥 중심 간 2,000)
DOOR_OV = 40.0                         # 텔레스코픽 도어 2장 겹침
DOOR_GAP = 10.0                        # 닫힘 시 중앙 맞닿음 간격 (고무 엣지 포함)
DOOR_W = (DOOR_RP - DOOR_LP - 60.0 - DOOR_GAP + 2 * DOOR_OV) / 4   # 도어 1장 폭 (≈ 500)
DOOR_TRACK = (FY0 - 50.0, FY0 - 90.0)  # 안쪽·바깥쪽 도어 레일 Y (고정 펜스 바깥)
POST_Y = (FY0 - 210.0, FY0 - 110.0)    # 구동 기둥 Y 범위 (도어 레일보다 바깥)
POST_H = 2100.0
DOOR_T = 3.0                           # 자동문 개폐 시간 [s] (가정)
DOOR_OPEN_HOLD = 2.0                   # 10개 완료 후 열림 상태 유지 (랙 교체 시간 — 표시용)


def seg_time(dq, dx, dt_cmd):
    """샘플 간 시간 = max(지령 시간, 관절 변화 / 관절 속도 한계, 제7축 이동 / 제7축 속도 한계)"""
    t_j = float(np.max(np.abs(dq) / JOINT_VMAX))
    return max(dt_cmd, t_j, abs(dx) / RAIL_VMAX)


def tier_z(k):
    return TIER_Z0 + TIER_P * k


def door_leaf_x0(side, idx, u):
    """자동문 도어 1장의 좌단 X.  side −1 좌 / +1 우,  idx 0 안쪽 레일 / 1 바깥쪽 레일,  u 열림량 0(닫힘) ~ 1(열림)
    텔레스코픽: 두 장 모두 같은 수납 위치로 선형 이동 → 바깥쪽 도어가 2배 거리·2배 속도"""
    if side < 0:
        closed = DOOR_LP + 30.0 + idx * (DOOR_W - DOOR_OV)
        opened = DOOR_LP + 50.0 - DOOR_W                  # 고정 펜스 뒤(바깥)로 수납
        return closed + (opened - closed) * u
    closed_r = DOOR_RP - 30.0 - idx * (DOOR_W - DOOR_OV)   # 우측은 오른쪽 끝 기준
    opened_r = DOOR_RP - 50.0 + DOOR_W
    return closed_r + (opened_r - closed_r) * u - DOOR_W


def find_font():
    for p in (r"C:\Windows\Fonts\malgun.ttf",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
              "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
              "/System/Library/Fonts/AppleSDGothicNeo.ttc"):
        if Path(p).exists():
            return p
    return None


# ===========================================================================
# 로봇 기구학 (V1)
# ===========================================================================
def dh_A(th, d, a, al):
    ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d], [0, 0, 0, 1.0]])


def fk(q):
    """관절각 q → 로봇 베이스 기준 링크 좌표계 [F0(베이스), F1, …, F6(플랜지)]"""
    T = np.eye(4)
    out = [T]
    for qi, (d, a, al) in zip(q, DH):
        T = T @ dh_A(qi, d, a, al)
        out.append(T)
    return out


def fk_flange(q):
    T = np.eye(4)
    for qi, (d, a, al) in zip(q, DH):
        T = T @ dh_A(qi, d, a, al)
    return T


def base_pose(xc):
    """제7축 캐리지 위치 xc → 월드 기준 로봇 베이스"""
    return H(None, (xc, RY, Z_BASE))


def ik_residual(q, M, w_rot=300.0):
    T = fk_flange(q)
    # 회전 오차: 행렬 성분 차이 (180° 뒤집힘 해를 배제하기 위해 vee 형식 대신 사용)
    return np.r_[T[:3, 3] - M[:3, 3], (T[:3, :3] - M[:3, :3]).ravel() * w_rot]


def pose_err(q, M):
    T = fk_flange(q)
    return float(np.linalg.norm(T[:3, 3] - M[:3, 3]) + 100.0 * np.linalg.norm(T[:3, :3] - M[:3, :3]))


def ik_solve(M, q0):
    r = least_squares(ik_residual, np.asarray(q0, float), args=(M,), method="lm",
                      xtol=1e-12, ftol=1e-12, max_nfev=600)
    return r.x, pose_err(r.x, M)


def nearest_equiv(q, ref, lim=JOINT_LIM):
    """각 관절을 2π 배수만큼 옮겨 ref에 가장 가깝게 (관절 한계 안에서)"""
    out = np.array(q, float)
    for j in range(len(out)):
        k0 = round((ref[j] - out[j]) / (2 * math.pi))
        cands = [out[j] + 2 * math.pi * (k0 + d) for d in (-2, -1, 0, 1, 2)]
        inside = [c for c in cands if abs(c) <= lim + 1e-9] or cands
        out[j] = min(inside, key=lambda c: abs(c - ref[j]))
    return out


def canon(q, q6_ref=None):
    """J1~J5를 표기 기준 범위(CANON_C ± 180°)로, J6은 q6_ref에 가장 가깝게(±360° 안)"""
    out = np.array(q, float)
    out[:5] = CANON_C + np.array([wrap_pi(v) for v in out[:5] - CANON_C])
    if q6_ref is not None:
        out[5] = nearest_equiv(out, np.r_[out[:5], q6_ref])[5]
    else:
        out[5] = wrap_pi(out[5])
    return out


def branch_sig(q):
    """자세군 구분: (팔꿈치 J3 부호, 손목 J5 부호) — 어깨 방향(J1)은 레일 간섭 여유로 자유 선택"""
    c = canon(q)
    return (math.sin(c[2]) < 0, math.sin(c[4]) > 0)


def full_sig(q):
    """자세군 + 어깨 방향 (J1이 랙 쪽 90°~270°인지)"""
    c = canon(q)
    return branch_sig(q) + (90 * D2R < c[0] < 270 * D2R,)


def L_(plan, ko, en):
    return ko if plan.ko else en


def rail_boxes(xc):
    """V4 형상 기준 제7축 주변 상자 (xmin, xmax, ymin, ymax, ztop)"""
    return [(xc - 150.0, xc + 150.0, RY - 150.0, RY + 150.0, Z_CARR[1]),                 # 캐리지 판
            (xc - 85.0, xc + 85.0, RY - 85.0, RY + 85.0, Z_BASE),                        # 라이저
            (xc + 20.0, xc + 130.0, RY - 260.0, RY - 150.0, Z_CARR[1]),                  # 주행 모터
            (RAIL_X0, RAIL_X1, RY - 120.0, RY + 120.0, Z_GUIDE[1]),                      # 레일·가이드
            (RAIL_X0 - 30.0, RAIL_X0, RY - 110.0, RY + 110.0, Z_GUIDE[1] + 20.0),        # 스토퍼(좌)
            (RAIL_X1, RAIL_X1 + 30.0, RY - 110.0, RY + 110.0, Z_GUIDE[1] + 20.0),        # 스토퍼(우)
            (RAIL_X0 + 100.0, RAIL_X1 - 100.0, RY + 130.0, RY + 230.0, 390.0),           # 케이블체인 트로프
            (xc - 20.0, xc + 20.0, RY + 100.0, RY + 220.0, Z_CARR[1])]                   # 체인 브래킷


_RAIL_PTS = {}


def rail_clear(q, xc, gamma):
    """로봇 링크(J2 이후)·툴 점과 제7축 캐리지·레일·체인 상자의 최소 높이 여유 [mm] (음수 = 간섭)"""
    if "rob" not in _RAIL_PTS:
        rob = {}
        for k, m, _ in robot_meshes(np.zeros(6)):
            if k >= 2:
                rob.setdefault(k, []).append(np.asarray(m.points)[::6])
        _RAIL_PTS["rob"] = {k: np.vstack(v) for k, v in rob.items()}
        tool = [np.asarray(m.points) for _, m, _ in tool_meshes()]
        _RAIL_PTS["tool"] = np.vstack(tool)[::2]
    Fs = fk(q)
    B = base_pose(xc)
    pts = []
    for k, P in _RAIL_PTS["rob"].items():
        M = B @ Fs[k]
        pts.append((M[:3, :3] @ P.T).T + M[:3, 3])
    Mt = B @ Fs[6] @ H(Rz(gamma))
    pts.append((Mt[:3, :3] @ _RAIL_PTS["tool"].T).T + Mt[:3, 3])
    W = np.vstack(pts)
    worst = np.inf
    for x0, x1, y0, y1, zt in rail_boxes(xc):
        m = (W[:, 0] > x0) & (W[:, 0] < x1) & (W[:, 1] > y0) & (W[:, 1] < y1)
        if m.any():
            worst = min(worst, float((W[m, 2] - zt).min()))
    return worst


def joint_origins_world(q, xc):
    B = base_pose(xc)
    return np.array([(B @ F)[:3, 3] for F in fk(q)])


def valid_pose(q, xc):
    O = joint_origins_world(q, xc)
    return O[2:, 2].min() > 280.0          # 관절 원점이 플린스·레일 위에 있을 것


# ===========================================================================
# 툴(포크형 엔드이펙터)·제품 좌표
# ===========================================================================
def M_F_P(gamma):
    """플랜지 → 제품 원점(상면 중심). 툴 좌표 = 플랜지를 Zf축으로 γ 회전, 제품은 +Yp로 L_FORK, +Zp로 H_TOOL"""
    return H(Rz(gamma)) @ H(None, (0.0, L_FORK, H_TOOL))


def prod_pose(p, tilt=0.0, psi=0.0):
    """제품 원점 위치 p, 기울기 tilt(Rx(−tilt)), 윤곽 추종 회전 ψ → 월드 기준 제품 자세"""
    return H(Rx(-tilt) @ Rz(psi), p)


def flange_target(pose, xc, gamma):
    p, tilt, psi = pose
    return inv_h(base_pose(xc)) @ prod_pose(p, tilt, psi) @ inv_h(M_F_P(gamma))


# ===========================================================================
# 검사 경로 (V1): 외측 → 전환 → 내측
# ===========================================================================
def inspection_path():
    loops = []
    for name, (a, b, r), inward in (("OUTER", OR_OUT, False), ("INNER", OR_IN, True)):
        pts, nr, perim = rounded_rect(a, b, r, STEP_MM)
        view = -nr if inward else nr                          # 관찰할 측면의 법선
        loops.append(dict(name=name, P=pts, V=view, perim=perim))
    # 외측 시작 = 하변(−Y측) 오른쪽 끝 → ψ ≈ 0 (지그 위 평면 자세에서 기울이기만 하면 됨)
    a, b, r = OR_OUT
    i0 = int(np.argmin(np.hypot(loops[0]["P"][:, 0] - (a - r), loops[0]["P"][:, 1] + b)))
    # 내측 시작 = 외측 끝점에서 가장 가까운 점 → 전환 이동 최소
    end_pt = np.roll(loops[0]["P"], -i0, axis=0)[-1]
    i1 = int(np.argmin(np.hypot(*(loops[1]["P"] - end_pt).T)))
    for lp, i_ in zip(loops, (i0, i1)):
        lp["P"] = np.roll(lp["P"], -i_, axis=0)
        lp["V"] = np.roll(lp["V"], -i_, axis=0)
        alpha = np.arctan2(lp["V"][:, 1], lp["V"][:, 0])
        lp["psi"] = np.unwrap(-math.pi / 2 - alpha)         # Rz(ψ)·n → −Y (로봇 쪽)
    po = loops[0]["psi"]
    po -= 2 * math.pi * round(po[0] / (2 * math.pi))
    pi_ = loops[1]["psi"]
    base = pi_ - 2 * math.pi * round((pi_[0] - po[-1]) / (2 * math.pi))
    # 내측 루프의 2π 배수 선택: 전체 ψ 누적 범위(= J6 회전량) 최소
    best = min((base + 2 * math.pi * k for k in (-1, 0, 1)),
               key=lambda c: max(po.max(), c.max()) - min(po.min(), c.min()))
    loops[1]["psi"] = best

    frames = []
    lo, li = loops
    for i in range(len(lo["P"])):
        frames.append(dict(loop="OUTER", inspect=True, idx=i, P=lo["P"][i], psi=float(lo["psi"][i])))
    P0, P1, s0, s1 = lo["P"][-1], li["P"][0], lo["psi"][-1], li["psi"][0]
    for j in range(1, TRANS_N + 1):
        u = smooth(j / (TRANS_N + 1))
        frames.append(dict(loop="MOVE", inspect=False, idx=j, P=P0 + (P1 - P0) * u, psi=float(s0 + (s1 - s0) * u)))
    for i in range(len(li["P"])):
        frames.append(dict(loop="INNER", inspect=True, idx=i, P=li["P"][i], psi=float(li["psi"][i])))
    limit_tilt(frames)
    for f in frames:
        R = Rx(-f["tilt"]) @ Rz(f["psi"])
        Pp = np.r_[f["P"], 0.0]
        f["T"] = Q - R @ Pp                                   # Tᵢ = Q − RᵢPᵢ
        f["check"] = float(np.linalg.norm(f["T"] + R @ Pp - Q))  # 검산: 0
    return frames, loops


def limit_tilt(frames):
    """렌즈·링조명 간섭 시 해당 구간만 기울기 자동 축소.
    내측 윤곽 짧은 변(양 끝)을 볼 때는 제품 몸체 332 mm가 카메라 쪽으로 45° 상승 →
    WD 87 mm · 링조명 Ø180 조건에서 렌즈 하부 원통(반경 LENS_R)에 닿음. 여유 LENS_MARGIN 확보까지 tilt 축소."""
    pts = [np.asarray(m.points) for _, m, _ in product_meshes()]
    for _, m, _ in tool_meshes():                              # 툴 점 → 제품 좌표 (축 동일, 원점 이동)
        pts.append(np.asarray(m.points) - (0.0, L_FORK, H_TOOL))
    off = PROD_W / 2 + 6
    for s in (-1, 1):
        pts.append(np.asarray(jaw_mesh().points) + (0.0, s * off, -H_TOOL))
    pts = np.vstack(pts)[::2]
    t_max = TILT_DEG * D2R

    def clearance(f, tilt):
        R = Rx(-tilt) @ Rz(f["psi"])
        W = (R @ (pts - np.r_[f["P"], 0.0]).T).T + Q
        inside = np.hypot(W[:, 0] - CX, W[:, 1] - CY) < LENS_R
        return float((Z_LENS - W[inside, 2]).min()) if inside.any() else np.inf

    raw = np.full(len(frames), t_max)
    for n, f in enumerate(frames):
        if clearance(f, t_max) >= LENS_MARGIN:
            continue
        lo, hi = TILT_MIN * D2R, t_max
        for _ in range(16):
            mid = (lo + hi) / 2
            if clearance(f, mid) >= LENS_MARGIN:
                lo = mid
            else:
                hi = mid
        raw[n] = lo
    # 부드럽게: 최소 필터(반폭 2w) → 이동평균(반폭 w)  ⇒ 결과 ≤ 원래 허용값
    w = 12
    mn = np.array([raw[max(0, i - 2 * w): i + 2 * w + 1].min() for i in range(len(raw))])
    sm = np.array([mn[max(0, i - w): i + w + 1].mean() for i in range(len(raw))])
    for f, t in zip(frames, sm):
        f["tilt"] = float(min(t, t_max))


# ===========================================================================
# 동작 프로그램 (시간 샘플 + 이벤트)
# ===========================================================================
class Motion:
    """샘플 = (시간, 캐리지 xc, 관절각 q, 그립, 단계명, 검사 진행, 검사 프레임 번호)"""

    def __init__(self, gamma, rng):
        self.g = gamma
        self.Mfp_inv = inv_h(M_F_P(gamma))
        self.rng = rng
        self.T, self.XC, self.Q, self.G, self.LAB, self.PROG, self.INS = [], [], [], [], [], [], []
        self.events = []
        self.t = 0.0
        self.q = None
        self.xc = None
        self.grip = 0.0
        self.pose = None
        self.label = ""
        self.prog = None
        self.max_err = 0.0
        self.warn = []

    def target(self, pose, xc):
        p, tilt, psi = pose
        return inv_h(base_pose(xc)) @ prod_pose(p, tilt, psi) @ self.Mfp_inv

    def _push(self, ins=-1):
        self.T.append(self.t)
        self.XC.append(self.xc)
        self.Q.append(np.array(self.q))
        self.G.append(self.grip)
        self.LAB.append(self.label)
        self.PROG.append(self.prog)
        self.INS.append(ins)

    def start(self, q, xc, pose, label=""):
        self.q, self.xc, self.pose, self.label = np.array(q, float), xc, pose, label
        self.ref_sig = branch_sig(self.q)                  # 전체 사이클을 같은 자세군으로
        self._push()

    def ev(self, kind, arg=None):
        self.events.append((self.t, kind, arg))

    def hold(self, dur, label=None):
        if label is not None:
            self.label = label
        if dur > 0:
            self.t += dur
            self._push()

    def set_grip(self, g, dur, label=None):
        if label is not None:
            self.label = label
        g0, t0, n = self.grip, self.t, 6
        for k in range(1, n + 1):
            u = k / n
            self.grip = g0 + (g - g0) * u
            self.t = t0 + dur * u
            self._push()

    def solve_target(self, M, xc):
        """PTP 목표 IK: 기준 자세군(팔꿈치·손목) 해들 중 제7축 레일 여유 확보 + 이전 자세에 가장 가까운 해
        → 표기 기준 범위(canon)로 정리"""
        cands = []

        def add(q, e):
            if e < IK_TOL and valid_pose(q, xc) and (self.ref_sig is None or branch_sig(q) == self.ref_sig):
                qn = canon(q, self.q[5])
                if not any(np.abs(qn[:5] - c[:5]).max() < 1e-3 for c in cands):
                    cands.append(qn)

        add(*ik_solve(M, self.q))
        for s in range(60):
            seed = self.q + self.rng.normal(0, 0.9, 6) if s < 12 else self.rng.uniform(-math.pi, math.pi, 6)
            add(*ik_solve(M, seed))
        if not cands:
            raise RuntimeError(f"IK 해 없음(기준 자세군): 목표 {np.round(M[:3, 3], 1)} (베이스 기준), xc={xc:.0f}")

        def score(qn):
            clr = rail_clear(qn, xc, self.g)
            return float(np.abs(qn - self.q).sum()) + 50.0 * max(0.0, RAIL_KEEPOUT - clr)

        return min(cands, key=score)

    def ptp(self, pose, xc, dur, label, q_target=None, n=18):
        """관절 보간 이동 (제7축 동시 이동)"""
        self.label = label
        q0, x0, t0 = self.q.copy(), self.xc, self.t
        q1 = np.array(q_target, float) if q_target is not None else self.solve_target(self.target(pose, xc), xc)
        # 지령 시간 dur 안에서 속도 한계를 넘으면 전체 이동 시간을 늘림 (smooth 곡선 최대 기울기 1.5 반영)
        need = 1.5 * max(float(np.max(np.abs(q1 - q0) / JOINT_VMAX)), abs(xc - x0) / RAIL_VMAX)
        dur = max(dur, need)
        for k in range(1, n + 1):
            u = k / n
            s = smooth(u)
            self.q = q0 + (q1 - q0) * s
            self.xc = x0 + (xc - x0) * s
            self.t = t0 + dur * u
            self._push()
        self.pose = pose
        return q1

    def lin(self, pose, dur, label, xc=None, step=10.0):
        """직선 보간 이동 (제품 위치·기울기·ψ 선형 보간, 매 샘플 IK 연속 추종)"""
        self.label = label
        (p0, tl0, ps0), (p1, tl1, ps1) = self.pose, pose
        ps1 = ps1 + 2 * math.pi * round((ps0 - ps1) / (2 * math.pi))   # 같은 방향은 회전 없이 (2π 표기 차이 무시)
        pose = (p1, tl1, ps1)
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        x0 = self.xc
        x1 = x0 if xc is None else xc
        n = max(3, math.ceil(max(np.linalg.norm(p1 - p0) / step, abs(tl1 - tl0) / (2 * D2R),
                                 abs(ps1 - ps0) / (2 * D2R), abs(x1 - x0) / step)))
        u_prev = 0.0
        for k in range(1, n + 1):
            u = k / n
            s = smooth(u)
            ps = (p0 + (p1 - p0) * s, tl0 + (tl1 - tl0) * s, ps0 + (ps1 - ps0) * s)
            q_prev, xc_prev = self.q.copy(), self.xc
            self.xc = x0 + (x1 - x0) * s
            self._ik_step(self.target(ps, self.xc), label)
            self.t += seg_time(self.q - q_prev, self.xc - xc_prev, dur * (u - u_prev))
            u_prev = u
            self._push()
        self.pose = pose

    def _ik_step(self, M, label):
        q, e = ik_solve(M, self.q)
        if e > IK_TOL:
            for _ in range(20):
                q2, e2 = ik_solve(M, self.q + self.rng.normal(0, 0.15, 6))
                if e2 < e:
                    q, e = q2, e2
                if e < IK_TOL:
                    break
            if e > IK_TOL:
                self.warn.append(f"IK 오차 {e:.2e} @ {label}")
        if np.abs(q - self.q).max() > 25 * D2R:
            self.warn.append(f"관절 급변 {np.abs(q - self.q).max() / D2R:.0f}° @ {label}")
        self.max_err = max(self.max_err, e)
        self.q = q

    def insp_block(self, frames, xc, label_fn, n_out, n_in):
        """검사 경로 프레임 연속 IK (캐리지 고정)"""
        self.xc = xc
        for n, f in enumerate(frames):
            self.label = label_fn(f)
            q_prev = self.q.copy()
            self._ik_step(self.target((f["T"], f["tilt"], f["psi"]), xc), self.label)
            if f["loop"] == "OUTER":
                self.prog = (f["idx"] + 1, 0)
            elif f["loop"] == "MOVE":
                self.prog = (n_out, 0)
            else:
                self.prog = (n_out, f["idx"] + 1)
            # 검사점 시간 = 지령 간격과 관절 속도 한계 중 큰 값 (코너에서 자동 감속)
            self.t += seg_time(self.q - q_prev, 0.0, DT_INSP if f["inspect"] else DT_TRANS)
            self._push(ins=n)
        f = frames[-1]
        self.pose = (f["T"], f["tilt"], f["psi"])

    # ---- 구간 기록·재생 (동일한 지그·검사 구간을 제품마다 재사용) ----
    def span_begin(self):
        return dict(i0=len(self.T), t0=self.t, ev0=len(self.events), q0=self.q.copy(), xc0=self.xc,
                    pose0=self.pose)

    def span_end(self, sp):
        i0 = sp["i0"]
        sp.update(T=np.array(self.T[i0:]) - sp["t0"], XC=list(self.XC[i0:]), Q=[q.copy() for q in self.Q[i0:]],
                  G=list(self.G[i0:]), LAB=list(self.LAB[i0:]), PROG=list(self.PROG[i0:]), INS=list(self.INS[i0:]),
                  EV=[(t - sp["t0"], k, a) for t, k, a in self.events[sp["ev0"]:]],
                  pose1=self.pose, dur=self.t - sp["t0"])
        return sp

    def replay(self, sp, off=None):
        t0 = self.t
        off = np.zeros(6) if off is None else off
        for k in range(len(sp["T"])):
            self.t = t0 + sp["T"][k]
            self.xc, self.q, self.grip = sp["XC"][k], sp["Q"][k] + off, sp["G"][k]
            self.label, self.prog = sp["LAB"][k], sp["PROG"][k]
            self._push(ins=sp["INS"][k])
        for t, kd, a in sp["EV"]:
            self.events.append((t0 + t, kd, a))
        self.t = t0 + sp["dur"]
        self.pose = sp["pose1"]


def ik_multi(M, xc, rng, n_seeds=300):
    """첫 자세: 무작위 초기값 탐색 → 팔꿈치 위(elbow-up)·바닥 위 해 선택 (V1)"""
    best = None
    for _ in range(n_seeds):
        q, e = ik_solve(M, rng.uniform(-math.pi, math.pi, 6))
        if e > 1e-6 or not valid_pose(q, xc):
            continue
        q = np.array([wrap_pi(v) for v in q])
        O = joint_origins_world(q, xc)
        score = O[2][2] - 0.3 * abs(O[2][1] - RY)
        if best is None or score > best[0]:
            best = (score, q)
    if best is None:
        raise RuntimeError("초기 자세 IK 해 없음 — 레이아웃·툴 오프셋·DH 값을 확인하세요.")
    return best[1]


# ===========================================================================
# 사이클 계획 (V4 사이클 + V1 검사 경로)
# ===========================================================================
class Plan:
    def __init__(self, ko=True, verbose=True):
        self.ko = ko
        t_start = time.time()
        self.frames, self.loops = inspection_path()
        self.n_out, self.n_in = len(self.loops[0]["P"]), len(self.loops[1]["P"])
        rng = np.random.default_rng(7)
        self.rng_sw = np.random.default_rng(11)
        L = self.L
        self.home_pose = ((XI, OUT_Y, Z_VIA), 0.0, 0.0)
        self.jig_above = ((CX, CY, Z_JIG + PROD_T + JIG_ABOVE_DZ), 0.0, 0.0)
        self.jig_stage = ((CX, OUT_Y, Z_VIA), 0.0, 0.0)       # 지그 앞 이송면 (지그·검사 구간 시작·끝)

        # ---- 1차: γ = 0 으로 지그·검사 구간 계산 → J6 중앙 정렬 그리퍼 장착각 γ 결정 ----
        if verbose:
            print("  [1/3] 초기 자세·검사 구간 IK (γ=0)…", flush=True)
        q_home0 = canon(ik_multi(flange_target(self.home_pose, XC_IN, 0.0), XC_IN, rng))
        m1 = Motion(0.0, rng)
        m1.start(q_home0, XC_IN, self.home_pose)
        m1.ptp(self.jig_stage, XC_JIG, 2.6, "")
        sp = m1.span_begin()
        m1.lin(self.jig_above, 1.0, L("④ 검사부로 이동 — 지그 위 접근", "Approach jig"))
        self._jig_span(m1)
        m1.lin((self.jig_stage[0], 0.0, m1.pose[2]), 1.0, L("검사 완료 — 이송면으로 후퇴", "Retract to transfer plane"))
        sp = m1.span_end(sp)
        # J6 중앙 정렬: 접근·검사·후퇴 구간 전체 (검사 ψ 누적 ~530° + 접근 J6 회전 → ±360° 안에 들어오도록)
        q6 = np.array([q[5] for q in sp["Q"]])
        center = (q6.max() + q6.min()) / 2
        self.gamma = wrap_pi(center)
        k = round((center - self.gamma) / (2 * math.pi))
        shift = self.gamma + 2 * math.pi * k                 # γ 반영 + 2π 배수 → 검사 구간 J6 중앙 = 0
        for q in sp["Q"]:
            q[5] -= shift
        sp["q0"][5] -= shift
        q_home = q_home0.copy()
        q_home[5] = wrap_pi(q_home[5] - self.gamma)
        self.jig_span = sp
        self.warn = list(m1.warn)
        if verbose:
            print(f"        그리퍼 장착각 γ = {self.gamma / D2R:.1f}° (플랜지 Zf축 기준), "
                  f"지그·검사 구간 J6 {(q6.min() - shift) / D2R:.0f}° ~ {(q6.max() - shift) / D2R:.0f}°", flush=True)
            print("  [2/3] 랙 입·출력 동작 IK…", flush=True)

        # ---- 2차: 전체 사이클 ----
        m = Motion(self.gamma, rng)
        # 자동문: 사이클 시작은 열림(랙 교체 직후) → 닫힘·인터록 확인 후 로봇 기동
        m.start(q_home, XC_IN, self.home_pose, L("대기 — 자동문 열림 (랙 교체 완료)", "Home - door open"))
        m.hold(0.5)
        t_close = m.t
        m.label = L("자동문 닫힘 — 안전 인터록 확인 중 (로봇 정지)", "Auto door closing - interlock")
        m._push()
        m.hold(DOOR_T)
        self.door_keys = [(0.0, 1.0), (t_close, 1.0), (m.t, 0.0)]
        m.hold(0.4, L("대기 — 자동문 닫힘 확인, 자동 운전 시작", "Home - door closed, auto start"))
        n_ok = n_ng = 0
        self.out_plan = []
        for kk in range(TIERS):
            tier = TIERS - 1 - kk                            # 입력 랙 위에서부터
            is_ng = kk in NG_PARTS
            self._input_span(m, kk, tier)
            self._transfer(m, self.jig_stage, XC_JIG, L("④ 검사부로 이동", "To inspection"), q_stage=sp["q0"])
            m.replay(sp)
            m.ev("judge", "NG" if is_ng else "OK")
            if is_ng:
                otier = NG_TIERS[n_ng]
                n_ng += 1
            else:
                otier = n_ok
                n_ok += 1
            self.out_plan.append(("NG" if is_ng else "OK", otier))
            self._output_span(m, kk, otier, is_ng)
            if verbose:
                print(f"        제품 {kk + 1:2d}/{TIERS}  입력 {tier + 1}단 → 출력 {otier + 1}단 ({'NG' if is_ng else 'OK'})",
                      flush=True)
        # 시작 자세로 복귀 → 사이클 반복 시 끊김 없음
        self._transfer(m, self.home_pose, XC_IN, L("10개 완료 — 대기 위치 복귀", "All done - return home"), q_stage=q_home)
        m.hold(0.5, L("10개 완료 — 대기 위치 정지 확인", "All done - robot stopped"))
        t_open = m.t
        m.label = L("자동문 열림 — 로봇 정지 상태", "Auto door opening")
        m._push()
        m.hold(DOOR_T)
        self.door_keys += [(t_open, 0.0), (m.t, 1.0)]
        self.door_open_t = t_open
        m.label = L("자동문 열림 — 랙 교체 대기 (처음부터 반복)", "Door open - rack exchange, repeat")
        m._push()
        m.hold(DOOR_OPEN_HOLD)
        self.warn += m.warn
        self.max_err = max(m.max_err, m1.max_err)

        self.T = np.array(m.T)
        self.XC = np.array(m.XC)
        self.Q = np.array(m.Q)
        self.G = np.array(m.G)
        self.LAB = m.LAB
        self.PROG = m.PROG
        self.INS = np.array(m.INS)
        self.events = sorted(m.events, key=lambda e: e[0])
        self.total = float(self.T[-1])
        self.cycle1 = next(t for t, kd, _ in self.events if kd == "place_out") + 3.0
        if verbose:
            print(f"  [3/3] 완료: 샘플 {len(self.T)}개, 사이클 {self.total:.0f} s, 계산 {time.time() - t_start:.1f} s",
                  flush=True)

    def L(self, ko, en):
        return ko if self.ko else en

    # ---- 장거리 이동 ----
    #  1) 전면 이송면(Y=OUT_Y, 제품 상면 Z_VIA)으로 직선 후퇴
    #  2) 이송면 안에서 직선 이동 + 제7축 동시 주행 (제품·포크가 포털·랙 기둥 쪽(+Y)으로 휘지 않음)
    #  3) 목적지로 직선 접근 (q_stage가 주어지면 이송면 자세 그 자체가 목적지 → 관절 표기만 맞춤)
    def _transfer(self, m, dest_pose, dest_xc, label, q_stage=None):
        (p0, tilt0, psi0) = m.pose
        p0 = np.asarray(p0, float)
        st1 = ((p0[0], OUT_Y, Z_VIA), 0.0, psi0)
        d1 = float(np.linalg.norm(p0 - np.asarray(st1[0])))
        if d1 > 1.0:
            m.lin(st1, 0.3 + d1 / 450.0, label)
        pd = np.asarray(dest_pose[0], float)
        # ψ는 같은 방향이라도 2π 배수 표기가 다를 수 있음(검사 후 ψ=360°) → 현재 표기 유지 (이동 중 제품 회전 방지)
        psi_d = dest_pose[2] + 2 * math.pi * round((psi0 - dest_pose[2]) / (2 * math.pi))
        st2 = ((pd[0], OUT_Y, Z_VIA), 0.0, psi_d)
        dx = max(abs(pd[0] - p0[0]), abs(dest_xc - m.xc))
        if dx > 1.0:
            m.lin(st2, max(1.0, 0.6 + dx / 900.0), label, xc=dest_xc, step=15.0)
        if q_stage is not None:
            if np.abs(m.q - q_stage).max() > 1e-6:
                # 같은 자세, J6만 360° 차이 → 이송점(빈 공간)에서 J6 되감기 (검사 구간 J6 중앙 정렬 유지)
                m.ptp(st2, dest_xc, 1.6, L_(self, "J6 되감기 (이송점, 제자리 회전)", "J6 unwind in place"),
                      q_target=q_stage, n=24)
                m.label = label
            m.pose = ((pd[0], OUT_Y, Z_VIA), 0.0, 0.0)
        else:
            d2 = float(np.linalg.norm(pd - np.asarray(st2[0])))
            if d2 > 1.0:
                m.lin((dest_pose[0], dest_pose[1], psi_d), 0.3 + d2 / 450.0, label)

    # ---- 입력 랙: 하부 진입 → SN 판독 → 리프트·파지 → 인출 ----
    def _input_span(self, m, kk, tier):
        L = self.L
        zb = tier_z(tier)

        def P(y, z_bottom):
            return ((XI, y, z_bottom + PROD_T), 0.0, 0.0)

        m.prog = None
        self._transfer(m, P(OUT_Y, zb - GAP), XC_IN, L(f"제품 {kk + 1}: 입력 랙 {tier + 1}단 앞으로 이동",
                                                       f"Part {kk + 1}: to input tier {tier + 1}"))
        m.lin(P(RACK_Y, zb - GAP), 1.0, L("② 하부 진입 (포크 수평 삽입)", "Fork in under part"))
        m.ev("sn", tier)
        m.hold(0.9, L("SN 판독 (포크 내장 카메라, 제품 하면)", "Read serial number"))
        m.lin(P(RACK_Y, zb), 0.5, L("③ 상향 리프트", "Lift"))
        m.prog = (0, 0)
        m.ev("pick_in", tier)
        m.set_grip(1.0, 0.35, L("③ 파지", "Grip"))
        m.lin(P(RACK_Y, zb + 18), 0.4, L("받침 립 넘김", "Clear lip"))
        m.lin(P(OUT_Y, zb + 18), 0.9, L("④ 인출", "Withdraw"))

    # ---- 지그 정렬 → 재파지 → 45° 윤곽 검사 → 복귀 (제품마다 동일 → 1회 계산 후 재사용) ----
    def _jig_span(self, m):
        L = self.L
        zt = Z_JIG + PROD_T
        on_jig = ((CX, CY, zt), 0.0, 0.0)
        m.lin(on_jig, 0.8, L("정렬 지그 안착", "Place on jig"))
        m.ev("to_jig")
        m.set_grip(0.0, 0.3, L("정렬 지그 안착 — 그립 해제", "Release"))
        m.lin(((CX, CY, zt - GAP), 0.0, 0.0), 0.4, L("지그 위치 보정 (핀 정렬)", "Jig alignment"))
        m.hold(0.6)
        m.lin(on_jig, 0.4, L("재파지", "Re-grip"))
        m.ev("from_jig")
        m.set_grip(1.0, 0.3, L("재파지", "Re-grip"))
        m.lin(self.jig_above, 0.6, L("지그에서 들어 올림", "Lift from jig"))
        f0, f1 = self.frames[0], self.frames[-1]
        pre = (f0["T"] - (0, 0, APPROACH_DZ), f0["tilt"], f0["psi"])
        m.ev("tilt_begin")
        m.lin(pre, 2.0, L(f"⑤ 검사 자세 진입 ({TILT_DEG:.0f}° 기울임, 초점면 아래 {APPROACH_DZ:.0f} mm)",
                          "Tilt to inspection pose"))
        m.lin((f0["T"], f0["tilt"], f0["psi"]), 0.6, L("⑤ 초점면 상승", "Rise to focus"))

        def lab(f):
            if f["loop"] == "MOVE":
                return L("⑤ 외측 → 내측 전환 (검사 없음)", "Outer → inner transfer")
            n = self.n_out if f["loop"] == "OUTER" else self.n_in
            nm = L("외측", "outer") if f["loop"] == "OUTER" else L("내측", "inner")
            return L(f"⑤ 정밀검사 {nm} {f['idx'] + 1}/{n} (POMEAS 고정 카메라, WD {WD:.0f})",
                     f"Inspection {nm} {f['idx'] + 1}/{n}")

        m.insp_block(self.frames, XC_JIG, lab, self.n_out, self.n_in)
        m.prog = (self.n_out, self.n_in)
        m.lin((f1["T"] - (0, 0, APPROACH_DZ), f1["tilt"], f1["psi"]), 0.5, L("검사 완료 — 초점면 이탈", "Leave focus"))
        psi_flat = 2 * math.pi * round(f1["psi"] / (2 * math.pi))
        m.lin(((CX, CY, Z_JIG + PROD_T + JIG_ABOVE_DZ), 0.0, psi_flat), 2.0, L("검사 완료 — 판정", "Judge"))
        m.ev("tilt_end")

    # ---- 출력 랙: 경유 → 단 앞 → 삽입 → 하강·해제 → 포크 후퇴 ----
    def _output_span(self, m, kk, otier, is_ng):
        L = self.L
        zo = tier_z(otier)
        lab = L("불량(NG) 단 적재", "Place NG") if is_ng else L("양품(OK) 단 적재", "Place OK")

        def P(y, z_bottom):
            return ((XO, y, z_bottom + PROD_T), 0.0, 0.0)

        self._transfer(m, P(OUT_Y, zo + 18), XC_OUT, L(f"{lab} — 출력 랙 {otier + 1}단 앞으로 이동", lab))
        m.lin(P(RACK_Y, zo + 18), 0.9, L(f"{lab} — 삽입", lab))
        m.lin(P(RACK_Y, zo), 0.4, lab)
        m.ev("place_out", (otier, "NG" if is_ng else "OK"))
        m.set_grip(0.0, 0.3, lab)
        m.prog = None
        m.lin(P(RACK_Y, zo - GAP), 0.4, L("하강", "Lower"))
        m.lin(P(OUT_Y, zo - GAP), 0.8, L("포크 후퇴", "Withdraw fork"))

    # ---- 시간 t → 보간 상태 ----
    def sample(self, t):
        t = min(max(t, 0.0), self.total)
        i = int(np.searchsorted(self.T, t, side="right")) - 1
        i = max(0, min(i, len(self.T) - 1))
        j = min(i + 1, len(self.T) - 1)
        dt = self.T[j] - self.T[i]
        u = (t - self.T[i]) / dt if dt > 1e-12 else 0.0
        xc = self.XC[i] + (self.XC[j] - self.XC[i]) * u
        q = self.Q[i] + (self.Q[j] - self.Q[i]) * u
        g = self.G[i] + (self.G[j] - self.G[i]) * u
        k = j if u > 0.5 else i
        return xc, q, g, self.LAB[k], self.PROG[k], int(self.INS[k])

    def door_u(self, t):
        """시간 t → 자동문 열림량 0(닫힘) ~ 1(열림), 구간별 가감속(smooth)"""
        keys = self.door_keys
        if t <= keys[0][0]:
            return keys[0][1]
        for (t0, u0), (t1, u1) in zip(keys, keys[1:]):
            if t0 <= t <= t1:
                s = smooth((t - t0) / (t1 - t0)) if t1 > t0 else 1.0
                return u0 + (u1 - u0) * s
        return keys[-1][1]


# ===========================================================================
# 형상: 로봇 (V1 방식 — 링크 좌표계 메쉬 + user_matrix)
# ===========================================================================
def cyl_mesh(c, axis, r, h, res=40):
    return pv.Cylinder(center=c, direction=axis, radius=r, height=h, resolution=res, capping=True).triangulate()


def seg_mesh(p0, p1, r):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    return cyl_mesh((p0 + p1) / 2, p1 - p0, r, float(np.linalg.norm(p1 - p0)))


def box_mesh(x0, x1, y0, y1, z0, z1):
    return pv.Box(bounds=(x0, x1, y0, y1, z0, z1)).triangulate()


def robot_meshes(q_ref):
    """기준 자세에서 만든 로봇 형상을 각 링크 좌표계로 변환 → (링크번호, 로컬 메쉬, 색)"""
    Fs = fk(q_ref)
    O = [F[:3, 3] for F in Fs]
    Z = [F[:3, 2] for F in Fs]
    W, J, B = COL["robot"], COL["robot_j"], COL["robot_b"]
    items = []

    def housing(k, c, ax, r, h, ring=False):
        items.append((k, cyl_mesh(c, ax, r, h), W))
        ax = nrm(ax)
        for sgn in (-1, 1):
            items.append((k, cyl_mesh(c + sgn * ax * (h / 2 + 4), ax, r + 0.5, 8), B if ring and sgn > 0 else J))

    items.append((0, cyl_mesh((0, 0, 8), (0, 0, 1), 82, 16), J))
    items.append((0, cyl_mesh((0, 0, 70), (0, 0, 1), 62, 110), W))
    items.append((1, cyl_mesh((0, 0, 128), (0, 0, 1), 63, 8), B))
    housing(1, O[1] + 70 * Z[1], Z[1], 62, 150, ring=True)            # J2
    items.append((2, seg_mesh(O[1] + 90 * Z[1], O[2] + 90 * Z[1], 46), W))   # 상완
    housing(2, O[2] + 60 * Z[2], Z[2], 54, 140, ring=True)            # J3
    items.append((3, seg_mesh(O[2] + 25 * Z[2], O[3] + 25 * Z[2], 38), W))   # 전완
    housing(3, O[3] + 70 * Z[3], Z[3], 42, 120)                       # J4
    items.append((4, seg_mesh(O[4], O[5], 38), W))
    housing(4, O[5] - 10 * Z[4], Z[4], 42, 95)                        # J5
    items.append((5, seg_mesh(O[5], O[6] - 10 * Z[5], 38), W))        # J6
    items.append((6, cyl_mesh(O[6] - 6 * Z[5], Z[5], 40, 12), J))     # 플랜지
    return [(k, m.transform(inv_h(Fs[k]), inplace=False), c) for k, m, c in items]


def tool_meshes():
    """포크형 엔드이펙터 (툴 좌표: 원점 = 플랜지 면, +Y = 포크 진입, +Z = 플랜지 축 = 제품 법선)"""
    zb = H_TOOL - PROD_T                                   # 포크 상면 = 제품 하면 (55)
    out = [("adapter", cyl_mesh((0, 0, 6), (0, 0, 1), 40, 12), COL["robot_j"]),
           ("body", box_mesh(-45, 45, -42, 42, 12, 52), COL["grip"]),
           ("arm", box_mesh(-28, 28, 30, L_FORK - 70, zb - 18, zb - 2), COL["alu_d"]),
           ("plate", box_mesh(-75, 75, L_FORK - 85, L_FORK + 38, zb - 10, zb), COL["grip_j"]),
           ("forkcam", cyl_mesh((0, L_FORK, zb - 6), (0, 0, 1), 11, 10), COL["cam"])]
    return out


def jaw_mesh():
    zb = H_TOOL - PROD_T
    return box_mesh(-32, 32, -6, 6, zb - 15, H_TOOL - 2)


def product_meshes():
    """제품 좌표 형상 (원점 = 상면 중심, Xp 길이 / Yp 폭 / Zp 상면 법선): (이름, 메쉬, 색)"""
    pts, _, _ = rounded_rect(PROD_L / 2, PROD_W / 2, PROD_R, 2.0)
    P = np.c_[pts, np.full(len(pts), -PROD_T)]
    poly = pv.PolyData(P, faces=np.r_[len(P), np.arange(len(P))])
    body = poly.extrude((0, 0, PROD_T), capping=True).triangulate()
    oa, ob, orr = OR_OUT
    ia, ib, irr = OR_IN
    pts, _, _ = rounded_rect((oa + ia) / 2, (ob + ib) / 2, (orr + irr) / 2, 2.0)
    ring = np.c_[pts, np.full(len(pts), 0.6)]
    oring = pv.lines_from_points(np.r_[ring, ring[:1]]).tube(radius=(oa - ia) / 2, n_sides=12)
    return [("body", body, COL["product"]), ("oring", oring, COL["oring"])]


# ===========================================================================
# 정적 형상 (V4)
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
        sc.box((px - 70, RY - 130, Z_PAD[0]), (px + 70, RY + 130, Z_PAD[1]), COL["steel_m"], spec=0.35, tag="steel")
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
            sc.bar((xs_, RY + s * 350, 110), (xs_, RY + s * 132, Z_BEAM[1] - 25), 60, 60, st, up=(1, 0, 0), **T)
            yf = RY + s * 330
            sc.cyl((xs_, yf, 0), (xs_, yf, 12), 40, COL["bolt"], n=20, tag="steel")
            sc.cyl((xs_, yf, 12), (xs_, yf, 30), 10, COL["bolt"], n=10, tag="steel")
            ya_ = RY + s * 380
            sc.box((xs_ - 60, min(ya_, ya_ + s * 90), 0), (xs_ + 60, max(ya_, ya_ + s * 90), 12), st, **T)
            for dx in (-35, 35):
                sc.hexbolt((xs_ + dx, ya_ + s * 50, 12), s=11, h=9, tag="steel")


def build_rail_static(sc: Scene):
    r = COL["rail"]
    sc.box((RAIL_X0, RY - 110, Z_RAIL[0]), (RAIL_X1, RY + 110, Z_RAIL[1]), COL["rail_cover"], spec=0.35, tag="rail")
    sc.box((RAIL_X0, RY - 120, Z_RAIL[0]), (RAIL_X1, RY + 120, Z_RAIL[0] + 10), r, tag="rail")
    for s in (-1, 1):
        sc.box((RAIL_X0, RY + s * 75 - 14, Z_GUIDE[0]), (RAIL_X1, RY + s * 75 + 14, Z_GUIDE[1]),
               COL["guide"], spec=0.5, tag="rail")
    for x in (RAIL_X0 - 30, RAIL_X1):
        sc.box((x, RY - 110, Z_RAIL[0]), (x + 30, RY + 110, Z_GUIDE[1] + 20), COL["yellow"], tag="rail")
    sc.box((RAIL_X0 + 100, RY + 130, 282), (RAIL_X1 - 100, RY + 230, 290), COL["alu_d"], tag="rail")
    for yy in (RY + 130, RY + 224):
        sc.box((RAIL_X0 + 100, yy, 290), (RAIL_X1 - 100, yy + 6, 380), COL["alu_d"], tag="rail")


def build_rack(sc: Scene, xc, label_col_fn):
    """10단 개방형 랙: 제품 양 끝만 받침(노랑) → 중앙 하부 개방 (포크 진입·SN 판독)"""
    al = COL["alu"]
    T = dict(tag="rack", spec=0.35)
    y_f, y_b = RACK_Y - 75, RACK_Y + 85
    xl, xr = xc - PROD_L / 2 - 40, xc + PROD_L / 2 + 40
    for x in (xl, xr):
        for y in (y_f, y_b):
            sc.box((x - 20, y - 20, Z_PLINTH), (x + 20, y + 20, tier_z(TIERS - 1) + 160), al, **T)
        sc.box((x - 20, y_f, Z_PLINTH), (x + 20, y_b, Z_PLINTH + 40), al, **T)
        sc.box((x - 20, y_f, tier_z(TIERS - 1) + 120), (x + 20, y_b, tier_z(TIERS - 1) + 160), al, **T)
    for y in (y_f, y_b):
        sc.box((xl, y - 20, tier_z(TIERS - 1) + 120), (xr, y + 20, tier_z(TIERS - 1) + 160), al, **T)
    sc.box((xl, y_b - 20, Z_PLINTH), (xr, y_b + 20, Z_PLINTH + 40), al, **T)
    zt = tier_z(TIERS - 1) + 120
    sc.bar((xl, y_b, Z_PLINTH + 40), (xr, y_b, zt), 30, 6, COL["alu_d"], up=(0, 1, 0), **T)
    sc.bar((xr, y_b, Z_PLINTH + 40), (xl, y_b, zt), 30, 6, COL["alu_d"], up=(0, 1, 0), **T)
    for k in range(TIERS):
        z = tier_z(k)
        for s in (-1, 1):
            xe = xc + s * (PROD_L / 2 - 12)
            x_out = xc + s * (PROD_L / 2 + 20)
            sc.box((min(xe, x_out), y_f + 10, z - 22), (max(xe, x_out), y_b - 10, z), COL["yellow"], tag="rack")
            lip = xc + s * (PROD_L / 2 + 4)
            sc.box((min(lip, lip + s * 16), y_f + 10, z), (max(lip, lip + s * 16), y_b - 10, z + 12),
                   COL["yellow"], tag="rack")
            sc.box((min(xc + s * (PROD_L / 2 + 20), xc + s * (PROD_L / 2 + 40)), y_f + 10, z - 22),
                   (max(xc + s * (PROD_L / 2 + 20), xc + s * (PROD_L / 2 + 40)), y_b - 10, z - 2),
                   COL["alu_d"], tag="rack")
        col = label_col_fn(k)
        if col is not None:
            sc.box((xl - 24, y_f - 2, z - 18), (xl - 20, y_f + 30, z - 4), col, tag="rack")
    sc.box((xc - 160, y_f - 25, tier_z(TIERS - 1) + 170), (xc + 160, y_f - 20, tier_z(TIERS - 1) + 270),
           COL["led"], tag="rack")


def build_portal_camera(sc: Scene):
    """정밀 카메라 독립 강재 포털 (기둥은 광축 좌우 ±650) + POMEAS 광학 헤드"""
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
    sc.box((CX - 70, CY - 60, 1640), (CX + 70, CY - 45, zt - 80), st, **T)
    sc.box((CX - 40, CY - 45, 1660), (CX + 40, CY - 25, 1800), COL["alu_d"], **T)
    sc.box((CX - 45, CY - 40, 1760), (CX + 45, CY + 50, 1840), COL["cam"], tag="cam")
    sc.cyl((CX, CY, 1560), (CX, CY, 1760), 55, COL["cam"], n=36, tag="cam", spec=0.4)
    sc.cyl((CX, CY, 1500), (CX, CY, 1560), 45, COL["lens"], n=36, tag="cam", spec=0.6)
    sc.cyl((CX, CY, 1470), (CX, CY, 1500), 95, COL["cam"], n=40, tag="cam")
    sc.cyl((CX, CY, Z_LENS), (CX, CY, 1470), 90, COL["ringlight"], n=40, tag="led")
    sc.cone((CX, CY, Z_LENS - 1), (CX, CY, Z_FOCUS), 60, COL["beam"], alpha=0.18, tag="beam")


def jig_boxes():
    """지그 간섭 검사용 상자 [(xmin, xmax, ymin, ymax, ztop)]: 양 끝 받침 2개 + XY 스테이지"""
    out = []
    for s in (-1, 1):
        xa, xb = sorted((CX + s * (PROD_L / 2 - 14), CX + s * (PROD_L / 2 + 26)))
        out.append((xa, xb, CY - 45.0, CY + 45.0, Z_JIG + 12.0))
    out.append((CX - 300.0, CX + 300.0, CY - 120.0, CY + 120.0, Z_JIG - 90.0))
    return out


def build_jig(sc: Scene):
    """정렬·고정 지그 (XY 서보 스테이지 위, 제품 양 끝 받침 → 중앙 하부 개방)"""
    T = dict(tag="jig", spec=0.35)
    y0, y1 = CY - 120, CY + 120
    zs = Z_JIG - 170.0                                     # 스테이지 하판 (V4: 1080)
    sc.box((CX - 90, y0 + 30, Z_PLINTH), (CX + 90, y1 - 30, zs), COL["alu"], **T)
    sc.box((CX - 260, y0, zs), (CX + 260, y1, zs + 20), COL["steel"], **T)
    sc.box((CX - 250, y0 + 20, zs + 20), (CX + 250, y1 - 20, zs + 50), COL["alu_d"], **T)
    sc.box((CX - 230, y0 + 30, zs + 50), (CX + 230, y1 - 30, zs + 80), COL["alu"], **T)
    for s in (-1, 1):
        x_in = CX + s * (PROD_L / 2 - 14)
        x_out = CX + s * (PROD_L / 2 + 26)
        sc.box((min(x_in, x_out), CY - 45, zs + 80), (max(x_in, x_out), CY + 45, Z_JIG), COL["grip_j"], **T)
        sc.box((min(x_out - s * 16, x_out), CY - 45, Z_JIG), (max(x_out - s * 16, x_out), CY + 45, Z_JIG + 12),
               COL["yellow"], tag="jig")
        sc.cyl((x_in + s * 10, CY + 38, Z_JIG), (x_in + s * 10, CY + 38, Z_JIG + 8), 4, COL["bolt"], n=10, tag="jig")
    sc.box((CX + 250, y0 + 20, zs + 20), (CX + 300, y1 - 20, zs + 80), COL["cam"], tag="jig")


def build_base_frame(sc: Scene):
    """오픈형 (V7): 밀폐 외함 대신 설비 베이스(플린스 스커트·레벨링 풋·상부 테두리)만 — 지붕·헤더·유리 도어 없음"""
    W = COL["white"]
    T = dict(tag="enc", spec=0.3)
    x0 = ENC_X0
    for x in (x0 + 60, -900.0, 300.0, CAB_X0, CAB_X1 - 60):
        for y in (-ENC_Y + 60, ENC_Y - 60):
            sc.cyl((x, y, 0), (x, y, 14), 38, COL["chain"], n=16, tag="enc")
            sc.cyl((x, y, 14), (x, y, 80), 11, COL["bolt"], n=12, tag="enc")
    sc.box((x0, -ENC_Y, 80), (ENC_X1, -ENC_Y + 20, Z_PLINTH), W, **T)
    sc.box((x0, ENC_Y - 20, 80), (ENC_X1, ENC_Y, Z_PLINTH), W, **T)
    for x in (x0, ENC_X1 - 20):
        sc.box((x, -ENC_Y, 80), (x + 20, ENC_Y, Z_PLINTH), W, **T)
    sc.box((x0 + 40, -ENC_Y - 3, 110), (ENC_X1 - 40, -ENC_Y, 118), COL["led"], tag="led")
    for y in (-ENC_Y, ENC_Y - 40):
        sc.box((x0, y, Z_PLINTH), (ENC_X1, y + 40, Z_PLINTH + 30), W, **T)
    for x in (x0, ENC_X1 - 40):
        sc.box((x, -ENC_Y, Z_PLINTH), (x + 40, ENC_Y, Z_PLINTH + 30), W, **T)


def wire_grid(segs, o, e, w, z0, z1, pitch=MESH_PITCH):
    """철망: 원점 o에서 수평 방향 e로 폭 w, 높이 z0~z1 평면에 피치 pitch 격자 선분 추가"""
    o = np.asarray(o, float)
    e = nrm(e)
    nx = max(1, int(round(w / pitch)))
    nz = max(1, int(round((z1 - z0) / pitch)))
    for k in range(1, nx):
        p = o + e * (w * k / nx)
        segs.append((p + (0, 0, z0), p + (0, 0, z1)))
    for k in range(1, nz):
        z = z0 + (z1 - z0) * k / nz
        segs.append((o + (0, 0, z), o + e * w + (0, 0, z)))


def lines_poly(segs):
    pts = np.array([p for s in segs for p in s], float)
    n = len(segs)
    lines = np.c_[np.full(n, 2), 2 * np.arange(n), 2 * np.arange(n) + 1].ravel()
    return pv.PolyData(pts, lines=lines)


def fence_run(sc: Scene, segs, p0, p1):
    """고정 철망 펜스 1열: 기둥(50×50) + 상·중·하 프레임 + 철망, 바닥 고정판"""
    F = COL["fence"]
    T = dict(tag="cover", spec=0.35)
    p0 = np.array([p0[0], p0[1], 0.0])
    p1 = np.array([p1[0], p1[1], 0.0])
    L = float(np.linalg.norm(p1 - p0))
    e = nrm(p1 - p0)
    n = max(1, math.ceil(L / FENCE_PANEL))
    posts = [p0 + e * (L * k / n) for k in range(n + 1)]
    for p in posts:
        sc.box(p + (-25, -25, 8), p + (25, 25, FENCE_H), F, **T)
        sc.box(p + (-65, -65, 0), p + (65, 65, 8), F, **T)
    for a, b in zip(posts, posts[1:]):
        a2, b2 = a + e * 25, b - e * 25
        for z, t in ((FENCE_H - 20, 40), (1000.0, 30), (120.0, 30)):
            sc.bar(a2 + (0, 0, z), b2 + (0, 0, z), 30, t, F, **T)
        w = float(np.linalg.norm(b2 - a2))
        wire_grid(segs, a2, e, w, 135.0, 985.0)
        wire_grid(segs, a2, e, w, 1015.0, FENCE_H - 40)


def build_fence(sc: Scene):
    """오픈형 철망 안전펜스 (V7): 설비 외곽 FENCE_OFF 바깥, 우측은 전장함이 펜스 역할 → 철망 선분 반환"""
    segs = []
    runs = [((FX0, FY0), (DOOR_LP, FY0)),                  # 전면 좌 (자동문 좌측 고정부 = 도어 수납부)
            ((DOOR_RP, FY0), (FX1, FY0)),                  # 전면 우
            ((FX1, FY0), (FX1, -ENC_Y - 25.0)),            # 우측 짧은 구간 (전장함 앞쪽 모서리까지)
            ((FX1, ENC_Y + 25.0), (FX1, FY1)),             # 우측 짧은 구간 (전장함 뒤쪽)
            ((FX1, FY1), (FX0, FY1)),                      # 후면
            ((FX0, FY1), (FX0, FY0))]                      # 좌측
    for p0, p1 in runs:
        fence_run(sc, segs, p0, p1)
    # 바닥 도어 레일 (수납 위치 ~ 중앙, 안쪽·바깥쪽 2열)
    xa = door_leaf_x0(-1, 0, 1.0) - 20
    xb = door_leaf_x0(+1, 0, 1.0) + DOOR_W + 20
    for yt in DOOR_TRACK:
        sc.box((xa, yt - 20, 0), (xb, yt + 20, 6), COL["alu_d"], tag="door", spec=0.5)
    return segs


def build_door_static(sc: Scene):
    """양개 전동 자동문 고정부: 노란 구동 기둥 2개 + 상부 빔·간판 + 펜스 연결 브래킷 + 바닥 경고 줄무늬"""
    Y = COL["yellow"]
    T = dict(tag="door", spec=0.4)
    y0, y1 = POST_Y
    for xp in (DOOR_LP, DOOR_RP):
        sc.box((xp - 110, y0 - 30, 0), (xp + 110, y1 + 30, 10), COL["steel"], tag="door")           # 고정판
        for dx in (-80, 80):
            sc.hexbolt((xp + dx, (y0 + y1) / 2, 10), s=9, h=8, tag="door")
        sc.box((xp - 60, y0, 10), (xp + 60, y1, POST_H), Y, **T)                                   # 구동 기둥
        sc.box((xp - 45, y0 - 55, 90), (xp + 45, y0, 380), COL["cam"], tag="door")                # 구동 모터·감속기
        sc.box((xp - 62, y0 - 2, 1450), (xp + 62, y0, 1700), COL["chain"], tag="door")            # 라벨판
        sc.box((xp - 40, y1, FENCE_H + 10), (xp + 40, FY0 - 25, FENCE_H + 60), COL["fence"], tag="door")  # 펜스 연결
        for zz in (600.0, 1300.0):                                                                # 도어 가이드 롤러
            sc.box((xp - 20, y1, zz), (xp + 20, DOOR_TRACK[0] + 18, zz + 30), COL["alu_d"], tag="door")
    xa, xb = DOOR_LP - 60, DOOR_RP + 60
    sc.box((xa, y0, POST_H), (xb, y1, POST_H + 80), Y, **T)                                        # 상부 빔
    sc.box((xa, y0 + 10, POST_H + 80), (xb, y1 - 10, POST_H + 300), COL["navy"], tag="door", spec=0.5)  # 간판
    sc.box((DOOR_LP + 80, y0 + 6, POST_H + 92), (DOOR_RP - 80, y0 + 10, POST_H + 98), COL["led"], tag="led")
    # 바닥: 도어 앞 황흑 경고 줄무늬
    ya, yb = y0 - 210, y0 - 10
    xs0, xs1 = DOOR_LP - 250, DOOR_RP + 250
    sc.poly([(xs0, ya, 2.5), (xs1, ya, 2.5), (xs1, yb, 2.5), (xs0, yb, 2.5)], COL["chain"], edges=False, tag="door")
    for x in np.arange(xs0, xs1 - 200, 160.0):
        sc.poly([(x, ya, 3.2), (x + 80, ya, 3.2), (x + 80 + (yb - ya), yb, 3.2), (x + (yb - ya), yb, 3.2)],
                COL["fyellow"], edges=False, tag="door")


def door_leaf_scene(lead):
    """도어 1장 (로컬: X 0~DOOR_W, Y 레일 중심, Z 바닥 기준).  lead +1/−1 = 오른쪽/왼쪽 선단에 고무 안전엣지, 0 없음
    → (Scene, 철망 선분)"""
    sc = Scene()
    segs = []
    F = COL["fence"]
    T = dict(tag="leaf", spec=0.35)
    W = DOOR_W
    z0 = 55.0
    for xa in (0.0, W - 40):
        sc.box((xa, -15, z0), (xa + 40, 15, FENCE_H), F, **T)                       # 세로 프레임
    for za, zb in ((FENCE_H - 40, FENCE_H), (1000.0, 1030.0), (z0, z0 + 70)):
        sc.box((40, -15, za), (W - 40, 15, zb), F, **T)                             # 가로 프레임
    wire_grid(segs, (40, 0, 0), (1, 0, 0), W - 80, z0 + 70, 1000.0)
    wire_grid(segs, (40, 0, 0), (1, 0, 0), W - 80, 1030.0, FENCE_H - 40)
    for xw in (70.0, W - 70):                                                        # 바닥 주행 바퀴
        sc.cyl((xw, -12, 28), (xw, 12, 28), 22, COL["chain"], n=16, tag="leaf")
        sc.box((xw - 30, -16, 45), (xw + 30, 16, z0), COL["alu_d"], tag="leaf")
    if lead:
        xr = W if lead > 0 else -12.0
        sc.box((xr, -12, z0 + 10), (xr + 12, 12, FENCE_H - 10), COL["rubber"], tag="leaf")   # 안전엣지
        xh = W - 90 if lead > 0 else 60.0
        sc.box((xh, -40, 950), (xh + 30, -15, 1150), COL["alu"], tag="leaf", spec=0.6)        # 손잡이
    return sc, segs


def build_cabinet(sc: Scene):
    W = COL["white"]
    T = dict(tag="cab", spec=0.3)
    x0, x1 = CAB_X0, CAB_X1
    sc.box((x0, -ENC_Y, 80), (x1, ENC_Y, Z_TOP), W, **T)
    sc.box((x0 + 10, -ENC_Y - 8, Z_HEADER), (x1 - 10, -ENC_Y, Z_TOP - 10), COL["navy"], tag="cab")
    sc.box((x0 + 30, -ENC_Y - 4, 260), (x1 - 30, -ENC_Y, 1000), COL["white_d"], tag="cab")
    sc.box((x1 - 70, -ENC_Y - 20, 560), (x1 - 55, -ENC_Y - 4, 760), COL["chain"], tag="cab")
    sc.box((x0 + 70, -ENC_Y - 25, 1320), (x1 - 70, -ENC_Y, 1700), COL["chain"], tag="cab")
    sc.box((x0 + 85, -ENC_Y - 27, 1335), (x1 - 85, -ENC_Y - 25, 1685), COL["screen"], tag="led")
    zb = 1180.0
    for i, col in enumerate((COL["ok"], COL["ok"], COL["ng"])):
        x = x0 + 120 + i * 90
        sc.cyl((x, -ENC_Y, zb), (x, -ENC_Y - 25, zb), 22, col, n=20, tag="cab", spec=0.5)
    sc.cyl((x1 - 120, -ENC_Y, zb), (x1 - 120, -ENC_Y - 15, zb), 42, COL["yellow"], n=24, tag="cab")
    sc.cyl((x1 - 120, -ENC_Y - 15, zb), (x1 - 120, -ENC_Y - 45, zb), 32, COL["ng"], n=24, tag="cab", spec=0.5)
    sc.box((x0 + 150, -ENC_Y - 6, 1060), (x1 - 150, -ENC_Y, 1090), COL["chain"], tag="cab")
    xs, ys = (x0 + x1) / 2 - 150, -300.0
    sc.cyl((xs, ys, Z_TOP), (xs, ys, Z_TOP + 120), 14, COL["bolt"], n=12, tag="cab")
    z = Z_TOP + 120
    for col in (COL["ok"], COL["yellow"], COL["ng"]):
        sc.cyl((xs, ys, z), (xs, ys, z + 80), 40, col, n=24, tag="led")
        z += 80
    sc.cyl((xs, ys, z), (xs, ys, z + 20), 42, COL["white_d"], n=24, tag="cab")


def add_safety_line(sc, x0, x1, y0, y1, w=80.0, tag="floor"):
    yl = COL["fyellow"]
    for (a, b, c, d) in (((x0, y0), (x1, y0), (x1, y0 + w), (x0, y0 + w)),
                         ((x0, y1 - w), (x1, y1 - w), (x1, y1), (x0, y1)),
                         ((x0, y0), (x0 + w, y0), (x0 + w, y1), (x0, y1)),
                         ((x1 - w, y0), (x1, y0), (x1, y1), (x1 - w, y1))):
        sc.poly([(*a, 3), (*b, 3), (*c, 3), (*d, 3)], yl, edges=False, tag=tag)


def build_floor_patch(sc: Scene):
    m = 900.0
    sc.poly([(ENC_X0 - m, -ENC_Y - m, 0), (CAB_X1 + m, -ENC_Y - m, 0),
             (CAB_X1 + m, ENC_Y + m, 0), (ENC_X0 - m, ENC_Y + m, 0)], COL["floor"], edges=False, tag="floor")
    add_safety_line(sc, FX0 - 400, CAB_X1 + 400, FY0 - 450, FY1 + 400, tag="floor")


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
    add_safety_line(sc, FX0 - 400, CAB_X1 + 400, FY0 - 450, FY1 + 400, tag="fac")
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
    yc = POST_Y[0] + 7                                  # 자동문 간판 전면
    xm = (DOOR_LP + DOOR_RP) / 2
    items = [("BSV Slit Valve O-ring Inspection", 70, (xm, yc, POST_H + 230)),
             ("AUTO DOOR  -  KEEP CLEAR", 34, (xm, yc, POST_H + 140)),
             ("EXCELSIOR", 70, ((CAB_X0 + CAB_X1) / 2, -ENC_Y - 12, Z_HEADER + 120)),
             ("INPUT RACK (10)", 48, (XI, RACK_Y - 104, tier_z(TIERS - 1) + 220)),
             ("OUTPUT RACK (OK/NG 10)", 40, (XO, RACK_Y - 104, tier_z(TIERS - 1) + 220)),
             ("POMEAS", 30, (CX, CY - 58, 1700))]
    for s, h, c in items:
        try:
            m = pv.Text3D(s, depth=3, height=h, center=c, normal=(0, -1, 0))
            pl.add_mesh(m, color="white", ambient=0.6, specular=0.2)
        except Exception:
            pass


def draw_carriage(sc, xc):
    for dx in (-90, 90):
        for s in (-1, 1):
            sc.box((xc + dx - 45, RY + s * 75 - 28, Z_GUIDE[1] - 5),
                   (xc + dx + 45, RY + s * 75 + 28, Z_CARR[0] + 18), COL["rail"], tag="dyn")
    sc.box((xc - 150, RY - 150, Z_CARR[0] + 18), (xc + 150, RY + 150, Z_CARR[1]), COL["alu_d"], spec=0.3, tag="dyn")
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


# ===========================================================================
# 경로 검증 지표 (V1 확장): 초점 정합, IK, 관절 한계, 렌즈·지그 간섭
# ===========================================================================
def collision_points(plan: Plan):
    """간섭 검사용 점 (각자 로컬 좌표): 제품, 툴(그립 닫힘), 로봇 링크별"""
    prod = np.vstack([np.asarray(m.points) for _, m, _ in product_meshes()])[::3]
    tool = [np.asarray(m.points) for _, m, _ in tool_meshes()]
    off = PROD_W / 2 + 6
    for s in (-1, 1):
        tool.append(np.asarray(jaw_mesh().points) + (0, L_FORK + s * off, 0))
    tool = np.vstack(tool)
    rob = {}
    for k, m, _ in robot_meshes(plan.Q[0]):
        rob.setdefault(k, []).append(np.asarray(m.points)[::4])
    rob = {k: np.vstack(v) for k, v in rob.items()}
    return prod, tool, rob


def evaluate(plan: Plan):
    """샘플별 지표 계산 → plan.metrics (검사 구간 1회분 + 전체)"""
    prod, tool, rob = collision_points(plan)
    g = plan.gamma
    Mfp = M_F_P(g)
    Mtool = H(Rz(g))
    boxes = jig_boxes()
    n = len(plan.T)
    lens_clr = np.full(n, np.inf)
    jig_clr = np.full(n, np.inf)
    xy_off = np.full(n, np.nan)
    wd_now = np.full(n, np.nan)
    low_z = np.full(n, np.inf)
    rail = np.full(n, np.inf)
    first = np.where(plan.INS >= 0)[0]
    first_block = first[: len(plan.frames)] if len(first) else first
    # 기울임 구간 (검사 자세 진입 ~ 판정 복귀, 제품 1번째) 전체 평가
    tb = next(t for t, kd, _ in plan.events if kd == "tilt_begin")
    te = next(t for t, kd, _ in plan.events if kd == "tilt_end")
    eval_idx = set(np.where((plan.T >= tb - 1e-9) & (plan.T <= te + 1e-9))[0].tolist())
    for i in range(n):
        Fs = fk(plan.Q[i])
        B = base_pose(plan.XC[i])
        low_z[i] = min((B @ F)[2, 3] for F in Fs[2:])
        if i % 2 == 0:
            rail[i] = rail_clear(plan.Q[i], plan.XC[i], g)
        if i not in eval_idx:
            continue
        F6w = B @ Fs[6]
        Mp = F6w @ Mfp
        Mt = F6w @ Mtool
        pw = (Mp[:3, :3] @ prod.T).T + Mp[:3, 3]
        tw = (Mt[:3, :3] @ tool.T).T + Mt[:3, 3]
        rw = [((B @ Fs[k])[:3, :3] @ P.T).T + (B @ Fs[k])[:3, 3] for k, P in rob.items()]
        W = np.vstack([pw, tw] + rw)
        r = np.hypot(W[:, 0] - CX, W[:, 1] - CY)
        inside = r < LENS_R
        if inside.any():
            lens_clr[i] = float((Z_LENS - W[inside, 2]).min())
        for x0, x1, y0, y1, zt in boxes:
            inj = (W[:, 0] > x0) & (W[:, 0] < x1) & (W[:, 1] > y0) & (W[:, 1] < y1)
            if inj.any():
                jig_clr[i] = min(jig_clr[i], float((W[inj, 2] - zt).min()))
        if plan.INS[i] >= 0:
            f = plan.frames[plan.INS[i]]
            pt = (Mp @ np.r_[f["P"], 0.0, 1.0])[:3]
            xy_off[i] = math.hypot(pt[0] - Q[0], pt[1] - Q[1])
            wd_now[i] = Z_LENS - pt[2]
    # ---- 자기간섭 (V6): 포크 끝부(암·판·카메라·조) + 파지 중 제품 vs 로봇 링크 0~4 (베이스~J5 하우징) ----
    #   메쉬 꼭짓점 사이 최소 거리 (근사값). 플랜지 바로 옆 J6 링크·툴 몸체는 붙어 있으므로 제외
    fork_pts = [np.asarray(m.points) for nm, m, _ in tool_meshes() if nm in ("arm", "plate", "forkcam")]
    for s in (-1, 1):
        fork_pts.append(np.asarray(jaw_mesh().points) + (0, L_FORK + s * (PROD_W / 2 + 6), 0))
    fork_pts = np.vstack(fork_pts)
    rob04 = {k: P[::2] for k, P in rob.items() if k <= 4}
    self_clr = np.full(n, np.inf)
    for i in range(0, n, 3):
        Fs = fk(plan.Q[i])
        B = base_pose(plan.XC[i])
        F6w = B @ Fs[6]
        Mt = F6w @ Mtool
        A = [(Mt[:3, :3] @ fork_pts.T).T + Mt[:3, 3]]
        if plan.G[i] > 0.5:
            Mp = F6w @ Mfp
            A.append((Mp[:3, :3] @ prod.T).T + Mp[:3, 3])
        tree = cKDTree(np.vstack(A))
        for k, P in rob04.items():
            M = B @ Fs[k]
            d, _ = tree.query((M[:3, :3] @ P.T).T + M[:3, 3])
            self_clr[i] = min(self_clr[i], float(d.min()))
    # ---- 관절·제7축 속도 (V6) ----
    dt = np.diff(plan.T)
    ok = dt > 1e-9
    vq = np.zeros((n, 6))
    vx = np.zeros(n)
    vq[1:][ok] = np.abs(np.diff(plan.Q, axis=0))[ok] / dt[ok, None]
    vx[1:][ok] = np.abs(np.diff(plan.XC))[ok] / dt[ok]
    plan.metrics = dict(lens=lens_clr, jig=jig_clr, xy=xy_off, wd=wd_now, low=low_z, rail=rail, block=first_block,
                        self=self_clr, vq=vq, vx=vx)
    return plan.metrics


def summarize(plan: Plan):
    mt = plan.metrics
    blk = mt["block"]
    q = plan.Q / D2R
    ins_mask = np.array([plan.frames[i]["inspect"] for i in plan.INS[blk]]) if len(blk) else np.array([])
    print("\n=================== 경로 검증 요약 (V7) ===================")
    print(f"  사이클 시간          : {plan.total:.1f} s (제품 {TIERS}개, 제품 1개 평균 {plan.total / TIERS:.1f} s, 개념값)")
    print(f"  샘플 수              : {len(plan.T)}  /  검사 구간 {len(plan.frames)}프레임 "
          f"(외측 {plan.n_out} · 전환 {TRANS_N} · 내측 {plan.n_in})")
    for lp in plan.loops:
        print(f"  {lp['name']:5s} 윤곽          : 둘레 {lp['perim']:.1f} mm, 검사점 {len(lp['P'])}개, "
              f"ψ {lp['psi'][0] / D2R:.0f}° → {lp['psi'][-1] / D2R:.0f}°")
    print(f"  그리퍼 장착각 γ      : {plan.gamma / D2R:.1f}°")
    print(f"  최대 |Tᵢ+RᵢPᵢ−Q|      : {max(f['check'] for f in plan.frames):.2e} mm (설계식 검산)")
    if len(blk):
        xy = mt["xy"][blk][ins_mask]
        wd = mt["wd"][blk][ins_mask]
        print(f"  FK 검산 검사점→Q XY  : 최대 {np.nanmax(xy):.2e} mm")
        print(f"  FK 검산 작업거리 WD  : {np.nanmin(wd):.4f} ~ {np.nanmax(wd):.4f} mm (목표 {WD:.1f})")
        dq = np.abs(np.diff(plan.Q[blk], axis=0)).max(axis=0) / D2R
        print("  검사 중 샘플간 최대 관절변화 [deg]: " + " ".join(f"{v:.1f}" for v in dq))
    print(f"  최대 IK 오차         : {plan.max_err:.2e}")
    for j in range(6):
        over = np.abs(q[:, j]).max() > JOINT_LIM / D2R + 1e-6
        print(f"  J{j + 1} 범위 [deg]      : {q[:, j].min():8.1f} ~ {q[:, j].max():8.1f}" + ("   ← 한계 초과" if over else ""))
    print(f"  제7축 범위 [mm]      : {plan.XC.min():.0f} ~ {plan.XC.max():.0f} (스트로크 {CARR_MIN:.0f} ~ {CARR_MAX:.0f})")
    lc = mt["lens"][np.isfinite(mt["lens"])]
    jc = mt["jig"][np.isfinite(mt["jig"])]
    if len(lc):
        print(f"  렌즈 최소 여유       : {lc.min():.1f} mm (반경 {LENS_R:.0f} 원통, 로봇+툴+제품)"
              + ("  (간섭!)" if lc.min() < 0 else ""))
    if len(jc):
        print(f"  지그 최소 여유       : {jc.min():.1f} mm (기울임 구간, 로봇+툴+제품)"
              + ("  (간섭!)" if jc.min() < 0 else ""))
    rc = mt["rail"][np.isfinite(mt["rail"])]
    if len(rc):
        i = int(np.nanargmin(np.where(np.isfinite(mt["rail"]), mt["rail"], np.inf)))
        print(f"  제7축 주변 최소 여유 : {rc.min():.1f} mm (캐리지·레일·케이블체인 vs 로봇+툴, @ {plan.LAB[i]})"
              + ("  (간섭!)" if rc.min() < 0 else ""))
    sc_ = mt["self"]
    if np.isfinite(sc_).any():
        i = int(np.argmin(sc_))
        print(f"  자기간섭 최소 거리   : {sc_[i]:.1f} mm (포크·파지 제품 vs 로봇 링크, 근사, @ {plan.LAB[i]})"
              + ("  (확인 필요!)" if sc_[i] < SELF_CLEAR_WARN else ""))
    print(f"  관절 원점 최저 높이  : {mt['low'].min():.1f} mm" + ("  (플린스 간섭 확인!)" if mt['low'].min() < Z_PLINTH else ""))
    vmax = mt["vq"].max(axis=0) / D2R
    over = vmax > JOINT_VMAX / D2R + 0.5
    print("  관절 최대 속도 [deg/s]: " + " ".join(f"J{j + 1} {v:.0f}" + ("!" if o else "") for j, (v, o) in enumerate(zip(vmax, over)))
          + f"  (한계 가정 {JOINT_VMAX[0] / D2R:.0f})")
    print(f"  제7축 최대 속도      : {mt['vx'].max():.0f} mm/s (한계 가정 {RAIL_VMAX:.0f})")
    # ---- 자동문 인터록 (V7): 로봇·제7축이 움직이는 샘플 구간에서 문이 조금이라도 열려 있으면 위반 ----
    du = np.array([plan.door_u(t) for t in plan.T])
    dm = np.array([plan.door_u(t) for t in (plan.T[1:] + plan.T[:-1]) / 2])
    moving = (np.abs(np.diff(plan.Q, axis=0)).max(axis=1) > 1e-9) | (np.abs(np.diff(plan.XC)) > 1e-9)
    bad = moving & (np.maximum(np.maximum(du[1:], du[:-1]), dm) > 1e-3)
    t_closed = float(np.sum(np.diff(plan.T)[np.maximum(du[1:], du[:-1]) <= 1e-3]))
    print(f"  자동문 인터록        : 로봇 이동 중 문 열림 {int(bad.sum())}건"
          + ("  (위반!)" if bad.any() else " (OK)")
          + f"  /  개폐 {DOOR_T:.1f} s, 닫힘 운전 {t_closed:.0f} s, 도어 1장 폭 {DOOR_W:.1f} mm")
    if len(blk):
        t_insp = plan.T[blk[-1]] - plan.T[blk[0]]
        print(f"  윤곽 검사 시간       : {t_insp:.1f} s (제품 1개, 코너 감속 반영)")
    if plan.warn:
        print(f"  경고 {len(plan.warn)}건 (처음 5건):")
        for w in plan.warn[:5]:
            print("    -", w)
    print("============================================================\n")


def export_csv(plan: Plan, path):
    mt = plan.metrics
    Mfp = M_F_P(plan.gamma)
    with open(path, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        w.writerow(["sample", "time_s", "step", "loop", "inspect", "idx", "Px", "Py", "psi_deg",
                    "Tx", "Ty", "Tz", "carriage_x", "q1", "q2", "q3", "q4", "q5", "q6", "grip",
                    "flange_x", "flange_y", "flange_z", "flange_rx", "flange_ry", "flange_rz",
                    "product_x", "product_y", "product_z", "xy_offset_mm", "wd_mm", "lens_clear_mm", "door_open"])
        for i in range(len(plan.T)):
            F6w = base_pose(plan.XC[i]) @ fk_flange(plan.Q[i])
            Mp = F6w @ Mfp
            rv = Rotation.from_matrix(F6w[:3, :3]).as_rotvec()
            k = plan.INS[i]
            if k >= 0:
                f = plan.frames[k]
                ins = [f["loop"], int(f["inspect"]), f["idx"], *np.round(f["P"], 4), round(f["psi"] / D2R, 4),
                       *np.round(f["T"], 4)]
            else:
                ins = ["", "", "", "", "", "", "", "", ""]

            def fmt(v, d=4):
                return round(float(v), d) if np.isfinite(v) else ""

            w.writerow([i, round(plan.T[i], 4), plan.LAB[i], *ins, round(plan.XC[i], 3),
                        *np.round(plan.Q[i] / D2R, 4), round(plan.G[i], 3),
                        *np.round(F6w[:3, 3], 4), *np.round(rv, 6), *np.round(Mp[:3, 3], 4),
                        fmt(mt["xy"][i], 6), fmt(mt["wd"][i]), fmt(mt["lens"][i], 2),
                        round(plan.door_u(plan.T[i]), 4)])
    print(f"CSV 저장: {path}")


# ===========================================================================
# 시뮬레이터
# ===========================================================================
def set_text(actor, s, corner=2):
    """corner: 코너 주석일 때 위치 (0 왼쪽 아래, 2 왼쪽 위)"""
    s = s.replace("|", "/")  # VTK: '|'가 있으면 지정 글꼴 대신 기본 글꼴로 그려져 한글이 깨짐
    if hasattr(actor, "SetInput"):
        actor.SetInput(s)
    elif hasattr(actor, "SetText"):
        actor.SetText(corner, s)


class CellSim:
    def __init__(self, pl, plan: Plan, factory=True, font=None):
        self.pl = pl
        self.plan = plan
        self.font = font
        self.ko = plan.ko
        self.t = 0.0
        self.speed = 1.0
        self.playing = True
        self.last = time.perf_counter()
        self.dyn_actors: dict = {}
        self.cover_actors, self.factory_actors, self.patch_actors = [], [], []
        self.show_cover, self.show_factory = True, factory
        self._slider_lock = False
        self._ready = False
        self.Mfp = M_F_P(plan.gamma)
        self.Mtool = H(Rz(plan.gamma))

        pl.subplot(0, 0)
        self._build_static(factory)
        self._build_dynamic_actors()
        self._add_overlay()
        pl.subplot(0, 1)
        self._build_camera_view()
        pl.subplot(0, 0)
        self._reset_state()
        self._ready = True
        self.update(force=True)

    # ---------------- 상태 ----------------
    def _reset_state(self):
        self.inp = set(range(TIERS))
        self.out: dict[int, str] = {}
        self.carry = False
        self.on_jig = False
        self.ev_i = 0
        self.flash_until = -1.0
        self.n_ok = self.n_ng = 0
        self.judge = None

    def _apply_events(self, t):
        ev = self.plan.events
        while self.ev_i < len(ev) and ev[self.ev_i][0] <= t + 1e-9:
            te, kind, arg = ev[self.ev_i]
            if kind == "pick_in":
                self.inp.discard(arg)
                self.carry = True
                self.judge = None
            elif kind == "to_jig":
                self.carry, self.on_jig = False, True
            elif kind == "from_jig":
                self.carry, self.on_jig = True, False
            elif kind == "judge":
                self.judge = arg
            elif kind == "place_out":
                self.carry = False
                self.out[arg[0]] = arg[1]
                if arg[1] == "OK":
                    self.n_ok += 1
                else:
                    self.n_ng += 1
            elif kind == "sn":
                self.flash_until = te + 0.9
            self.ev_i += 1

    # ---------------- 장면 ----------------
    def _build_static(self, factory):
        pl = self.pl
        sc = Scene()
        build_steel_base(sc)
        build_rail_static(sc)
        build_rack(sc, XI, lambda k: None)
        build_rack(sc, XO, lambda k: COL["ng"] if k in NG_TIERS else COL["ok"])
        build_portal_camera(sc)
        build_jig(sc)
        build_base_frame(sc)
        segs = build_fence(sc)
        build_door_static(sc)
        build_cabinet(sc)
        for key, m in scene_to_groups(sc, excl=("cover",)).items():
            add_group(pl, key, m)
        for key, m in scene_to_groups(sc, tags={"cover"}).items():
            self.cover_actors.append(add_group(pl, key, m))
        # 철망 (선으로 표시 → 화면 크기와 무관하게 1 px, 내부가 비쳐 보임)
        self.cover_actors.append(pl.add_mesh(lines_poly(segs), color=COL["mesh"], line_width=1))
        add_header_text(pl)
        sf = Scene()
        build_factory(sf)
        for key, m in scene_to_groups(sf).items():
            a = add_group(pl, key, m)
            a.SetVisibility(factory)
            self.factory_actors.append(a)
        sp = Scene()
        build_floor_patch(sp)
        for key, m in scene_to_groups(sp).items():
            a = add_group(pl, key, m)
            a.SetVisibility(not factory)
            self.patch_actors.append(a)
        # 촬영점 Q 표시
        pl.add_mesh(pv.Sphere(radius=5, center=Q), color=COL["ng"])

    def _prod_set(self, meshes, cam=False):
        acts = []
        for name, m, c in meshes:
            acts.append(self.pl.add_mesh(m, color=c, smooth_shading=True,
                                         specular=0.5 if name == "body" else 0.2))
        return acts

    def _build_dynamic_actors(self):
        pl = self.pl
        plan = self.plan
        # 캐리지 (xc=0 기준 형상 + 평행이동)
        sc = Scene()
        draw_carriage(sc, 0.0)
        self.carr_actors = [add_group(pl, k, m) for k, m in scene_to_groups(sc).items()]
        # 로봇
        self.robot_actors = []
        for k, m, c in robot_meshes(plan.Q[0]):
            a = pl.add_mesh(m, color=c, smooth_shading=True, specular=0.4 if c == COL["robot"] else 0.3)
            self.robot_actors.append((a, k))
        # 툴
        self.tool_meshes = tool_meshes()
        self.tool_actors = [pl.add_mesh(m, color=c, smooth_shading=True, specular=0.35)
                            for _, m, c in self.tool_meshes]
        self.jaw = jaw_mesh()
        self.jaw_actors = [pl.add_mesh(self.jaw, color=COL["grip"], specular=0.3) for _ in range(2)]
        ring = cyl_mesh((0, L_FORK, H_TOOL - PROD_T - 0.5), (0, 0, 1), 17, 1.5)
        self.ring_actor = pl.add_mesh(ring, color=COL["ringlight"], ambient=0.6)
        # 제품 풀 (입력 10 · 출력 10 · 지그 1 · 파지 1) — 형상 공유
        self.pmesh = product_meshes()
        self.p_in = [self._prod_set(self.pmesh) for _ in range(TIERS)]
        self.p_out = [self._prod_set(self.pmesh) for _ in range(TIERS)]
        self.p_jig = self._prod_set(self.pmesh)
        self.p_carry = self._prod_set(self.pmesh)
        for k in range(TIERS):
            self._place(self.p_in[k], prod_pose((XI, RACK_Y, tier_z(k) + PROD_T)))
            self._place(self.p_out[k], prod_pose((XO, RACK_Y, tier_z(k) + PROD_T)))
        self._place(self.p_jig, prod_pose((CX, CY, Z_JIG + PROD_T)))
        # 검사 진행 표시선 (완료 = 초록 / 미완료 = 회색), O-ring 바로 위
        self.prog_lines = []
        for lp in plan.loops:
            P = np.c_[lp["P"], np.full(len(lp["P"]), 2.9)]
            line = pv.lines_from_points(np.r_[P, P[:1]])
            line.point_data["done"] = np.zeros(line.n_points)
            self.prog_lines.append(line)
        self.prog_actors = [pl.add_mesh(line, scalars="done", cmap=[hexc(COL["todo"]), hexc(COL["done"])], clim=[0, 1],
                                        line_width=4, render_lines_as_tubes=True, show_scalar_bar=False)
                            for line in self.prog_lines]
        # 플랜지 궤적 표시 (검사 구간 1회분)
        blk = np.where(plan.INS >= 0)[0][: len(plan.frames)]
        if len(blk):
            pts = np.array([(base_pose(plan.XC[i]) @ fk_flange(plan.Q[i]) @ self.Mtool)[:3, 3] for i in blk])
            pl.add_mesh(pv.lines_from_points(pts), color=COL["tcp"], line_width=2, opacity=0.6)
        self.tcp_marker = pl.add_mesh(pv.Sphere(radius=7), color=COL["tcp"])
        # 자동문 도어 4장 (좌·우 × 안쪽·바깥쪽 레일), 바깥쪽 도어 선단에 안전엣지
        self.leaves = []
        for side in (-1, 1):
            for idx in (0, 1):
                lead = -side if idx == 1 else 0              # 좌측 바깥 도어 → 오른쪽 선단, 우측 → 왼쪽 선단
                lsc, lsegs = door_leaf_scene(lead)
                acts = [add_group(pl, k, m) for k, m in scene_to_groups(lsc).items()]
                acts.append(pl.add_mesh(lines_poly(lsegs), color=COL["mesh"], line_width=1))
                self.leaves.append((side, idx, acts))
                self.cover_actors.extend(acts)
        # 상부 빔 상태등 (좌·우)
        self.door_lamps = []
        for xp in (DOOR_LP, DOOR_RP):
            yl = POST_Y[0]
            m = cyl_mesh((xp, yl - 12, POST_H + 40), (0, 1, 0), 30, 24)
            self.door_lamps.append(pl.add_mesh(m, color=COL["ok"], ambient=0.7, diffuse=0.4))

    def _place(self, acts, M, visible=True):
        for a in acts:
            a.user_matrix = M
            a.SetVisibility(visible)

    def _add_overlay(self):
        pl = self.pl
        kw = dict(font_size=11, color="#1d2733")
        if self.font:
            kw["font_file"] = self.font
        self.txt = pl.add_text(" ", position=(0.01, 0.90), viewport=True, **dict(kw, font_size=9))
        # 관절각·검사 수치 7줄: 왼쪽 아래(체크박스 위)로 이동 → 설비 가림 없음
        self.hud = pl.add_text(" ", position=(0.01, 0.12), viewport=True, font_size=8, color="#14213D",
                               font="courier")
        for a in (self.txt, self.hud):              # 배경 더 투명하게
            try:
                a.prop.background_color = "white"
                a.prop.background_opacity = 0.35
            except Exception:
                pass
        help_ko = ("Space 재생/정지  ·  ←→ 이동  ·  ↑↓ 속도  ·  k 펜스  ·  d 자동문 열림  ·  n 공장  ·  c 시점  ·  h 문자\n"
                   "6축 DH 역기구학(개념 DH값) — 실제 CS66 DH·충돌검사로 재확인 필요  ·  치수는 개념값")
        help_en = ("Space play/pause - <- -> step - Up/Down speed - k fence - d door open - n factory - c view - h text\n"
                   "6-axis DH IK with concept DH values - verify with real CS66 data")
        pl.add_text(help_ko if self.ko else help_en, position=(0.22, 0.012), viewport=True, **dict(kw, font_size=8))
        try:
            pl.add_checkbox_button_widget(lambda v: self.set_play(v), value=True, position=(10, 10), size=30,
                                          color_on="#2e8b57", color_off="#b03030")
            pl.add_checkbox_button_widget(lambda v: self.set_cover(v), value=True, position=(10, 48), size=30,
                                          color_on="#6fa8dc", color_off="#888888")
            pl.add_text("재생" if self.ko else "Play", position=(46, 16), **dict(kw, font_size=9))
            pl.add_text("펜스" if self.ko else "Fence", position=(46, 54), **dict(kw, font_size=9))
        except Exception:
            pass
        try:
            self.slider = pl.add_slider_widget(self._on_slider, [0.0, self.plan.total], value=0.0,
                                               title="t [s]", pointa=(0.25, 0.075), pointb=(0.75, 0.075),
                                               style="modern", interaction_event="always", fmt="%.1f",
                                               title_height=0.018, color="#14213D")
        except Exception:
            self.slider = None

    def _build_camera_view(self):
        """오른쪽: 고정 카메라 시점 (V1) — 파지 제품·포크만 표시"""
        pl = self.pl
        pl.set_background("#1C1F24", all_renderers=False)
        self.cam_prod = self._prod_set(self.pmesh)
        self.cam_tool = [pl.add_mesh(m, color=c, smooth_shading=True) for _, m, c in self.tool_meshes]
        self.cam_jaws = [pl.add_mesh(self.jaw, color=COL["grip"]) for _ in range(2)]
        self.cam_prog = [pl.add_mesh(line, scalars="done", cmap=[hexc(COL["todo"]), hexc(COL["done"])], clim=[0, 1],
                                     line_width=6, render_lines_as_tubes=True, show_scalar_bar=False)
                         for line in self.prog_lines]
        Lc = CAM_FOV * 0.15
        c0 = Q + np.array([0, 0, WD * 0.5])                   # 제품에 가려지지 않도록 렌즈 쪽에 표시
        for d in ((Lc, 0, 0), (0, Lc, 0)):
            pl.add_mesh(pv.Line(c0 - np.array(d), c0 + np.array(d)), color="#FF3B3B", line_width=2)
        pl.camera_position = [tuple(Q + [0, 0, WD]), tuple(Q), (0, 1, 0)]
        pl.camera.view_angle = 2 * math.degrees(math.atan(CAM_FOV / 2 / WD))
        pl.camera.clipping_range = (1.0, 4000.0)
        pl.add_text(f"CAMERA VIEW (POMEAS, fixed)\nFOV ~{CAM_FOV:.0f} mm @ WD {WD:.0f} mm (concept)",
                    position="upper_left", font_size=9, color="#FFFFFF")
        self.cam_hud = pl.add_text(" ", position="lower_left", font_size=9, color="#F0B07A", font="courier")

    # ---------------- 조작 ----------------
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

    def toggle_hud(self):
        self.show_hud = not getattr(self, "show_hud", True)
        for a in (self.txt, self.hud, self.cam_hud):
            a.SetVisibility(self.show_hud)
        self.pl.render()

    def toggle_factory(self):
        self.show_factory = not self.show_factory
        for a in self.factory_actors:
            a.SetVisibility(self.show_factory)
        for a in self.patch_actors:
            a.SetVisibility(not self.show_factory)
        self.pl.render()

    def faster(self):
        self.speed = min(32.0, self.speed * 2)

    def slower(self):
        self.speed = max(0.125, self.speed / 2)

    def step(self, d):
        self.playing = False
        self.set_time((self.t + d) % self.plan.total)
        self.pl.render()

    def goto_door_open(self):
        """자동문 열림 구간 1 s 전으로 이동 후 재생"""
        self.set_time(max(0.0, self.plan.door_open_t - 1.0))
        self.set_play(True)

    def _on_slider(self, v):
        if self._slider_lock or not self._ready:
            return
        self.playing = False
        self.set_time(float(v))

    def set_time(self, t):
        self._reset_state()
        self.t = t
        self.update(force=True)

    # ---------------- 프레임 갱신 ----------------
    def update(self, force=False):
        now = time.perf_counter()
        dt = min(0.5, now - self.last)
        self.last = now
        plan = self.plan
        T = plan.total
        prev = self.t % T
        if self.playing and not force:
            self.t += dt * self.speed
        tl = self.t % T
        if tl < prev and not force:          # 한 사이클 종료 → 랙 상태 초기화
            self._reset_state()
        xc, q, grip, label, prog, ins = plan.sample(tl)
        self._apply_events(tl)

        # 캐리지·체인
        Mc = H(None, (xc, 0.0, 0.0))
        for a in self.carr_actors:
            a.user_matrix = Mc
        sc = Scene()
        draw_chain(sc, xc)
        groups = scene_to_groups(sc)
        for key, m in groups.items():
            if key in self.dyn_actors:
                a = self.dyn_actors[key]
                a.mapper.dataset.copy_from(m)
                a.mapper.dataset.Modified()
            else:
                self.pl.subplot(0, 0)
                self.dyn_actors[key] = add_group(self.pl, key, m)

        # 로봇·툴
        B = base_pose(xc)
        Fs = fk(q)
        for a, k in self.robot_actors:
            a.user_matrix = B @ Fs[k]
        F6w = B @ Fs[6]
        Mt = F6w @ self.Mtool
        Mp = F6w @ self.Mfp
        for a in self.tool_actors + self.cam_tool:
            a.user_matrix = Mt
        off = PROD_W / 2 + 6 + 18 * (1 - grip)
        for (a, b), s in zip(zip(self.jaw_actors, self.cam_jaws), (-1, 1)):
            M = Mt @ H(None, (0, L_FORK + s * off, 0))
            a.user_matrix = M
            b.user_matrix = M
        self.ring_actor.user_matrix = Mt
        flash = tl < self.flash_until
        self.ring_actor.prop.color = (1.0, 1.0, 0.6) if flash else COL["ringlight"]
        self.tcp_marker.user_matrix = H(None, Mt[:3, 3])

        # 자동문
        du = plan.door_u(tl)
        for side, idx, acts in self.leaves:
            M = H(None, (door_leaf_x0(side, idx, du), DOOR_TRACK[idx], 0.0))
            for a in acts:
                a.user_matrix = M
        if du <= 1e-3:
            lamp, dstate = COL["ok"], L_(plan, "닫힘 (자동 운전 · 인터록 ON)", "CLOSED (auto)")
        elif du >= 1 - 1e-3:
            lamp, dstate = COL["yellow"], L_(plan, "열림 (로봇 정지)", "OPEN (robot stopped)")
        else:
            lamp = COL["amber"] if int(tl * 4) % 2 == 0 else (0.35, 0.22, 0.05)
            dstate = L_(plan, f"동작 중 {du * 100:3.0f}% 열림", f"MOVING {du * 100:3.0f}% open")
        for a in self.door_lamps:
            a.prop.color = lamp

        # 제품
        for k in range(TIERS):
            for a in self.p_in[k]:
                a.SetVisibility(k in self.inp)
            for a in self.p_out[k]:
                a.SetVisibility(k in self.out)
        for a in self.p_jig:
            a.SetVisibility(self.on_jig)
        self._place(self.p_carry, Mp, self.carry)
        self._place(self.cam_prod, Mp, self.carry)
        show_prog = self.carry and prog is not None
        pg = prog or (0, 0)
        counts = ((pg[0], plan.n_out), (pg[1], plan.n_in))
        for line, a, b, (n_done, n_all) in zip(self.prog_lines, self.prog_actors, self.cam_prog, counts):
            done = np.zeros(line.n_points)
            done[:n_done] = 1.0
            if n_done >= n_all:
                done[:] = 1.0
            line.point_data["done"][:] = done
            line.Modified()
            for act in (a, b):
                act.user_matrix = Mp
                act.SetVisibility(show_prog)

        # 텍스트
        L = plan.L
        jd = f"   ·   {L('판정', 'Judge')}: {self.judge}" if self.judge else ""
        s = (f"{L('BSV Slit Valve O-ring 정밀 검사 설비 — 통합 시뮬레이션 V7 (오픈형 철망 펜스 + 양개 전동 자동문)', 'BSV O-ring inspection cell - integrated simulation V7 (mesh fence + auto door)')}\n"
             f"{L('단계', 'Step')}: {label}\n"
             f"{L('자동문', 'Auto door')}: {dstate}\n"
             f"{L('제7축', 'Carriage')} X = {xc:7.0f} mm   ·   {L('속도', 'speed')} ×{self.speed:g}   ·   "
             f"{L('재생', 'play') if self.playing else L('일시정지', 'pause')}\n"
             f"{L('입력 랙', 'Input')} {len(self.inp)}  /  {L('양품', 'OK')}(OK) {self.n_ok}  /  "
             f"{L('불량', 'NG')}(NG) {self.n_ng}   ·   {L('사이클', 'cycle')} {tl:6.1f} / {T:.0f} s{jd}")
        set_text(self.txt, s)
        qd = q / D2R
        fl = Mt[:3, 3]
        hud = (f"q [deg] ={qd[0]:7.1f}{qd[1]:7.1f}{qd[2]:7.1f}{qd[3]:7.1f}{qd[4]:7.1f}{qd[5]:7.1f}\n"
               f"flange (world) = ({fl[0]:8.1f},{fl[1]:8.1f},{fl[2]:8.1f}) mm   gamma = {plan.gamma / D2R:.1f} deg\n")
        cam = " "
        if ins >= 0:
            f = plan.frames[ins]
            pt = (Mp @ np.r_[f["P"], 0.0, 1.0])[:3]
            wd_now = Z_LENS - pt[2]
            xy_err = math.hypot(pt[0] - Q[0], pt[1] - Q[1])
            clr = plan.metrics["lens"][plan.metrics["block"][ins]] if len(plan.metrics["block"]) > ins else np.inf
            state = "INSPECT" if f["inspect"] else "MOVE   "
            hud += (f"[{f['loop']:5s}] {state} idx {f['idx'] + 1:4d}   psi = {f['psi'] / D2R:8.2f} deg   tilt = {f['tilt'] / D2R:.1f}\n"
                    f"P_i (product)   = ({f['P'][0]:8.2f},{f['P'][1]:8.2f}, 0.00) mm\n"
                    f"T_i = Q - R_i P_i ({f['T'][0]:8.2f},{f['T'][1]:8.2f},{f['T'][2]:8.2f}) mm\n"
                    f"WD (lens-point) = {wd_now:8.3f} mm   XY offset = {xy_err:.3f} mm\n"
                    f"lens clearance  = {clr:8.1f} mm  {'OK' if clr > 5 else '!! CHECK'}")
            cam = f"point -> Q offset {xy_err:.3f} mm\nWD {wd_now:.3f} mm   psi {f['psi'] / D2R:.1f} deg"
        set_text(self.hud, hud)
        set_text(self.cam_hud, cam, corner=0)
        if self.slider is not None and not self._slider_lock:
            self._slider_lock = True
            try:
                self.slider.GetRepresentation().SetValue(tl)
            finally:
                self._slider_lock = False

    def tick(self, *_):
        self.update()
        self.pl.render()


def default_camera(pl):
    pl.subplot(0, 0)
    pl.camera_position = [(-1300.0, -6400.0, 2700.0), (-250.0, 0.0, 1050.0), (0.0, 0.0, 1.0)]
    pl.camera.view_angle = 30.0


def make_plotter(off_screen=False, size=(1800, 900)):
    cam_inset = (0.70, 0.03, 0.98, 0.24)   # 카메라 작은 창 위치 (왼쪽, 아래, 오른쪽, 위) — 화면 비율 0~1
    pl = pv.Plotter(shape=(1, 2), col_weights=[2.3, 1.0], border=False, off_screen=off_screen, window_size=size,
                    lighting="light_kit", title="BSV Slit Valve O-ring Inspection - Simulation V7")
    # 카메라 화면을 오른쪽 위 작은 창으로: 설비 화면은 전체, 카메라 화면을 그 위에 겹침
    pl.renderers[0].SetViewport(0.0, 0.0, 1.0, 1.0)
    pl.renderers[1].SetViewport(*cam_inset)
    pl.renderers[1].SetInteractive(False)          # 마우스 회전·확대는 설비 화면만
    pl.subplot(0, 0)
    pl.set_background("#d9dee4", top="#f6f8fa", all_renderers=False)
    try:
        pl.enable_depth_peeling(number_of_peels=4)
    except Exception:
        pass
    return pl


def run_interactive(plan, factory=True, use_vtk_timer=False):
    pl = make_plotter()
    try:
        pl.enable_anti_aliasing("ssaa")
    except Exception:
        pass
    sim = CellSim(pl, plan, factory=factory, font=find_font())
    default_camera(pl)
    pl.add_key_event("space", sim.toggle_play)
    pl.add_key_event("Up", sim.faster)
    pl.add_key_event("Down", sim.slower)
    pl.add_key_event("Right", lambda: sim.step(0.25))
    pl.add_key_event("Left", lambda: sim.step(-0.25))
    pl.add_key_event("k", lambda: sim.set_cover(not sim.show_cover))
    pl.add_key_event("n", sim.toggle_factory)
    pl.add_key_event("h", sim.toggle_hud)
    pl.add_key_event("d", sim.goto_door_open)
    pl.add_key_event("c", lambda: (default_camera(pl), pl.render()))
    print(f"사이클 {plan.total:.0f} s (제품 {TIERS}개) — 창을 닫거나 q 키로 종료")

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


def snapshot_times(plan: Plan):
    """제품 1개 사이클의 단계별 대표 시점 (+ 검사 중 4개 시점)"""
    picks, seen = [], set()
    t_end = plan.cycle1
    for i in range(len(plan.T)):
        if plan.T[i] > t_end:
            break
        key = plan.LAB[i].split(" —")[0].split(" (")[0]
        if plan.INS[i] >= 0:
            continue
        if key in seen:
            continue
        seen.add(key)
        j = i
        while j + 1 < len(plan.T) and plan.LAB[j + 1] == plan.LAB[i]:
            j += 1
        picks.append((plan.T[j] - 1e-3, plan.LAB[i]))
    blk = plan.metrics["block"]
    for frac in (0.12, 0.40, 0.52, 0.80):
        if len(blk):
            i = blk[int(frac * (len(blk) - 1))]
            picks.append((plan.T[i], plan.LAB[i]))
    return sorted(picks)


def run_snapshots(plan, out_dir: Path, factory=False):
    out_dir.mkdir(parents=True, exist_ok=True)
    pl = make_plotter(off_screen=True, size=(1920, 960))
    sim = CellSim(pl, plan, factory=factory, font=find_font())
    sim.playing = False
    for j, (t, lab) in enumerate(snapshot_times(plan)):
        sim.set_time(t)
        default_camera(pl)
        pl.render()
        pl.screenshot(str(out_dir / f"snap_{j:02d}.png"))
        print(f"  snap {j:02d}  t={t:7.2f}s  {lab}")
    pl.close()


def record_gif(plan, path, factory=False, dt=0.4):
    pl = make_plotter(off_screen=True, size=(1440, 720))
    sim = CellSim(pl, plan, factory=factory, font=find_font())
    sim.playing = False
    default_camera(pl)
    pl.open_gif(path, fps=12)
    t = 0.0
    while t <= plan.cycle1:
        sim.set_time(t)
        pl.write_frame()
        blk = plan.metrics["block"]
        in_insp = len(blk) and plan.T[blk[0]] <= t <= plan.T[blk[-1]]
        t += dt * (2.5 if in_insp else 1.0)
    pl.close()
    print("GIF 저장:", path)


def main():
    ap = argparse.ArgumentParser(description="BSV O-ring 정밀 검사 설비 통합 시뮬레이터 V7 (V6 + 오픈형 철망 펜스·양개 전동 자동문)")
    ap.add_argument("--no-factory", action="store_true", help="공장 환경 숨김")
    ap.add_argument("--snap", type=str, default=None, help="창 없이 스크린샷 저장 폴더")
    ap.add_argument("--gif", type=str, default=None, help="제품 1개 사이클 GIF 녹화 경로 (imageio 필요)")
    ap.add_argument("--csv", type=str, default=None, help="전체 사이클 경로·관절각 CSV 저장 경로")
    ap.add_argument("--vtk-timer", action="store_true", help="VTK 타이머 방식으로 재생 (기본: 직접 루프)")
    a = ap.parse_args()

    font = find_font()
    print("경로 계획 · IK 계산 중…")
    plan = Plan(ko=font is not None)
    print("검증 지표 계산 중…", flush=True)
    evaluate(plan)
    summarize(plan)
    if a.csv:
        export_csv(plan, a.csv)
    if a.snap:
        run_snapshots(plan, Path(a.snap), factory=not a.no_factory)
    elif a.gif:
        record_gif(plan, a.gif, factory=not a.no_factory)
    else:
        run_interactive(plan, factory=not a.no_factory, use_vtk_timer=a.vtk_timer)


if __name__ == "__main__":
    main()
