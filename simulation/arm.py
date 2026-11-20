"""
ELITE CS66 Slit Valve O-ring 로봇 검사 경로 3D 시뮬레이터 V2
================================================================
- 고정 카메라 + 로봇이 제품을 45° 기울여 파지 → O-ring 외측·내측 윤곽을 따라 이동
- 핵심 계산식
    Rᵢ = Rx(45°)·Rz(ψᵢ)            (윤곽 추종 자세, 장변에서는 ψ = 0 → Rx(45°))
    Tᵢ = Q − Rᵢ·Pᵢ                  (검사점 Pᵢ를 카메라 촬영점 Q로 이동)
    Tᴮ_TCP = Tᴮ_P · (Tᵀᶜᴾ_P)⁻¹      (로봇 TCP(Tool Center Point, 공구 중심점) 지령값)
- 화면: 왼쪽 = 설비 전체 3D, 오른쪽 = 고정 카메라 시점 (검사점이 항상 화면 중앙에 오는지 확인)

환경: Python 3.12.3 / pip install pyvista numpy scipy

실행
    python oring_robot_sim_v2.py                  # 대화형 3D 시뮬레이션 (실행 즉시 자동 재생)
    python oring_robot_sim_v2.py --csv path.csv   # 경로·관절각 데이터 CSV 저장 후 실행
    python oring_robot_sim_v2.py --offscreen      # 창 없이 대표 프레임 PNG 저장 (검증용)
    python oring_robot_sim_v2.py --mp4 sim.mp4    # 전체 동작 동영상 녹화 (imageio, imageio-ffmpeg 필요)
    python oring_robot_sim_v2.py --gif sim.gif    # 전체 동작 GIF 녹화 (imageio 필요)

    ※ Jupyter / VS Code 대화형 창이 아니라 터미널(명령 프롬프트)에서 실행해야 3D 창 재생이 됨

V2 변경
    - 시작 직후 슬라이더 초기 이벤트가 '일시정지'로 처리되어 재생이 멈추던 버그 수정
    - Space 키와 재생 버튼 상태 동기화
    - --mp4 동영상 녹화 옵션 추가

조작
    Space        재생 / 일시정지
    ← / →        한 프레임 이동 (일시정지 상태)
    ↑ / ↓        재생 속도 증가 / 감소
    하단 슬라이더  프레임 직접 이동
    마우스         회전(좌) / 이동(휠 클릭) / 확대(휠)

주의 (개념 시뮬레이션)
    - 로봇 기구학(DH 파라미터)은 6축 협동로봇 일반형 개념값 — ELITE CS66 실제 DH로 교체 필요 (CONFIG.dh)
    - 카메라 촬영점 Q, 작업거리, 그리퍼 오프셋, O-ring 윤곽 치수는 모두 예시·가정값
    - 간섭 검사는 렌즈 하부 원통 영역에 대한 단순 점 검사 (정밀 충돌 검사 아님)
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass, field

import numpy as np
import pyvista as pv
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

D2R = np.pi / 180.0


# ============================================================================
# 1. 설정 (모든 수치는 개념·예시값)
# ============================================================================
@dataclass
class Config:
    # 카메라 촬영점 Q (로봇 베이스 좌표, mm) 와 작업거리
    Q: tuple = (450.0, 0.0, 450.0)
    wd: float = 65.0                      # 작업거리: 렌즈 선단 ~ 촬영점
    tilt_deg: float = 45.0                # 파지 기울기 (Xp축 기준)
    cam_fov_mm: float = 20.0              # 오른쪽 카메라 화면 시야 폭 (표시용)

    # 제품 (제품 좌표: 원점 = 상면 중심, Xp 길이 / Yp 폭 / Zp 상면 법선)
    body_half: tuple = (180.0, 30.0)      # 360 × 60
    body_thk: float = 20.0
    body_r: float = 8.0
    oring_outer: tuple = (170.0, 24.0, 20.0)   # 반폭X, 반폭Y, 코너R (외측 윤곽)
    oring_inner: tuple = (166.0, 20.0, 16.0)   # (내측 윤곽) → O-ring 폭 4 mm
    step_mm: float = 1.5                  # 검사점 간격 (시야 2 mm · 중첩 25% 가정)

    # 툴 체인: 플랜지 → TCP → 제품 원점
    flange_to_tcp: float = 60.0
    tcp_to_product: float = 150.0

    # 로봇 DH (표준 DH: d, a, alpha) — 6축 협동로봇 개념값, CS66 실제값으로 교체 필요
    dh: list = field(default_factory=lambda: [
        (150.0, 0.0, np.pi / 2), (0.0, -420.0, 0.0), (0.0, -400.0, 0.0),
        (125.0, 0.0, np.pi / 2), (105.0, 0.0, -np.pi / 2), (100.0, 0.0, 0.0)])
    joint_limit_deg: float = 360.0

    # 루프 간 이동 프레임 수
    transition_frames: int = 60
    tool_mount_deg: float | None = None   # 그리퍼 장착 각도 (None = J6 범위 중앙 정렬 자동 계산)
    # 렌즈 간섭 영역 (렌즈 축 기준 반경, mm)
    lens_keepout_r: float = 55.0


CFG = Config()


# ============================================================================
# 2. 수학 함수
# ============================================================================
def Rx(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def H(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def rounded_rect(a, b, r, ds):
    """반폭 a(X), b(Y), 코너 R r 의 둥근 사각 윤곽을 간격 ds로 샘플 → (점, 외향 법선, 둘레)"""
    segs = [("L", (-(a - r), b), (a - r, b), (0, 1)), ("A", (a - r, b - r), 90, 0),
            ("L", (a, b - r), (a, -(b - r)), (1, 0)), ("A", (a - r, -(b - r)), 0, -90),
            ("L", (a - r, -b), (-(a - r), -b), (0, -1)), ("A", (-(a - r), -(b - r)), -90, -180),
            ("L", (-a, -(b - r)), (-a, b - r), (-1, 0)), ("A", (-(a - r), b - r), 180, 90)]
    lens = [np.hypot(s[2][0] - s[1][0], s[2][1] - s[1][1]) if s[0] == "L" else abs(s[3] - s[2]) * D2R * r for s in segs]
    total = sum(lens)
    n = max(8, int(round(total / ds)))
    pts, nrm = [], []
    for k in range(n):
        s_ = k * total / n
        for seg, L in zip(segs, lens):
            if s_ <= L + 1e-9:
                u = s_ / L if L > 0 else 0.0
                if seg[0] == "L":
                    p = np.array(seg[1]) + u * (np.array(seg[2]) - np.array(seg[1]))
                    nn = np.array(seg[3], float)
                else:
                    ang = (seg[2] + u * (seg[3] - seg[2])) * D2R
                    nn = np.array([np.cos(ang), np.sin(ang)])
                    p = np.array(seg[1]) + r * nn
                pts.append(p)
                nrm.append(nn)
                break
            s_ -= L
    return np.array(pts), np.array(nrm), total


# ---------------- 로봇 기구학 ----------------
def dh_A(th, d, a, al):
    ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]])


def fk(q, dh):
    """관절각 q → 각 링크 좌표계 [F0(베이스), F1, …, F6(플랜지)]"""
    T = np.eye(4)
    out = [T.copy()]
    for qi, (d, a, al) in zip(q, dh):
        T = T @ dh_A(qi, d, a, al)
        out.append(T.copy())
    return out


def ik_residual(q, dh, M_target, w_rot=300.0):
    T = fk(q, dh)[-1]
    e_p = T[:3, 3] - M_target[:3, 3]
    # 회전 오차: 행렬 성분 차이 (180° 뒤집힘 해를 배제하기 위해 vee 형식 대신 사용)
    e_r = (T[:3, :3] - M_target[:3, :3]).ravel() * w_rot
    return np.r_[e_p, e_r]


def ik_solve(M_target, q_seed, dh):
    r = least_squares(ik_residual, q_seed, args=(dh, M_target), method="lm", xtol=1e-12, ftol=1e-12)
    return r.x, float(np.linalg.norm(r.fun))


def ik_initial(M_target, dh, n_seeds=200, seed=3):
    """첫 자세: 무작위 초기값 탐색 → 팔꿈치 위(elbow-up)·바닥 위 해 선택"""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(n_seeds):
        q, res = ik_solve(M_target, rng.uniform(-np.pi, np.pi, 6), dh)
        if res > 1e-6:
            continue
        q = (q + np.pi) % (2 * np.pi) - np.pi
        O = [F[:3, 3] for F in fk(q, dh)]
        if min(o[2] for o in O[1:]) < 60.0:
            continue
        score = O[2][2] - 0.3 * abs(O[2][1])
        if np.abs(fk(q, dh)[-1][:3, :3] - M_target[:3, :3]).max() > 1e-6:
            continue
        if best is None or score > best[0]:
            best = (score, q)
    if best is None:
        raise RuntimeError("초기 자세 IK 해 없음 — Q, 그리퍼 오프셋, DH 값을 확인하세요.")
    return best[1]


# ============================================================================
# 3. 경로 계획: 검사점 → 제품 자세 → TCP → 플랜지 → 관절각
# ============================================================================
def plan(cfg: Config):
    Q = np.array(cfg.Q)
    tilt = cfg.tilt_deg * D2R
    M_tcp_P = H(t=(0, 0, cfg.tcp_to_product))
    M_f_tcp = H(t=(0, 0, cfg.flange_to_tcp))

    loops = []
    for name, (a, b, r), inward in (("OUTER", cfg.oring_outer, False), ("INNER", cfg.oring_inner, True)):
        pts, nrm, perim = rounded_rect(a, b, r, cfg.step_mm)
        view_n = -nrm if inward else nrm                       # 관찰할 측면의 법선
        alpha = np.arctan2(view_n[:, 1], view_n[:, 0])
        psi = np.unwrap(np.pi / 2 - alpha)                     # Rz(ψ)·n → +Yp 정렬
        loops.append(dict(name=name, P=np.c_[pts, np.zeros(len(pts))], psi=psi, perim=perim))

    # 내측 루프 시작 ψ를 외측 끝 ψ와 가깝게 (2π 배수 조정) → 6축 회전량 최소화
    end_psi = loops[0]["psi"][-1]
    k = np.round((end_psi - loops[1]["psi"][0]) / (2 * np.pi))
    loops[1]["psi"] = loops[1]["psi"] + 2 * np.pi * k

    frames = []
    for lp in loops:
        for i in range(len(lp["P"])):
            frames.append(dict(loop=lp["name"], inspect=True, idx=i, P=lp["P"][i], psi=lp["psi"][i]))
        if lp is loops[0]:
            # 외측 → 내측 이동 (검사 없음): P, ψ 선형 보간
            P0, P1 = loops[0]["P"][-1], loops[1]["P"][0]
            s0, s1 = loops[0]["psi"][-1], loops[1]["psi"][0]
            for j in range(1, cfg.transition_frames + 1):
                u = j / (cfg.transition_frames + 1)
                u = 0.5 - 0.5 * np.cos(np.pi * u)              # 부드러운 가감속
                frames.append(dict(loop="MOVE", inspect=False, idx=j, P=P0 + (P1 - P0) * u, psi=s0 + (s1 - s0) * u))

    for f in frames:
        R = Rx(tilt) @ Rz(f["psi"])
        T = Q - R @ f["P"]                                     # Tᵢ = Q − RᵢPᵢ
        f["R"], f["T"] = R, T
        f["check"] = float(np.linalg.norm(T + R @ f["P"] - Q))  # 검산: 0 이어야 함
        f["M_B_P"] = H(R, T)
        f["M_B_TCP"] = f["M_B_P"] @ np.linalg.inv(M_tcp_P)     # Tᴮ_TCP = Tᴮ_P·(Tᵀᶜᴾ_P)⁻¹

    # ---- 그리퍼 장착 각도 γ 자동 결정 ----
    # 윤곽 전체를 돌면 ψ 누적 회전량이 약 535°(외측 356° + 내측 355°, 180° 엇갈림) → 6축(±360°) 범위 중앙에 오도록
    # 플랜지 대비 그리퍼를 Zf축으로 γ 만큼 돌려 장착 (6축 각도 = 기준값 − γ)
    dpsi = np.array([f["psi"] for f in frames]) - frames[0]["psi"]
    if cfg.tool_mount_deg is None:
        q_try = ik_initial(frames[0]["M_B_TCP"] @ np.linalg.inv(H(t=(0, 0, cfg.flange_to_tcp))), cfg.dh)
        q6_target = -(dpsi.min() + dpsi.max()) / 2
        gamma = (q_try[5] - q6_target + np.pi) % (2 * np.pi) - np.pi
    else:
        gamma = cfg.tool_mount_deg * D2R
    M_f_tcp = H(Rz(gamma), (0, 0, cfg.flange_to_tcp))
    print(f"  그리퍼 장착 각도 γ = {gamma / D2R:.1f}° (플랜지 Zf축 기준)")
    for f in frames:
        f["M_B_F"] = f["M_B_TCP"] @ np.linalg.inv(M_f_tcp)     # 플랜지 목표

    # ---- IK (연속 추종) ----
    q = ik_initial(frames[0]["M_B_F"], cfg.dh)
    q[5] = q[5] - 2 * np.pi * np.round((q[5] + (dpsi.min() + dpsi.max()) / 2) / (2 * np.pi))
    t0 = time.time()
    for n, f in enumerate(frames):
        q, res = ik_solve(f["M_B_F"], q, cfg.dh)
        f["q"], f["ik_res"] = q.copy(), float(np.abs(fk(q, cfg.dh)[-1] - f["M_B_F"]).max())
        f["Fs"] = fk(q, cfg.dh)
        if n % 200 == 0:
            print(f"  IK {n:4d}/{len(frames)}  잔차 {res:.1e}", flush=True)
    print(f"  IK 완료 {time.time() - t0:.1f}s")
    # 각 관절 궤적을 2π 배수만큼 평행이동 → 관절 한계(±360°) 안에서 가운데 정렬 (자세는 동일)
    Qs = np.array([f["q"] for f in frames])
    shift = -2 * np.pi * np.round((Qs.max(0) + Qs.min(0)) / 2 / (2 * np.pi))
    for f in frames:
        f["q"] = f["q"] + shift
    return frames, loops, dict(M_tcp_P=M_tcp_P, M_f_tcp=M_f_tcp, gamma=gamma)


# ============================================================================
# 4. 형상 (개념 설계)
# ============================================================================
COL = dict(white="#F2F3F5", cap="#2E3440", alu="#C3C9D2", jig="#4B5566", rubber="#E07A1F", cam="#2A2F38",
           lens="#1C1F24", ring="#DADDE2", stand="#AEB5BF", red="#C8102E", done="#2E9E5B", todo="#8E97A6")


def cyl(c, axis, r, h, res=48):
    return pv.Cylinder(center=c, direction=axis, radius=r, height=h, resolution=res, capping=True).triangulate()


def seg(p0, p1, r):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    return cyl((p0 + p1) / 2, p1 - p0, r, np.linalg.norm(p1 - p0))


def rrect_solid(a, b, r, z0, z1):
    pts, _, _ = rounded_rect(a, b, r, 1.0)
    P = np.c_[pts, np.full(len(pts), z0)]
    poly = pv.PolyData(P, faces=np.r_[len(P), np.arange(len(P))])
    return poly.extrude((0, 0, z1 - z0), capping=True).triangulate()


def product_meshes(cfg: Config):
    """제품 좌표 기준 형상: (이름, 메쉬, 색)"""
    a, b = cfg.body_half
    oa, ob, orr = cfg.oring_outer
    ia, ib, irr = cfg.oring_inner
    ca, cb, cr = (oa + ia) / 2, (ob + ib) / 2, (orr + irr) / 2
    pts, _, _ = rounded_rect(ca, cb, cr, 1.0)
    ring = np.c_[pts, np.full(len(pts), 0.6)]
    oring = pv.lines_from_points(np.r_[ring, ring[:1]]).tube(radius=(oa - ia) / 2, n_sides=16)
    out = [("body", rrect_solid(a, b, cfg.body_r, -cfg.body_thk, 0), COL["alu"]),
           ("oring", oring, COL["rubber"]),
           ("jig", pv.Box(bounds=(-a - 15, a + 15, -b - 10, b + 10, -cfg.body_thk - 14, -cfg.body_thk)).triangulate(), COL["jig"])]
    for sx in (-150, 150):
        for y0, y1 in ((-b - 10, -b - 2), (b + 2, b + 10)):
            out.append(("clamp", pv.Box(bounds=(sx - 9, sx + 9, y0, y1, -cfg.body_thk, -8)).triangulate(), COL["jig"]))
    # 그리퍼 브래킷 (TCP ~ 지그 하부)
    out.append(("bracket", seg((0, 0, -cfg.tcp_to_product), (0, 0, -cfg.body_thk - 14), 24), "#6B7484"))
    out.append(("plate", pv.Box(bounds=(-60, 60, -30, 30, -cfg.body_thk - 22, -cfg.body_thk - 14)).triangulate(), "#6B7484"))
    return out


def robot_meshes(q_ref, cfg: Config):
    """로봇 형상을 기준 자세에서 만든 뒤 각 링크 좌표계로 변환 → (링크번호, 로컬 메쉬, 색)"""
    Fs = fk(q_ref, cfg.dh)
    O = [F[:3, 3] for F in Fs]
    Z = [F[:3, 2] for F in Fs]
    items = []

    def housing(k, c, ax, r, h):
        items.append((k, cyl(c, ax, r, h), COL["white"]))
        ax = ax / np.linalg.norm(ax)
        for sgn in (-1, 1):
            items.append((k, cyl(c + sgn * ax * (h / 2 + 4), ax, r + 0.5, 8), COL["cap"]))

    items.append((0, cyl((0, 0, 8), (0, 0, 1), 82, 16), COL["white"]))
    items.append((0, cyl((0, 0, 70), (0, 0, 1), 62, 110), COL["white"]))
    items.append((1, cyl((0, 0, 128), (0, 0, 1), 63, 8), COL["cap"]))
    housing(1, O[1] + 70 * Z[1], Z[1], 62, 150)                 # J2
    items.append((2, seg(O[1] + 90 * Z[1], O[2] + 90 * Z[1], 46), COL["white"]))   # 상완
    housing(2, O[2] + 60 * Z[2], Z[2], 54, 140)                 # J3
    items.append((3, seg(O[2] + 25 * Z[2], O[3] + 25 * Z[2], 38), COL["white"]))   # 전완
    housing(3, O[3] + 70 * Z[3], Z[3], 42, 120)                 # J4
    items.append((4, seg(O[4], O[5], 38), COL["white"]))
    housing(4, O[5] - 10 * Z[4], Z[4], 42, 95)                  # J5
    items.append((5, seg(O[5], O[6] - 10 * Z[5], 38), COL["white"]))   # J6
    items.append((6, cyl(O[6] - 6 * Z[5], Z[5], 40, 12), COL["cap"]))   # 플랜지
    items.append((6, seg(O[6], O[6] + cfg.flange_to_tcp * Z[6], 30), "#8E96A3"))  # 툴 어댑터
    return [(k, m.transform(np.linalg.inv(Fs[k]), inplace=False), c) for k, m, c in items]


def camera_meshes(cfg: Config):
    Q = np.array(cfg.Q)
    z0 = Q[2] + cfg.wd
    out = [cyl((Q[0], Q[1], z0 + 55), (0, 0, 1), 21, 110), cyl((Q[0], Q[1], z0 + 3), (0, 0, 1), 23, 6)]
    cols = [COL["lens"], "#5A616C"]
    out.append(pv.ParametricTorus(ringradius=40, crosssectionradius=9).translate((Q[0], Q[1], z0 + 10)))
    cols.append(COL["ring"])
    out.append(pv.Box(bounds=(Q[0] - 32, Q[0] + 32, Q[1] - 32, Q[1] + 32, z0 + 110, z0 + 195)).triangulate())
    cols.append(COL["cam"])
    top = z0 + 195
    # 카메라 스탠드: 제품 회전 영역을 피해 +Y 쪽(뒤편)에 설치 (개념)
    sy = 420.0
    for bx in [(Q[0] - 20, Q[0] + 20, Q[1] - 20, Q[1] + 20, top, 800), (Q[0] - 20, Q[0] + 20, Q[1] - 20, sy + 20, 800, 840),
               (Q[0] - 20, Q[0] + 20, sy - 20, sy + 20, 0, 800), (Q[0] - 60, Q[0] + 60, sy - 60, sy + 60, 0, 12)]:
        out.append(pv.Box(bounds=bx).triangulate())
        cols.append(COL["stand"])
    return list(zip(out, cols))


def floor_grid(xr=(-400, 1000), yr=(-500, 500), step=50):
    lines = [pv.Line((x, yr[0], 0), (x, yr[1], 0)) for x in np.arange(xr[0], xr[1] + 1, step)]
    lines += [pv.Line((xr[0], y, 0), (xr[1], y, 0)) for y in np.arange(yr[0], yr[1] + 1, step)]
    plane = pv.Plane(center=((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, -0.5), i_size=xr[1] - xr[0], j_size=yr[1] - yr[0])
    return plane, lines[0].merge(lines[1:])


# ============================================================================
# 5. 간섭·작업거리 지표
# ============================================================================
def collision_points(cfg: Config):
    """간섭 검사용 제품·지그 표면 점 (제품 좌표)"""
    pts = []
    for name, m, _ in product_meshes(cfg):
        if name in ("body", "jig", "clamp", "oring"):
            pts.append(np.asarray(m.points))
    return np.vstack(pts)


def lens_clearance(M_B_P, pts_local, cfg: Config):
    """렌즈 축 반경 lens_keepout_r 이내 점들의 '렌즈 선단 − 점 높이' 최소값 (음수 = 간섭)"""
    Q = np.array(cfg.Q)
    W = (M_B_P[:3, :3] @ pts_local.T).T + M_B_P[:3, 3]
    r = np.hypot(W[:, 0] - Q[0], W[:, 1] - Q[1])
    inside = r < cfg.lens_keepout_r
    if not inside.any():
        return np.inf
    return float(((Q[2] + cfg.wd) - W[inside, 2]).min())


def summarize(frames, cfg: Config):
    q = np.array([f["q"] for f in frames]) / D2R
    print("\n=========== 경로 검증 요약 ===========")
    print(f"  프레임 수           : {len(frames)}  (검사 {sum(f['inspect'] for f in frames)} / 이동 {sum(not f['inspect'] for f in frames)})")
    print(f"  최대 |Tᵢ+RᵢPᵢ−Q|    : {max(f['check'] for f in frames):.2e} mm")
    print(f"  최대 IK 잔차        : {max(f['ik_res'] for f in frames):.2e}")
    for j in range(6):
        print(f"  J{j + 1} 범위 [deg]     : {q[:, j].min():8.1f} ~ {q[:, j].max():8.1f}" +
              ("   ← 한계 초과" if np.abs(q[:, j]).max() > cfg.joint_limit_deg else ""))
    dq = np.abs(np.diff(q, axis=0)).max(axis=0)
    print(f"  프레임간 최대 관절변화 [deg]: " + " ".join(f"{v:.1f}" for v in dq))
    clr = [f["clear"] for f in frames]
    print(f"  렌즈 최소 여유      : {min(clr):.1f} mm  {'(간섭!)' if min(clr) < 0 else ''}")
    low = min(min(F[:3, 3][2] for F in f["Fs"][1:]) for f in frames)
    print(f"  관절 원점 최저 높이 : {low:.1f} mm  {'(바닥 간섭!)' if low < 0 else ''}")
    print("=======================================\n")


def export_csv(frames, path):
    with open(path, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        w.writerow(["frame", "loop", "inspect", "Px", "Py", "Pz", "psi_deg", "Tx", "Ty", "Tz",
                    "TCPx", "TCPy", "TCPz", "TCP_rx", "TCP_ry", "TCP_rz", "TCP_roll", "TCP_pitch", "TCP_yaw",
                    "q1", "q2", "q3", "q4", "q5", "q6", "check_mm", "lens_clear_mm"])
        for n, f in enumerate(frames):
            M = f["M_B_TCP"]
            rv = Rotation.from_matrix(M[:3, :3]).as_rotvec()
            rpy = Rotation.from_matrix(M[:3, :3]).as_euler("xyz", degrees=True)
            w.writerow([n, f["loop"], int(f["inspect"]), *np.round(f["P"], 4), round(f["psi"] / D2R, 4), *np.round(f["T"], 4),
                        *np.round(M[:3, 3], 4), *np.round(rv, 6), *np.round(rpy, 4), *np.round(f["q"] / D2R, 4),
                        f"{f['check']:.2e}", round(f["clear"], 3) if np.isfinite(f["clear"]) else ""])
    print(f"CSV 저장: {path}")


# ============================================================================
# 6. 시각화
# ============================================================================
class Simulator:
    def __init__(self, cfg: Config, frames, loops, mats, off_screen=False):
        self.cfg, self.frames, self.loops, self.mats = cfg, frames, loops, mats
        self.N = len(frames)
        self.i = 0
        self.playing = True
        self.speed = 2.0
        self._acc = 0.0
        self._slider_lock = False
        self._ready = False
        self.cam_actors, self.prod_actors, self.prod_triad, self.robot_actors = [], [], [], []
        Q = np.array(cfg.Q)

        self.p = pv.Plotter(shape=(1, 2), col_weights=[2.2, 1.0], window_size=(1700, 900), off_screen=off_screen,
                            title="ELITE CS66 O-ring 검사 경로 시뮬레이터 V2")
        p = self.p
        # ---------------- 왼쪽: 설비 전체 ----------------
        p.subplot(0, 0)
        p.renderer.set_background("#FFFFFF", top="#E9EDF2")
        plane, grid = floor_grid()
        p.add_mesh(plane, color="#EEF1F5", ambient=1.0, diffuse=0.0, specular=0.0)
        p.add_mesh(grid, color="#D5DAE2", line_width=1)
        for m, c in camera_meshes(cfg):
            p.add_mesh(m, color=c, smooth_shading=True, specular=0.4)
        p.add_mesh(pv.Line(Q, Q + [0, 0, cfg.wd]), color=COL["red"], line_width=3)
        p.add_mesh(pv.Sphere(radius=4, center=Q), color=COL["red"])
        # TCP 궤적 (전체 경로)
        tcp_pts = np.array([f["M_B_TCP"][:3, 3] for f in frames])
        p.add_mesh(pv.lines_from_points(tcp_pts), color="#6B4FA0", line_width=2, opacity=0.6)
        self.tcp_marker = p.add_mesh(pv.Sphere(radius=6), color="#6B4FA0")
        # 로봇
        for k, m, c in robot_meshes(frames[0]["q"], cfg):
            a = p.add_mesh(m, color=c, smooth_shading=True, specular=0.4 if c == COL["white"] else 0.3)
            self.robot_actors.append((a, k))
        # 제품 + 검사 진행 표시
        self._add_product(p, self.prod_actors)
        # 좌표계 표시 (베이스 / 제품)
        self._triad(p, 160, H())
        self.prod_triad = self._triad(p, 90, None)
        p.add_point_labels([Q + [0, 0, -18]], ["Q"], font_size=14, text_color=COL["red"], shape=None, show_points=False, always_visible=True)
        p.camera_position = [(1750, -1900, 1300), (380, 40, 380), (0, 0, 1)]
        p.camera.view_angle = 25
        self.hud = p.add_text("", position="upper_left", font_size=10, color="#14213D", font="courier")
        p.add_text("Space: play/pause   <- ->: step   Up/Down: speed", position="lower_right", font_size=8, color="#5B6576")
        self.slider = p.add_slider_widget(self._on_slider, [0, self.N - 1], value=0, title="frame", pointa=(0.25, 0.06),
                                          pointb=(0.75, 0.06), style="modern", interaction_event="always", fmt="%.0f",
                                          title_height=0.02, color="#14213D")
        self.play_btn = p.add_checkbox_button_widget(self._on_play, value=True, position=(15, 15), size=32,
                                                     color_on="#2E9E5B", color_off="#C8102E")

        # ---------------- 오른쪽: 고정 카메라 시점 ----------------
        p.subplot(0, 1)
        p.renderer.set_background("#1C1F24")
        self._add_product(p, self.cam_actors, view=True)
        L = cfg.cam_fov_mm * 0.15
        for d in ((L, 0, 0), (0, L, 0)):
            c0 = Q + np.array([0, 0, cfg.wd * 0.5])   # 제품에 가려지지 않도록 렌즈 쪽 위에 표시
            p.add_mesh(pv.Line(c0 - np.array(d), c0 + np.array(d)), color="#FF3B3B", line_width=2)
        p.camera_position = [tuple(Q + [0, 0, cfg.wd]), tuple(Q), (0, 1, 0)]
        p.camera.view_angle = 2 * np.degrees(np.arctan(cfg.cam_fov_mm / 2 / cfg.wd))
        p.camera.clipping_range = (1.0, 3000.0)
        p.add_text(f"CAMERA VIEW (fixed)\nFOV ~{cfg.cam_fov_mm:.0f} mm @ WD {cfg.wd:.0f} mm (concept)", position="upper_left",
                   font_size=9, color="#FFFFFF")
        self.cam_hud = p.add_text("", position="lower_left", font_size=9, color="#F0B07A", font="courier")
        p.subplot(0, 0)

        p.add_key_event("space", self._toggle)
        p.add_key_event("Right", lambda: self._step(1))
        p.add_key_event("Left", lambda: self._step(-1))
        p.add_key_event("Up", lambda: self._speed(1.5))
        p.add_key_event("Down", lambda: self._speed(1 / 1.5))
        self._ready = True
        self.apply(0)

    # ---- 장면 구성 보조 ----
    def _add_product(self, p, store, view=False):
        for name, m, c in product_meshes(self.cfg):
            if view and name in ("bracket", "plate"):
                continue
            a = p.add_mesh(m, color=c, smooth_shading=True, specular=0.5 if name == "body" else 0.25)
            store.append(a)
        # 검사 진행 표시선 (완료 = 초록 / 미완료 = 회색), O-ring 바로 위
        self.prog_meshes = getattr(self, "prog_meshes", [])
        for lp in self.loops:
            P = lp["P"].copy()
            P[:, 2] = 2.9
            line = pv.lines_from_points(np.r_[P, P[:1]])
            line["done"] = np.zeros(line.n_points)
            a = p.add_mesh(line, scalars="done", cmap=[COL["todo"], COL["done"]], clim=[0, 1], line_width=4 if not view else 6,
                           show_scalar_bar=False, render_lines_as_tubes=True)
            store.append(a)
            self.prog_meshes.append((lp["name"], line))

    def _triad(self, p, L, M):
        acts = []
        for k, c in enumerate(("#D1342F", "#2E9E5B", "#2F6FD0")):
            d = np.zeros(3)
            d[k] = 1
            a = p.add_mesh(pv.Arrow(start=(0, 0, 0), direction=d, scale=L, tip_length=0.2, tip_radius=0.07, shaft_radius=0.025), color=c)
            if M is not None:
                a.user_matrix = M
            acts.append(a)
        return acts

    # ---- 프레임 적용 ----
    def apply(self, i):
        if not self._ready:
            return
        i = int(np.clip(i, 0, self.N - 1))
        self.i = i
        f = self.frames[i]
        Fs = f["Fs"]
        for a, k in self.robot_actors:
            a.user_matrix = Fs[k]
        M_P = Fs[6] @ self.mats["M_f_tcp"] @ self.mats["M_tcp_P"]   # 로봇 FK로 계산한 실제 제품 자세
        for a in self.prod_actors + self.cam_actors + self.prod_triad:
            a.user_matrix = M_P
        tcp = (Fs[6] @ self.mats["M_f_tcp"])[:3, 3]
        self.tcp_marker.user_matrix = H(t=tcp)
        # 검사 진행
        for name, line in self.prog_meshes:
            done = np.zeros(line.n_points)
            n_loop = line.n_points - 1
            if name == "OUTER":
                upto = n_loop if f["loop"] in ("MOVE", "INNER") else f["idx"] + 1
            else:
                upto = f["idx"] + 1 if f["loop"] == "INNER" else 0
            done[:upto] = 1.0
            if upto >= n_loop:
                done[:] = 1.0
            line["done"] = done
        # 지표
        pt_world = (M_P @ np.r_[f["P"], 1.0])[:3]
        Q = np.array(self.cfg.Q)
        wd_now = (Q[2] + self.cfg.wd) - pt_world[2]
        xy_err = np.hypot(*(pt_world[:2] - Q[:2]))
        clr = f["clear"]
        q = f["q"] / D2R
        state = "INSPECT" if f["inspect"] else "MOVE   "
        txt = (f"Frame {i + 1:4d}/{self.N}  [{f['loop']:5s}] {state}   speed x{self.speed:.1f}\n"
               f"psi            = {f['psi'] / D2R:9.2f} deg    tilt Rx = {self.cfg.tilt_deg:.0f} deg\n"
               f"P_i  (product) = ({f['P'][0]:8.2f},{f['P'][1]:8.2f},{f['P'][2]:6.2f}) mm\n"
               f"T_i = Q - R_i P_i ({f['T'][0]:8.2f},{f['T'][1]:8.2f},{f['T'][2]:8.2f}) mm\n"
               f"TCP  (base)    = ({tcp[0]:8.2f},{tcp[1]:8.2f},{tcp[2]:8.2f}) mm\n"
               f"q [deg] = {q[0]:7.1f}{q[1]:7.1f}{q[2]:7.1f}{q[3]:7.1f}{q[4]:7.1f}{q[5]:7.1f}\n"
               f"err(T+RP-Q)    = {f['check']:.1e} mm   IK err = {f['ik_res']:.1e}\n"
               f"WD (lens-point)= {wd_now:8.3f} mm   XY offset = {xy_err:.3f} mm\n"
               f"lens clearance = {clr:8.1f} mm  {'OK' if clr > 5 else '!! CHECK'}")
        self.hud.SetText(2, txt)
        self.cam_hud.SetText(0, f"point -> Q offset {xy_err:.3f} mm\nWD {wd_now:.3f} mm")
        if not self._slider_lock:
            self._slider_lock = True
            self.slider.GetRepresentation().SetValue(i)
            self._slider_lock = False

    # ---- 조작 ----
    def _on_slider(self, v):
        # 위젯 생성·초기 렌더 시 들어오는 이벤트(값 = 현재 프레임)는 무시 → 사용자가 실제로 움직였을 때만 일시정지
        v = int(round(v))
        if self._slider_lock or not self._ready or v == self.i:
            return
        self._slider_lock = True
        self._set_playing(False)
        self.apply(v)
        self._slider_lock = False

    def _set_playing(self, state):
        self.playing = bool(state)
        if hasattr(self, "play_btn"):
            self.play_btn.GetRepresentation().SetState(1 if self.playing else 0)

    def _on_play(self, state):
        self.playing = bool(state)

    def _toggle(self):
        self._set_playing(not self.playing)
        self.p.render()

    def _step(self, d):
        self._set_playing(False)
        self.apply((self.i + d) % self.N)
        self.p.render()

    def _speed(self, k):
        self.speed = float(np.clip(self.speed * k, 0.25, 20))

    def _tick(self, _step):
        if not self.playing:
            return
        self._acc += self.speed
        n = int(self._acc)
        if n:
            self._acc -= n
            self.apply((self.i + n) % self.N)

    def run(self):
        self.p.add_timer_event(max_steps=10 ** 9, duration=30, callback=self._tick)
        self.p.show()

    def snapshots(self, prefix="sim"):
        for k, i in enumerate(np.linspace(0, self.N - 1, 5).astype(int)):
            self.apply(i)
            self.p.render()
            fn = f"{prefix}_{k}_frame{i}.png"
            self.p.screenshot(fn)
            print("저장:", fn)
        self.p.close()

    def record_gif(self, path, every=4):
        self.p.open_gif(path)
        for i in range(0, self.N, every):
            self.apply(i)
            self.p.write_frame()
        self.p.close()
        print("GIF 저장:", path)

    def record_mp4(self, path, every=1, fps=30):
        self.p.open_movie(path, framerate=fps, quality=7)
        for i in range(0, self.N, every):
            self.apply(i)
            self.p.write_frame()
        self.p.close()
        print("MP4 저장:", path)


# ============================================================================
# 7. main
# ============================================================================
def main():
    ap = argparse.ArgumentParser(description="ELITE CS66 O-ring 검사 경로 3D 시뮬레이터 V2")
    ap.add_argument("--csv", help="경로·관절각 CSV 저장 경로")
    ap.add_argument("--offscreen", action="store_true", help="창 없이 대표 프레임 PNG 저장")
    ap.add_argument("--gif", help="GIF 녹화 경로 (imageio 필요)")
    ap.add_argument("--mp4", help="MP4 녹화 경로 (imageio, imageio-ffmpeg 필요)")
    args = ap.parse_args()

    print("경로 계획 · IK 계산 중…")
    frames, loops, mats = plan(CFG)
    pts_local = collision_points(CFG)
    for f in frames:
        f["clear"] = lens_clearance(f["M_B_P"], pts_local, CFG)
    for lp in loops:
        print(f"  {lp['name']} 루프: 둘레 {lp['perim']:.1f} mm, 검사점 {len(lp['P'])}개")
    summarize(frames, CFG)
    if args.csv:
        export_csv(frames, args.csv)

    sim = Simulator(CFG, frames, loops, mats, off_screen=bool(args.offscreen or args.gif or args.mp4))
    if args.mp4:
        sim.record_mp4(args.mp4)
    elif args.gif:
        sim.record_gif(args.gif)
    elif args.offscreen:
        sim.snapshots()
    else:
        sim.run()


if __name__ == "__main__":
    sys.exit(main())
