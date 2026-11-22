"""v12: Packout XL corner-tube auto latch on L-track. BRACKET + LID + SLIDER + printed W SPRING at one rear corner,
a static HOOK at the other.

Push the box in: the tube hits the slider's 45 deg ramp, shoves it outboard, and once the tube's 8.5 mm front face is
past, the spring snaps the wedge back across it. Pull the knob (outboard, ~10 mm) to let the box out.

  - SLIDER: one part. A wedge that sticks out of the block's inboard face over the tube's front face, a body that
    slides in a lane in the bracket, a dog that pushes the spring, and a stem + knob out the block's outboard face.
    The box's pull lands flat on the wedge's back face and goes into the bracket's front cheek: no cam-out.
  - SPRING: a flat 4-arm zigzag (W) that lies in a well behind the slider, in the bracket's depth toward the wall.
    Its two ends are round nubs on a common line, so every back fold carries the full moment and the front fold
    almost none: twice the stroke of a plain serpentine for the same strain.
  - LID: closes the top, carries two tabs (one closes the mouth above the wedge and is the slider's "out" stop,
    one closes the knob notch and is the spring's seat). 4 x M6 x 60 cap screws, heads on the lid, nuts in hex
    pockets in the bottom face (bridged in two short layers, no supports). Retract stop: the slider body's end on
    the outboard wall.

v12 vs v11: mounts on horizontal L-track with two 3/8"-24 threaded stud fittings per part, 6" apart, straddling the
block (the tube's load lands between them). The bracket bears on the track itself: a groove along its back clears the
fittings' tops, so tightening the nuts pulls the fittings' lugs up into the track's lips. Leg 10 -> 18 mm (its back
face moved toward the wall) so the 30 mm studs clear the box. Latch: round holes. HOOK (other corner): slots with
30 mm of travel to dial in the box's spacing (L-track only steps in 1"). Install: hook that corner's tube sideways,
swing the box back about it, and the other corner latches. Out: pull the handle, swing out, slide off the hook.

Prints: bracket standing, lid top-face down, slider on its latch face, spring flat. No supports, no bridges.

Frame (truck, same as v9): X out from the cab wall, Y along the wall (+Y outboard; the box is on -Y), Z up, bracket
bottom at Z=0. Tube profile + placement measured off a fan-made Packout 48-22-8425 CAD model.

Run: uv run python v12/latch.py [--check]
"""

import math
import sys
from pathlib import Path

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union

from build123d import Cylinder, Face, Plane, Pos, Rot, Vector, Wire, export_step, export_stl, extrude, mirror

HERE = Path(__file__).parent
OUT = HERE / "out"

IN = 25.4
S2 = math.sqrt(0.5)

# ---------------------------------------------------------------- corner tube (u along its 30 mm width, v along its
# outward normal, origin at its centroid). 30.0 x 14.2. The face toward the tailgate (+X) is the 8.5 mm flat.
BAR_UV = [(5.31, 5.50), (-5.18, 5.62), (-6.56, 7.12), (-10.89, 7.18), (-14.97, 3.22), (-15.01, -0.73), (-9.05, -6.84),
          (8.89, -7.04), (14.96, -1.13), (15.03, 2.87), (11.06, 6.94), (6.72, 6.98)]
C_BACK, C_SIDE = 5.93, 1.47
CORNER_LEG = 41.56
BOX_W, BOX_D = 21.8 * IN, 15.5 * IN

# ---------------------------------------------------------------- bracket (leg + pocket: same as v9)
X_LEG = 30.67           # leg's front face = the tube pocket's back wall (fixed to the box)
LEG_T = 18.0            # leg: the L-track studs' tips must clear the box (see the check)
X_PF = X_LEG - LEG_T    # bracket's back face: sits on the L-track
# ---------------------------------------------------------------- L-track (horizontal), 3/8"-24 double-stud fittings
STUD_D = 10.2           # through hole for the 3/8" (9.5) stud
STUD_H = 30.0           # stud length above the fitting (1.18")
STUD_PITCH = 6 * IN     # latch: two fittings 6" apart (track holes are on a 1" pitch); the shortest that clears the block
HOOK_PITCH = 5 * IN     # hook: its block is small, so 5" (both plates come out ~the same length)
WASHER_R = 12.7         # 3/8" USS flat washer (1" OD)
NUT38_T = 8.3           # 3/8" hex nut
GROOVE_W, GROOVE_D = 22.0, 3.5                  # relief along the back for the fittings' tops (45 deg top edge)
PLATE_END = 16.0        # plate past the outer holes' edges ... from the hole/slot end
HOOK_SLOT = 30.0        # hook's slot travel along the track: > 1" so any box spacing can be dialed in
HOOK_W = 20.0           # hook block, outboard of its lip (Y)
HOOK_TIP = 2.0          # hook lip's tip (Y): it doesn't retract, so it can cover the whole face
HOOK_LEAD = 2.5         # 45 deg lead-in on the lip tip's back edge
HOOK_ENTRY = 20.0       # the hook-side tube slides in sideways this far
FIT = 0.4               # pocket to the tube
ARM_CLR = FIT             # block's inboard face = the tube pocket's outboard wall: one flat face, no step
W_MIN = 3.0             # thinnest feature anywhere, no exceptions
R_OUT, R_IN = 2.0, 2.0
H = 60.0                # overall height: an M6 x 60 cap screw, head on the lid, ends flush with the bottom
LID_T = 6.0
H_B = H - LID_T         # bracket top under the lid
Z_FL = 6.0              # lane floor
SILL_L, SILL_H = 6.0, 5.0                       # sill across the lane at the mouth (Y, Z): the slider's "out" stop

# ---------------------------------------------------------------- slider
T = 14.0                # slider thickness (X)
LANE_CLR = 0.3          # each side, X
Y_TIP = 6.0             # wedge tip, locked: covers 5.2 mm of the tube's 8.5 mm front face
TIP_T = 4.0             # wedge thickness (X) at its tip; the 45 deg ramp grows it to T
RETRACT_CLR = 0.4       # retracted wedge tip past the tube's slide path
TAB_T = 3.5             # lid tab over the mouth (Y): the slider's "out" stop
DOG_T = 6.5             # dog (Y): only pushes the spring now
DOG_GAP = 0.5           # dog to the lid's mouth tab, locked (the sill is the stop, not the tab)
LEAD = 3.5              # lead-in chamfer on the lid's outboard tab: pressing the lid down cams the spring's preload in
STEM_H = 16.0           # stem (Z): runs along the top of the slider body, out through a notch in the outboard wall
KNOB_GAP = 3.0          # handle to the block, locked
HOLE_Y, HOLE_Z = 22.0, 32.0                     # D-handle's finger hole (gloved fingers go through it along X)
BAR = 8.0               # handle bar, all the way around: the hole is the outline offset in by this
HANDLE_R = 13.0         # outer corner radius (the hole's corners: HANDLE_R - BAR, concentric)
HANDLE_CH = 2.0         # 45 deg chamfer on the handle's front/back edges

# ---------------------------------------------------------------- spring (W, lies flat; arms along X)
SP_T = 3.5              # arm thickness
SP_B = 20.0             # spring height (Z): 14 felt right in PLA; ASA is ~1/1.75 as stiff
G_OUT = 7.0             # gap in the two hairpins (arms 1-2, 3-4): each closes ~half the stroke at the front
G_MID = 4.0             # gap between arms 2-3 (they're joined at the front, where the moment is ~0)
NUB1 = 4.0              # arm 1's nub (on the dog) and arm 4's (on the lid tab). Tall, so the end arms can tilt ~12 deg
NUB4 = 4.0              # behind them without touching the flat faces. The force line runs through both nubs
PRE = 3.0               # preload, locked
SP_CLR = 0.6            # spring to its well
X_C = 65.0              # nubs' X (inside the dog's X span)
E = 2000.0              # ASA / PETG, MPa
EPS_MAX = 0.016         # peak strain allowed, fully retracted (it's ~0.3% at rest)
MU = 0.3

# ---------------------------------------------------------------- bolts: 4 x M6 x 60 cap screws, nuts in the bottom
M6_HOLE = 6.4
HEAD_D, HEAD_K = 10.0, 6.0                      # ISO 4762 cap head (renders only: it sits on the lid)
NUT_AF = 10.3           # hex pocket across flats (M6 nut = 10)
NUT_D = 6.0             # pocket depth: a plain M6 nut (5) + 1
BRIDGE = 0.3            # two bridge layers over each pocket: a slot, then a square, then the round hole
BOLT_IN_Y = 9.0         # bolt center in from the block's inboard/outboard faces (hex pocket + 3 mm wall)
CHEEK = NUT_AF + 2 * W_MIN                      # front cheek (X): holds the box pull and the front bolts' nut pockets
RHO = 1.15e-6


def log(msg):
    print(msg, flush=True)


# ================================================================ plan (X-Y)
def uv_to_xy(u, v, c):
    return (c[0] + S2 * u - S2 * v, c[1] + S2 * u + S2 * v)


E_REAR = max(-S2 * u + S2 * v for u, v in BAR_UV)
E_OUT = max(S2 * u + S2 * v for u, v in BAR_UV)
BAR_C = (X_LEG + FIT + E_REAR, C_SIDE)
XB = BAR_C[0] + C_BACK
Y_A = BAR_C[1] + E_OUT + ARM_CLR                # block's inboard face (the slider's mouth)


def bar_xy():
    return Polygon([uv_to_xy(u, v, BAR_C) for u, v in BAR_UV])


def _face():
    pts = list(bar_xy().exterior.coords)
    xf = max(x for x, _ in pts)
    ys = [y for x, y in pts if x > xf - 0.3]
    return xf, min(ys), max(ys)


X_FACE, Y_FACE0, Y_FACE1 = _face()
X_S0 = X_FACE                                   # lane (the tube's face plane is its back wall's plane)
X_T0 = X_S0 + LANE_CLR
X_T1 = X_T0 + T
X_S1 = X_T1 + LANE_CLR
X_END = X_S1 + CHEEK
BAND_TOP = max(y for _, y in bar_xy().exterior.coords) + FIT     # the tube's slide path, outboard edge
TRAVEL = BAND_TOP + RETRACT_CLR - Y_TIP         # wedge stroke to let the tube by
STROKE = TRAVEL + 0.5                           # knob stroke: the body's end lands on the outboard wall, under the stem
DMAX = PRE + STROKE

# spring arms (Y, free shape): arm1 inboard (pushed by the dog), arm4 outboard (sits on the lid's outboard tab)
Y_DOG = Y_A + TAB_T + DOG_GAP + DOG_T           # dog's push face, locked
Y1 = Y_DOG - PRE + NUB1 + SP_T / 2
Y2 = Y1 + G_OUT + SP_T
Y3 = Y2 + G_MID + SP_T
Y4 = Y3 + G_OUT + SP_T
Y_STOP = Y4 + SP_T / 2 + NUB4                    # outboard tab's face
OTAB_T = 6.0
Y_OUT = Y_STOP + OTAB_T                         # block's outboard face

X_BB, X_FB = X_LEG - 10.0 + M6_HOLE / 2 + W_MIN, X_S1 + CHEEK / 2   # back bolts where v11 had them: the spring keeps its well
BOLTS = [(x, y) for x in (X_BB, X_FB) for y in (Y_A + BOLT_IN_Y, Y_OUT - BOLT_IN_Y)]
X_W0 = X_BB + M6_HOLE / 2 + W_MIN               # spring well's back wall
R_O, R_M = (G_OUT + SP_T) / 2, (G_MID + SP_T) / 2
X_BK = X_W0 + SP_CLR + R_O + SP_T / 2           # back folds' centers
X_FR = X_S1 - SP_CLR - R_M - SP_T / 2           # front fold's center
X_E = X_C + 0.5                                 # arm 1 / arm 4 front ends (just past the nubs)

# Z stack: lane floor, slider body, spring (in its well, over the slider in the lane), lid
Z_SP = H_B - SP_CLR - SP_B                      # spring bottom = well floor
Z_BT = Z_SP - 0.4                               # slider body top
Z_STEM0 = Z_BT - STEM_H                         # stem bottom = top of the outboard wall's stop face


def box_xy():
    c = CORNER_LEG
    x0, y1, x1, y0 = XB, 0.0, XB + BOX_D, -BOX_W
    return Polygon([(x0, y1 - c), (x0 + c, y1), (x1 - c, y1), (x1, y1 - c), (x1, y0 + c), (x1 - c, y0), (x0 + c, y0), (x0, y0 + c)])


def opening(p, r):
    return p.buffer(-r, join_style="round").buffer(r, join_style="round")


def closing(p, r):
    return p.buffer(r, join_style="round").buffer(-r, join_style="round")


def polys(p):
    return [p] if p.geom_type == "Polygon" else [g for g in getattr(p, "geoms", []) if g.geom_type == "Polygon"]


def largest(p):
    return max(polys(p), key=lambda g: g.area)


def slide_path(dist=150.0):
    bar = bar_xy()
    pts = list(bar.exterior.coords)
    edges = [Polygon([p, q, (q[0] + dist, q[1]), (p[0] + dist, p[1])]).buffer(0) for p, q in zip(pts[:-1], pts[1:])]
    return unary_union([bar, affinity.translate(bar, dist, 0)] + edges)


def pocket_xy():
    blk = sbox(X_LEG - 1.0, BAR_C[1] - 22.0, X_S0, Y_A + 1.0)
    return blk.difference(box_xy().buffer(1.0)).difference(slide_path().buffer(FIT, join_style="mitre"))


def plan_xy():
    arm = sbox(X_PF, Y_A, X_END, Y_OUT)
    leg = sbox(X_PF, LEG_Y0, X_LEG, LEG_Y1)
    gus = Polygon([(X_LEG - 0.01, Y_A + 0.01), (X_LEG + 4.0, Y_A + 0.01), (X_LEG - 0.01, Y_A - 4.0)])
    plan = closing(opening(unary_union([arm, leg, gus, pocket_xy()]), R_OUT), R_IN)
    plan = plan.difference(slide_path().buffer(FIT, join_style="mitre"))
    return largest(opening(plan, W_MIN / 2 - 0.01))


def lane_xy():
    return sbox(X_S0, Y_A - 5.0, X_S1, Y_STOP)


def notch_xy():
    """The stem's way out through the outboard wall (from Z_STEM0 up; the lid tab closes it over the stem)."""
    return sbox(X_S0, Y_STOP - 0.01, X_S1, Y_OUT + 5.0)


def well_xy():
    """Spring well, behind the lane. Compressed, the W fans out at the back (its end arms tilt about their nubs), so
    the well runs out to a 3 mm outer wall past the lid tab's face."""
    return sbox(X_W0, Y1 - SP_T / 2 - 1.2, X_S0 + 0.01, Y_OUT - W_MIN)




def hexagon(x, y, af, flats_x):
    r = af / math.sqrt(3)
    a0 = 30.0 if flats_x else 0.0
    return Polygon([(x + r * math.cos(math.radians(a0 + 60 * i)), y + r * math.sin(math.radians(a0 + 60 * i))) for i in range(6)])


def nut_pocket_xy(x, y):
    """A closed hex, flats facing X."""
    return hexagon(x, y, NUT_AF, True)


def _pocket_y0():
    """Where the tube pocket's block starts along the leg (its inboard-most Y in front of the leg)."""
    return plan_xy().intersection(sbox(X_LEG + 0.5, -1e3, 1e3, 1e3)).bounds[1]


def lid_xy():
    return opening(plan_xy().intersection(sbox(X_PF, Y_A, X_END, Y_OUT)), R_OUT)   # just the block: a rounded rectangle


STUD_Z = H_B / 2
_SC = (Y_A + Y_OUT) / 2 - 0.3                   # centered on the block (-0.3: the inner washer clears the pocket block)
STUDS = (_SC - STUD_PITCH / 2, _SC + STUD_PITCH / 2)
LEG_Y0, LEG_Y1 = STUDS[0] - STUD_D / 2 - PLATE_END, STUDS[1] + STUD_D / 2 + PLATE_END
HOOK_C = Y_A + HOOK_W / 2                       # hook block's center
HOOK_STUDS = (HOOK_C - HOOK_PITCH / 2, HOOK_C + HOOK_PITCH / 2)
HOOK_Y0 = HOOK_STUDS[0] - HOOK_SLOT / 2 - STUD_D / 2 - PLATE_END
HOOK_Y1 = HOOK_STUDS[1] + HOOK_SLOT / 2 + STUD_D / 2 + PLATE_END
X_HF = X_T1 + 2.0                               # hook's front face


def groove_xz():
    """Relief along the back face (X-Z section) for the fittings' tops. Its top edge is a 45 deg slope (prints standing)."""
    z0, z1, d = STUD_Z - GROOVE_W / 2, STUD_Z + GROOVE_W / 2, GROOVE_D
    return Polygon([(X_PF - 1.0, z0), (X_PF + d, z0), (X_PF + d, z1 - d), (X_PF, z1), (X_PF - 1.0, z1)])


def hook_lip_xy():
    t = HOOK_LEAD
    return Polygon([(X_T0 + t, HOOK_TIP), (X_HF, HOOK_TIP), (X_HF, Y_A + 1.0), (X_T0, Y_A + 1.0), (X_T0, HOOK_TIP + t)])


def entry_path():
    """The hook-side tube sliding in sideways (from -Y) with the box pushed back against the leg."""
    bar = bar_xy()
    return unary_union([affinity.translate(bar, 0, -HOOK_ENTRY * i / 40) for i in range(41)])


def hook_xy():
    """Static hook, modeled in the latch's (right-corner) frame and mirrored for export: the leg behind the tube, a
    small block outboard of it, and a fixed lip over the tube's front face. Open on the inboard side for the tube
    to slide in."""
    blk = sbox(X_PF, Y_A, X_HF, Y_A + HOOK_W)
    leg = sbox(X_PF, HOOK_Y0, X_LEG, HOOK_Y1)
    gus = Polygon([(X_LEG - 0.01, Y_A + 0.01), (X_LEG + 4.0, Y_A + 0.01), (X_LEG - 0.01, Y_A - 4.0)])
    body = closing(opening(unary_union([blk, leg, gus, pocket_xy(), hook_lip_xy()]), R_OUT), R_IN)   # lip rounded with the block: no seam
    body = body.difference(unary_union([bar_xy(), entry_path()]).buffer(FIT, join_style="mitre"))
    return largest(opening(body, W_MIN / 2 - 0.01))


# ================================================================ spring (X-Y), beam model
def spring_path(step=0.6):
    """Centerline of the W, from arm 1's front end to arm 4's front end."""
    pts = []

    def line(a, b):
        n = max(2, int(math.dist(a, b) / step))
        pts.extend((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n))

    def arc(c, r, a0, a1):
        n = max(4, int(abs(math.radians(a1 - a0)) * r / step))
        pts.extend((c[0] + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)), c[1] + r * math.sin(math.radians(a0 + (a1 - a0) * i / n)))
                   for i in range(n))

    line((X_E, Y1), (X_BK, Y1))
    arc((X_BK, (Y1 + Y2) / 2), R_O, -90, -270)
    line((X_BK, Y2), (X_FR, Y2))
    arc((X_FR, (Y2 + Y3) / 2), R_M, -90, 90)
    line((X_FR, Y3), (X_BK, Y3))
    arc((X_BK, (Y3 + Y4) / 2), R_O, -90, -270)
    line((X_BK, Y4), (X_E, Y4))
    pts.append((X_E, Y4))
    return np.array(pts)


def _contacts(p):
    i1 = int(np.argmin(np.hypot(p[:, 0] - X_C, p[:, 1] - Y1)))
    i4 = int(np.argmin(np.hypot(p[:, 0] - X_C, p[:, 1] - Y4)))
    return i1, i4


def spring_fe(delta, b=SP_B):
    """Linear plane frame. Arm 4's nub is held (Y), arm 1's nub is pushed +Y by delta. Returns force (N), peak
    strain, and the displaced centerline."""
    p = spring_path()
    n = len(p)
    A, I = b * SP_T, b * SP_T ** 3 / 12
    K = np.zeros((3 * n, 3 * n))
    elems = []
    for e in range(n - 1):
        (x0, y0), (x1, y1) = p[e], p[e + 1]
        L = math.hypot(x1 - x0, y1 - y0)
        c, s = (x1 - x0) / L, (y1 - y0) / L
        a, k1, k2, k3 = E * A / L, 12 * E * I / L ** 3, 6 * E * I / L ** 2, E * I / L
        kl = np.array([[a, 0, 0, -a, 0, 0], [0, k1, k2, 0, -k1, k2], [0, k2, 4 * k3, 0, -k2, 2 * k3],
                       [-a, 0, 0, a, 0, 0], [0, -k1, -k2, 0, k1, -k2], [0, k2, 2 * k3, 0, -k2, 4 * k3]])
        r = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        Tm = np.kron(np.eye(2), r)
        dofs = [3 * e, 3 * e + 1, 3 * e + 2, 3 * e + 3, 3 * e + 4, 3 * e + 5]
        K[np.ix_(dofs, dofs)] += Tm.T @ kl @ Tm
        elems.append((dofs, kl, Tm))
    i1, i4 = _contacts(p)
    K[3 * i4 + 2, 3 * i4 + 2] += 1e-3 * E * I                     # pins the free spin about the line of the nubs
    fixed = {3 * i4: 0.0, 3 * i4 + 1: 0.0, 3 * i1 + 1: delta}
    fd = [d for d in range(3 * n) if d not in fixed]
    pd = list(fixed)
    up = np.array([fixed[d] for d in pd])
    u = np.zeros(3 * n)
    u[fd] = np.linalg.solve(K[np.ix_(fd, fd)], -K[np.ix_(fd, pd)] @ up)
    u[pd] = up
    force = (K[3 * i1 + 1] @ u)
    m_max = 0.0
    for dofs, kl, Tm in elems:
        f = kl @ (Tm @ u[dofs])
        m_max = max(m_max, abs(f[2]), abs(f[5]))
    eps = m_max * (SP_T / 2) / I / E
    # the spin about the nubs' line and the X shift are free here. The W and its load are mirror images about its
    # midline, so the real shape is too: spin it until the two nubs turn equal and opposite, and center it in X
    d = u.reshape(-1, 3)[:, :2]
    th = u.reshape(-1, 3)[:, 2]
    phi = -(th[i1] + th[i4]) / 2
    c = (p[i1] + p[i4]) / 2
    d = d + phi * np.column_stack([-(p[:, 1] - c[1]), p[:, 0] - c[0]])
    d[:, 0] -= d[:, 0].mean()
    return force, eps, p + d


def spring_xy(pts=None, nub_shift=(0.0, 0.0)):
    """Spring outline: the centerline (free, or displaced) thickened, plus the two nubs."""
    i1, i4 = _contacts(spring_path())
    p = spring_path() if pts is None else pts
    body = LineString(p).buffer(SP_T / 2, quad_segs=12)
    n1 = Point(p[i1][0], p[i1][1] - SP_T / 2).buffer(NUB1, quad_segs=16)
    n4 = Point(p[i4][0], p[i4][1] + SP_T / 2).buffer(NUB4, quad_segs=16)
    return unary_union([body, n1, n4])


# ================================================================ slider (Y-Z profile, locked; extruded X_T0..X_T1)
Y_K0 = Y_OUT + KNOB_GAP                         # knob's inboard face, locked
Y_BODY1 = Y_STOP - STROKE                       # the full-height body's outboard end, locked


Z_WEDGE0 = Z_FL + SILL_H + 0.3                  # wedge bottom (over the sill)
Y_SHOULDER = Y_A + SILL_L                       # the body's shoulder, locked: it sits on the sill's inner face


def slider_yz(handle=True):
    body = sbox(Y_SHOULDER, Z_FL, Y_BODY1, Z_BT)
    wedge = sbox(Y_TIP, Z_WEDGE0, Y_SHOULDER + 1.0, Z_BT)
    dog = sbox(Y_DOG - DOG_T, Z_BT - 0.5, Y_DOG, H_B - 0.4)
    stem = sbox(Y_BODY1 - 1.0, Z_STEM0, Y_K0 + BAR / 2, Z_BT)                    # buried past the handle's chamfers
    return unary_union([body, wedge, dog, stem] + ([handle_yz()] if handle else []))


def handle_yz():
    """D-handle: a closed loop off the stem. Gloved fingers hook through it from the tailgate side."""
    zc = H / 2
    outer = sbox(Y_K0, zc - HOLE_Z / 2 - BAR, Y_K0 + HOLE_Y + 2 * BAR, zc + HOLE_Z / 2 + BAR)
    outer = outer.buffer(-HANDLE_R, join_style="round").buffer(HANDLE_R, join_style="round", quad_segs=16)
    return outer.difference(outer.buffer(-BAR, join_style="round", quad_segs=16))


def ramp_xy():
    """What the 45 deg ramp cuts off the wedge (X-Y): the tube coming in from +X rides up it."""
    run = T - TIP_T
    return Polygon([(X_T0 + TIP_T, Y_TIP - 1.0), (X_T1 + 1.0, Y_TIP - 1.0), (X_T1 + 1.0, Y_TIP + run + 1.0), (X_T1, Y_TIP + run),
                    (X_T0 + TIP_T, Y_TIP)])


def wedge_xy(shift=0.0):
    """The slider's footprint over the tube (plan), shifted +Y by `shift` (retracting)."""
    w = sbox(X_T0, Y_TIP, X_T1, Y_BODY1).difference(ramp_xy())
    return affinity.translate(w, 0, shift)


# ================================================================ 3D (build123d)
def wire(coords, z):
    return Wire.make_polygon([Vector(x, y, z) for x, y in list(coords)[:-1]], close=True)


def slab(p, z0, z1):
    out = None
    for g in [q for q in polys(p) if q.area > 0.01]:
        f = Face(wire(g.exterior.coords, z0), [wire(h.coords, z0) for h in g.interiors])
        s = extrude(f, amount=z1 - z0, dir=(0, 0, 1))
        out = s if out is None else out + s
    return out


def yz_place(s, x0):
    """A solid modeled in XY (x=Y, y=Z, z=thickness) placed so its thickness runs along world +X from x0."""
    return Pos(x0, 0, 0) * Rot(0, 90, 90) * s


def z_rod(x, y, z0, z1, d):
    return Pos(x, y, z0) * Cylinder(d / 2, z1 - z0, align=None)


def bracket():
    log("[bracket] building")
    plan = plan_xy()
    body = slab(plan, 0, H_B)                                                     # the lid sits on the block only
    body -= slab(lane_xy(), Z_FL, H + 1)
    body += slab(sbox(X_S0 - 0.01, Y_A, X_S1 + 0.01, Y_SHOULDER), Z_FL - 0.01, Z_FL + SILL_H)
    body -= slab(well_xy(), Z_SP, H + 1)
    body -= slab(notch_xy(), Z_STEM0 - 0.3, H + 1)
    for x, y in BOLTS:
        body -= z_rod(x, y, -1, H + 1, M6_HOLE)
        pk = nut_pocket_xy(x, y)
        body -= slab(pk, -1, NUT_D)
        strip = pk.intersection(sbox(x - M6_HOLE / 2, -1e3, x + M6_HOLE / 2, 1e3))       # layer 1 bridges across Y
        body -= slab(strip, NUT_D - 0.01, NUT_D + BRIDGE)
        body -= slab(sbox(x - M6_HOLE / 2, y - M6_HOLE / 2, x + M6_HOLE / 2, y + M6_HOLE / 2), NUT_D, NUT_D + 2 * BRIDGE)  # layer 2
    return track_fit(body, STUDS, 0.0, LEG_Y0, LEG_Y1)


def track_fit(body, studs, travel, y0, y1):
    """The back-face groove and the stud holes (travel 0) or slots along the track."""
    body -= Pos(0, y1 + 1.0, 0) * Rot(90, 0, 0) * slab(groove_xz(), 0, y1 - y0 + 2.0)
    for y in studs:
        p = sbox(y - travel / 2, STUD_Z, y + travel / 2, STUD_Z + 0.001).buffer(STUD_D / 2, quad_segs=32)
        body -= yz_place(slab(p, 0, LEG_T + 2.0), X_PF - 1.0)
    return body


def hook():
    """In place, latch's frame (right corner)."""
    log("[hook] building")
    return track_fit(slab(hook_xy(), 0, H_B), HOOK_STUDS, HOOK_SLOT, HOOK_Y0, HOOK_Y1)


def hook_print():
    """Mirrored for the other rear corner (prints standing, as exported)."""
    return mirror(hook(), about=Plane.XZ)


def lid():
    """In place (top face up)."""
    log("[lid] building")
    s = slab(lid_xy(), H_B, H)
    x0, x1 = X_S0 + 0.25, X_S1 - 0.25
    s += slab(sbox(x0, Y_A, x1, Y_A + TAB_T), Z_SP, H_B + 0.01)                   # closes the mouth over the wedge
    s += slab(sbox(x0, Y_STOP, x1, Y_OUT), Z_SP, H_B + 0.01)                      # over the knob notch: spring seat
    lead = Polygon([(Y_STOP - 1.0, Z_SP - 1.0), (Y_STOP + LEAD + 1.0, Z_SP - 1.0), (Y_STOP - 1.0, Z_SP + LEAD + 1.0)])
    s -= yz_place(slab(lead, 0, X_S1 - X_S0 + 2.0), X_S0 - 1.0)
    for x, y in BOLTS:
        s -= z_rod(x, y, H_B - 1, H + 1, M6_HOLE)
    return s


def lid_print():
    return Pos(0, 0, H) * Rot(180, 0, 0) * lid()


def slider_print():
    """Lying on its latch face (-X): profile in x=Y, y=Z, thickness up. The ramp, then the D-handle (as wide as the
    block's front face, its front and back edges chamfered)."""
    log("[slider] building")
    s = slab(slider_yz(handle=False), 0, T)
    rmp = Polygon([(y, x - X_T0) for x, y in ramp_xy().exterior.coords])           # (Y, thickness)
    s -= Pos(0, Z_BT + 1.0, 0) * Rot(90, 0, 0) * slab(rmp, 0, Z_BT - Z_FL + 2.0)
    return s + chamfered_slab(handle_yz(), 0, X_END - X_T0, HANDLE_CH)


def chamfered_slab(p, z0, z1, c):
    """Extrude with a 45 deg chamfer on both end faces' edges (outer and hole). The bed-side one is a 45 deg slope."""
    f0 = Face(wire(p.exterior.coords, z0 + c), [wire(h.coords, z0 + c) for h in p.interiors])
    f1 = Face(wire(p.exterior.coords, z1 - c), [wire(h.coords, z1 - c) for h in p.interiors])
    return (extrude(f0, amount=c, dir=(0, 0, -1), taper=45) + slab(p, z0 + c - 0.001, z1 - c + 0.001)
            + extrude(f1, amount=c, dir=(0, 0, 1), taper=45))


def slider(shift=0.0):
    return Pos(0, shift, 0) * yz_place(slider_print(), X_T0)


def spring_print():
    log("[spring] building")
    return slab(spring_xy(), 0, SP_B)


def spring_at(delta):
    _, _, pts = spring_fe(delta)
    return slab(spring_xy(pts), Z_SP, Z_SP + SP_B)


# ================================================================ checks
def thin_report(name, p):
    thin = p.difference(opening(p, W_MIN / 2 - 0.05))
    bits = [q for q in polys(thin) if q.area > 0.8]
    log(f"  {name:28s} features under {W_MIN:.0f} mm: {len(bits)}" + "".join(
        f"\n      {q.area:.1f} mm2 at ({q.centroid.x:.1f}, {q.centroid.y:.1f})" for q in bits))
    return not bits


def spring_clearances(pts):
    """Closest approach (mm, surface to surface) of the displaced spring to itself (parts > one fold apart along it),
    to the dog's face, to the lid tab's face, and to the well's walls. The nubs are the only intended contacts."""
    p = np.asarray(pts)
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
    i1, i4 = _contacts(spring_path())
    self_min = 1e9
    for i in range(len(p)):
        far = np.abs(s - s[i]) > math.pi * R_O + 2.0
        if far.any():
            self_min = min(self_min, np.min(np.hypot(*(p[far] - p[i]).T)) - SP_T)
    dog_face = p[i1][1] - SP_T / 2 - NUB1
    stop_face = p[i4][1] + SP_T / 2 + NUB4
    lane = (p[:, 0] > X_T0 - SP_T / 2)
    near1 = np.abs(s - s[i1]) > NUB1 + 0.8
    near4 = np.abs(s - s[i4]) > NUB4 + 0.8
    dog = np.min(p[lane & near1, 1] - SP_T / 2 - dog_face)
    tab = np.min(stop_face - (p[lane & near4, 1] + SP_T / 2))
    room = unary_union([well_xy(), sbox(X_S0, -1e3, X_S1, 1e3)])             # in the lane: dog + tab faces, above
    sp = LineString(p).buffer(SP_T / 2, quad_segs=8)
    w = sp.distance(room.exterior) if sp.within(room) else -sp.difference(room).area
    xs = float(p[:, 0].max() + SP_T / 2)
    return self_min, dog, tab, w, xs


def check():
    ok = True
    bar, box_p, plan = bar_xy(), box_xy(), plan_xy()
    log(f"[check] tube +X face x={X_FACE:.1f}, y {Y_FACE0:.1f}..{Y_FACE1:.1f}; slider x {X_T0:.1f}..{X_T1:.1f}, cheek to x={X_END:.1f}")
    hy = handle_yz().bounds
    log(f"[check] block {X_END - X_PF:.0f} (X) x {Y_OUT - Y_A:.1f} (Y) x {H:.0f} (Z, lid {LID_T:.0f}); handle out to y={hy[2]:.1f} "
        f"({hy[2] - Y_OUT:.0f} past the block), {hy[3] - hy[1]:.0f} tall")
    log(f"[check] wedge: tip y={Y_TIP} (covers {Y_FACE1 - Y_TIP:.1f} of the {Y_FACE1 - Y_FACE0:.1f} mm face), {Z_BT - Z_WEDGE0:.1f} tall, "
        f"45 deg ramp {TIP_T:.0f} -> {T:.0f} thick; stroke to clear the tube {TRAVEL:.1f}, knob stroke {STROKE:.1f}")
    ok &= Y_TIP + TRAVEL >= BAND_TOP + RETRACT_CLR - 1e-6 and Y_TIP + STROKE < Y_A + 2.0
    stop_h = Z_STEM0 - 0.3 - Z_FL
    log(f"[check] stops: out = the body's shoulder on the mouth sill; in = the body's end on the outboard wall under the stem "
        f"({stop_h:.1f} x {T:.0f} mm face, bracket material); body ends y={Y_BODY1:.1f} locked (dog to y={Y_DOG:.1f})")
    ok &= stop_h >= W_MIN and Y_BODY1 > Y_DOG + 3.0 and LEAD > PRE + 0.4
    log(f"[check] out stop: the body's shoulder on a {SILL_L:.0f} x {SILL_H:.0f} sill across the mouth (bracket); dog clears the lid tab by {DOG_GAP}, "
        f"floor {Z_FL:.0f} inside / {Z_FL + SILL_H:.0f} at the mouth")
    log(f"[check] handle: {KNOB_GAP:.0f} mm off the block locked, finger hole {HOLE_Y:.0f} x {HOLE_Z:.0f}, {X_END - X_T0:.0f} wide, "
        f"bar {BAR:.0f} all round, corners R{HANDLE_R:.0f}/R{HANDLE_R - BAR:.0f}; stem {STEM_H:.0f} x {T:.0f}")

    # ---- spring
    f_pre, e_pre, p_pre = spring_fe(PRE)
    f_max, e_max, p_max = spring_fe(DMAX)
    f_ins, _, _ = spring_fe(PRE + TRAVEL)
    k = f_pre / PRE
    log(f"[check] spring: {SP_T} x {SP_B} arms, back folds at x={X_BK - R_O - SP_T / 2:.1f}, nubs at x={X_C}, "
        f"{(Y_STOP - Y1 + SP_T / 2 + NUB1):.1f} long free; {k:.2f} N/mm")
    log(f"    locked (preload {PRE}): {f_pre:.1f} N, strain {e_pre * 100:.2f}%;  knob pulled ({DMAX:.1f}): {f_max:.1f} N, "
        f"strain {e_max * 100:.2f}% (limit {EPS_MAX * 100:.1f}%)")
    ok &= e_max < EPS_MAX
    for name, pts in (("free", spring_path()), ("locked", p_pre), ("pulled", p_max)):
        s_min, dog, tab, w, xs = spring_clearances(pts)
        log(f"    {name:6s}: arms to each other {s_min:.2f}, arm 1 to the dog {dog:.2f}, arm 4 to the tab {tab:.2f}, "
            f"to the well walls {w:.2f}, front x={xs:.1f} (lane wall {X_S1:.1f})")
        if name != "free":
            ok &= s_min > 0.3 and dog > 0.2 and tab > 0.2 and w > 0.2 and xs < X_S1 - 0.2
    ramp = math.radians(45.0)
    push = f_ins * (math.tan(ramp) + MU) / (1 - MU * math.tan(ramp)) + 2 * MU * f_ins
    log(f"    box push to latch ~{push:.0f} N peak (ramp + friction, spring {f_ins:.0f} N at the tube's edge); "
        f"knob pull {f_pre:.0f} -> {f_max:.0f} N")

    # ---- strength (box yanked out at F)
    F = 1000.0
    hw = Z_BT - Z_WEDGE0
    arm = Y_A - (Y_TIP + Y_FACE1) / 2
    s_mouth = 6 * F * arm / (hw * T ** 2)
    tau = 1.5 * F / (hw * T)
    s_cheek = 6 * (F * H_B / 4) / ((Y_OUT - Y_A) * CHEEK ** 2)
    log(f"[check] box yanked at {F:.0f} N: wedge {s_mouth:.0f} MPa at the mouth, {tau:.1f} MPa shear across its layers; "
        f"cheek ~{s_cheek:.0f} MPa; floor + lid each ~{F / 2:.0f} N in tension")
    ok &= s_mouth < 25 and tau < 10

    # ---- fit, play, paths
    ret = wedge_xy(TRAVEL)
    log(f"[check] retracted wedge to the tube's path: {ret.distance(slide_path().buffer(FIT, join_style='mitre')):.2f} mm")
    ok &= not ret.intersects(slide_path())
    for name, a, b in (("wedge locked vs tube", wedge_xy(), bar), ("bracket vs tube", plan, bar), ("bracket vs box", plan, box_p),
                       ("wedge vs box", wedge_xy(), box_p)):
        ov = a.intersection(b).area
        log(f"  {name:28s} overlap {ov:6.2f} mm2, gap {a.distance(b):.2f} mm")
        ok &= ov < 0.05
    walls = unary_union([plan.difference(lane_xy()), wedge_xy()])

    def first_hit(move, steps):
        return next((d for d in steps if move(bar, d).intersection(walls).area > 0.02), None)

    trans = {a: first_hit(lambda b, d, a=a: affinity.translate(b, d * math.cos(math.radians(a)), d * math.sin(math.radians(a))),
                          [0.05 * i for i in range(1, 201)]) for a in range(0, 360, 45)}
    twist = {s: first_hit(lambda b, d, s=s: affinity.rotate(b, s * d, origin=BAR_C), [0.25 * i for i in range(1, 121)]) for s in (1, -1)}
    fmt = lambda v: "FREE" if v is None else f"{v:.2f}"
    log("[check] tube free play, locked (deg: mm): " + "  ".join(f"{a}:{fmt(v)}" for a, v in trans.items())
        + f"; twist +{fmt(twist[1])}/-{fmt(twist[-1])} deg")
    ok &= all(v is not None for v in trans.values()) and all(v is not None for v in twist.values())

    # ---- bolts + walls
    cav = unary_union([lane_xy(), well_xy()])
    bolt_ok = all(plan.contains(Point(x, y).buffer(M6_HOLE / 2 + W_MIN - 0.01)) and
                  Point(x, y).distance(cav) >= M6_HOLE / 2 + W_MIN - 0.01 for x, y in BOLTS)
    nut_ok = all(plan.contains(nut_pocket_xy(x, y).buffer(W_MIN - 0.01)) and
                 nut_pocket_xy(x, y).distance(lane_xy()) >= W_MIN - 0.01 for x, y in BOLTS)
    log(f"[check] 4 x M6 at " + ", ".join(f"({x:.1f}, {y:.1f})" for x, y in BOLTS) + f": shank walls {'>= 3' if bolt_ok else 'THIN'}, "
        f"nut pockets {'>= 3' if nut_ok else 'THIN'}; M6 x 60 cap screw over {H:.0f} mm: ends {H - 60:+.1f} from the bottom face, "
        f"nut pocket {NUT_D:.0f} deep (plain nut)")
    ok &= nut_ok and abs(H - 60.0) < 0.6
    ok &= track_check(plan, STUDS, 0.0, Y_OUT, "latch")
    ok &= track_check(hook_xy(), HOOK_STUDS, HOOK_SLOT, Y_A + HOOK_W, "hook")
    tip = (XB - X_PF) - (STUD_H + GROOVE_D)
    thread = STUD_H - LEG_T - 2.5 - NUT38_T
    log(f"[check] L-track studs: back face to the box {XB - X_PF:.1f}; a 30 mm stud (+{GROOVE_D} if a fitting stands proud into the groove) "
        f"clears the box by {tip:.1f}; past a washer + nut {thread:+.1f} of thread (>= 0: full nut)")
    ok &= tip >= 2.0 and thread >= 0.0
    ok &= hook_check()
    ok &= bolt_ok
    wpk = well_xy().distance(slide_path().buffer(FIT, join_style="mitre"))
    log(f"[check] spring well to the tube pocket: {wpk:.1f} mm of wall")
    ok &= wpk >= W_MIN
    log(f"[check] printability: nothing under {W_MIN:.0f} mm")
    ok &= thin_report("bracket at the slider", plan.difference(lane_xy()))
    ok &= thin_report("bracket at the stem", plan.difference(lane_xy()).difference(notch_xy()))
    ok &= thin_report("bracket at the spring", plan.difference(lane_xy()).difference(well_xy()))
    ok &= thin_report("lid", lid_xy())
    ok &= thin_report("slider (side)", slider_yz())
    ok &= thin_report("slider (plan)", wedge_xy())
    ok &= thin_report("spring", spring_xy())
    ok &= thin_report("hook", hook_xy())
    log(f"[check] {'OK' if ok else 'PROBLEMS ABOVE'}")
    return ok


def track_check(plan, studs, travel, y_block1, name):
    """Washers clear the tube pocket's block and the block's far face; plate symmetric about the holes."""
    y_pb = plan.intersection(sbox(X_LEG + 0.5, -1e3, 1e3, Y_A)).bounds[1]
    near = y_pb - (studs[0] + travel / 2 + WASHER_R)
    far = (studs[1] - travel / 2 - WASHER_R) - y_block1
    log(f"[check] {name}: studs at y={studs[0]:.1f} / {studs[1]:.1f} ({(studs[1] - studs[0]) / IN:.0f}\" apart"
        + (f", slots +-{travel / 2:.0f}" if travel else ", round holes") + f"), z={STUD_Z:.0f}; washers clear the tube pocket block by "
        f"{near:.1f}, the block's far face by {far:.1f}")
    return near >= 1.0 and far >= 1.0


def hook_check():
    """The hook holds the tube back, forward and outboard; it's open inboard (that's how the tube goes in, and the
    latch at the other corner holds that way). The tube must be able to turn a little while the box swings in."""
    walls = hook_xy()
    bar = bar_xy()

    def first_hit(move, steps):
        return next((d for d in steps if move(bar, d).intersection(walls).area > 0.02), None)

    trans = {a: first_hit(lambda b, d, a=a: affinity.translate(b, d * math.cos(math.radians(a)), d * math.sin(math.radians(a))),
                          [0.05 * i for i in range(1, 401)]) for a in (0, 90, 180, 270)}
    twist = {s: first_hit(lambda b, d, s=s: affinity.rotate(b, s * d, origin=BAR_C), [0.25 * i for i in range(1, 41)]) for s in (1, -1)}
    fmt = lambda v: "free" if v is None else f"{v:.2f}"
    lip = hook_lip_xy().intersection(sbox(0, 0, 1e3, Y_FACE1)).bounds
    log(f"[check] hook: tube play fwd {fmt(trans[0])} / outboard {fmt(trans[90])} / back {fmt(trans[180])} / inboard {fmt(trans[270])} "
        f"(should be free); turns +{fmt(twist[1])}/-{fmt(twist[-1])} deg; lip bears on the face y {HOOK_TIP + HOOK_LEAD:.1f}..{Y_FACE1:.1f}")
    return all(trans[a] is not None for a in (0, 90, 180)) and trans[270] is None and min(twist[1] or 9, twist[-1] or 9) >= 2.0


def main():
    OUT.mkdir(exist_ok=True)
    check()
    parts = (("bracket", bracket), ("lid", lid_print), ("slider", slider_print), ("spring", spring_print), ("hook", hook_print),
             ("lid_in_place", lid), ("slider_in_place", slider), ("spring_locked", lambda: spring_at(PRE)),
             ("spring_pulled", lambda: spring_at(DMAX)), ("slider_pulled", lambda: slider(STROKE)))
    for name, fn in parts:
        s = fn()
        export_stl(s, str(OUT / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.15)
        if "_" not in name:
            export_step(s, str(OUT / f"{name}.step"))
        log(f"  -> out/{name}.stl  volume {s.volume / 1000:.1f} cm3, ~{s.volume * RHO * 1000:.0f} g")


if __name__ == "__main__":
    check() if "--check" in sys.argv else main()
