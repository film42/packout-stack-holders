"""Top sections of v14 at the spring's height: locked / box going in / knob pulled.
Run: uv run python v14/plan.py"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from shapely import affinity  # noqa: E402
from shapely.geometry import Point, box as sbox  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
import latch as M  # noqa: E402


def draw(ax, p, **kw):
    for g in M.polys(p):
        x, y = g.exterior.xy
        ax.fill(x, y, **kw)
        for h in g.interiors:
            hx, hy = h.xy
            ax.fill(hx, hy, color="white")


def main():
    out = M.HERE / "renders"
    out.mkdir(exist_ok=True)
    plan = M.plan_xy()
    section = plan.difference(M.lane_xy()).difference(M.well_xy())
    tabs = [sbox(M.X_S0, M.Y_A, M.X_S1, M.Y_A + M.TAB_T), sbox(M.X_S0, M.Y_STOP, M.X_S1, M.Y_OUT), sbox(M.X_S0, M.Y_A, M.X_S1, M.Y_SHOULDER)]
    dog = sbox(M.X_T0, M.Y_DOG - M.DOG_T, M.X_T1, M.Y_DOG)
    hb = M.handle_yz().bounds
    knob = sbox(M.X_T0, hb[0], M.X_END, hb[2])
    stem = sbox(M.X_T0, M.Y_BODY1, M.X_T1, M.Y_K0)
    fig, axs = plt.subplots(1, 3, figsize=(24, 10))
    cases = (("LOCKED: the wedge sits on the tube's front face", 0.0, M.PRE, 0.0),
             ("BOX GOING IN: the tube rides the ramp and pushes the wedge out", M.TRAVEL - 0.6, M.PRE + M.TRAVEL - 0.6, 14.0),
             ("KNOB PULLED: the body bottoms on the outboard wall, the box slides out", M.STROKE, M.DMAX, 25.0))
    for ax, (name, shift, d, tube_dx) in zip(axs, cases):
        draw(ax, section, color="#2a2b2f", alpha=0.85)
        for t in tabs:
            draw(ax, t, color="#555a62", alpha=0.9)
        draw(ax, affinity.translate(M.bar_xy(), tube_dx, 0), color="#c9cdd2", ec="#888")
        draw(ax, affinity.translate(M.box_xy(), tube_dx, 0).intersection(sbox(0, -80, 200, 0)), color="#c8102e", alpha=0.35)
        for g in (M.wedge_xy(shift), affinity.translate(stem, 0, shift), affinity.translate(knob, 0, shift)):
            draw(ax, g, color="#d7261e", alpha=0.55)
        draw(ax, affinity.translate(dog, 0, shift), color="#d7261e", alpha=0.95)
        _, eps, pts = M.spring_fe(d)
        draw(ax, M.spring_xy(pts), color="#f08a24", alpha=0.95)
        for x, y in M.BOLTS:
            draw(ax, Point(x, y).buffer(M.M6_HOLE / 2), color="#9aa0a6", ec="k")
        ax.set_aspect("equal")
        ax.set_xlim(15, 105)
        ax.set_ylim(-12, 95)
        ax.grid(alpha=0.3)
        ax.set_xlabel("X: out from the cab wall, toward the tailgate ->  (mm)")
        ax.set_ylabel("Y: along the wall, outboard ->  (mm)")
        ax.set_title(f"v14 {name}\nbracket (black) + lid tabs (grey), slider (red: dog solid), spring (orange, strain {eps * 100:.1f}%), "
                     f"tube (silver), box (red)", fontsize=9)
    fig.savefig(out / "00_plan.png", dpi=75, bbox_inches="tight")
    print("  -> renders/00_plan.png", flush=True)


if __name__ == "__main__":
    main()
