"""3D views of v12 to v12/renders/. Run after latch.py:  uv run python v12/render.py"""

import sys
from pathlib import Path

import numpy as np
import pyvista as pv

sys.path.insert(0, str(Path(__file__).parent))
import latch as M  # noqa: E402
from shapely.geometry import Polygon  # noqa: E402
M.Polygon = Polygon

pv.OFF_SCREEN = True
OUT = M.HERE / "renders"
BLACK, BLACK2, SLIDE, SPRING, ALU, BOX, STEEL = "#26272b", "#1b1c1f", "#d7261e", "#f08a24", "#c9cdd2", "#c8102e", "#9aa0a6"


def load(name):
    return pv.read(str(M.OUT / f"{name}.stl"))


def prism(poly, z0, z1):
    pts = np.array(poly.exterior.coords)[:-1]
    n = len(pts)
    faces = [n] + list(range(n)) + [n] + list(range(n, 2 * n))
    for i in range(n):
        j = (i + 1) % n
        faces += [4, i, j, n + j, n + i]
    return pv.PolyData(np.vstack([np.column_stack([pts, np.full(n, z0)]), np.column_stack([pts, np.full(n, z1)])]), faces).triangulate()


def scene(pulled=False, lid=True, box=True, beauty=False, hook=False):
    s = "pulled" if pulled else "locked"
    items = [(load("bracket"), BLACK, {})]
    if lid:
        items.append((load("lid_in_place"), BLACK, {}))
    items.append((load(f"slider_{'pulled' if pulled else 'in_place'}"), BLACK2 if beauty else SLIDE, {}))
    if not lid:
        items.append((load(f"spring_{s}"), SPRING, {}))
    if lid:
        for x, y in M.BOLTS:
            head = pv.Cylinder(center=(x, y, M.H + M.HEAD_K / 2), direction=(0, 0, 1), radius=M.HEAD_D / 2, height=M.HEAD_K, resolution=48)
            items.append((head, "#3a3c40", dict(specular=0.6)))
            items.append((pv.Cylinder(center=(x, y, M.H + M.HEAD_K + 0.02), direction=(0, 0, 1), radius=2.9, height=0.05, resolution=6),
                           "#121315", {}))                                                  # hex socket (drawn, not cut)
    items += track_items(M.STUDS)
    bar = M.bar_xy()
    items.append((prism(bar, -90, 250), ALU, dict(specular=0.8, specular_power=40)))
    if box:
        items.append((prism(M.box_xy(), -90, 250), BOX, dict(opacity=0.35) if hook else {}))
    if hook:
        items += hook_items()
    return items


TRACK_T, TRACK_W = 9.0, 31.75                   # L-track rail (guess), on the wall behind the brackets


def track_items(studs, ymirror=False):
    """Studs (3/8", 30 mm) with a washer + nut on the leg's front face."""
    out = []
    for y in studs:
        y = -M.BOX_W - y if ymirror else y
        out.append((pv.Cylinder(center=(M.X_PF + M.STUD_H / 2, y, M.STUD_Z), direction=(1, 0, 0), radius=4.75, height=M.STUD_H, resolution=32),
                    STEEL, {}))
        out.append((pv.Cylinder(center=(M.X_LEG + 1.25, y, M.STUD_Z), direction=(1, 0, 0), radius=M.WASHER_R, height=2.5, resolution=48),
                    STEEL, {}))
        out.append((pv.Cylinder(center=(M.X_LEG + 2.5 + M.NUT38_T / 2, y, M.STUD_Z), direction=(1, 0, 0), radius=8.2, height=M.NUT38_T,
                                resolution=6), "#4a4d52", {}))
    return out


def hook_items():
    """The hook at the box's other rear corner (hook.stl is already mirrored about y=0)."""
    hk = load("hook").translate((0, -M.BOX_W, 0), inplace=False)
    bar = M.Polygon([(x, -M.BOX_W - y) for x, y in M.bar_xy().exterior.coords])
    return [(hk, BLACK, {}), (prism(bar, -90, 250), ALU, dict(specular=0.8, specular_power=40))] + track_items(M.HOOK_STUDS, ymirror=True)


def shot(name, items, cam, parallel=False, scale=None):
    p = pv.Plotter(off_screen=True, window_size=(1600, 1100))
    p.set_background("#d9dde2", top="#f7f8fa")
    for mesh, color, kw in items:
        p.add_mesh(mesh, color=color, smooth_shading=True, split_sharp_edges=True, **kw)
    p.add_mesh(pv.Plane(center=(M.X_PF - TRACK_T - 0.5, -250, 50), direction=(1, 0, 0), i_size=600, j_size=1000), color="#3b3e44")
    p.add_mesh(pv.Box(bounds=(M.X_PF - TRACK_T, M.X_PF, -M.BOX_W - 250, 250, M.STUD_Z - TRACK_W / 2, M.STUD_Z + TRACK_W / 2)),
               color="#7d8288", specular=0.5)
    p.enable_anti_aliasing("ssaa")
    p.camera_position = cam
    if parallel:
        p.enable_parallel_projection()
        p.camera.parallel_scale = scale
    p.add_light(pv.Light(position=(600, 400, 600), intensity=0.45))
    OUT.mkdir(exist_ok=True)
    p.screenshot(str(OUT / f"{name}.png"))
    p.close()
    print(f"  -> renders/{name}.png", flush=True)


def BOXW():
    return M.BOX_W


def main():
    c = (55, 35, 20)
    shot("01_black", scene(beauty=True, box=False), [(300, 260, 190), (55, 40, 20), (0, 0, 1)])
    shot("12_box_both_corners", scene(beauty=True, hook=True), [(700, -277, 900), (60, -277, 20), (0, 0, 1)])
    shot("13_hook", scene(beauty=True, box=False, hook=True), [(300, -BOXW() - 240, 190), (55, -BOXW() - 30, 20), (0, 0, 1)])
    shot("14_hook_with_box", scene(beauty=True, hook=True), [(320, -BOXW() - 220, 230), (55, -BOXW() - 20, 20), (0, 0, 1)])
    shot("02_locked", scene(), [(330, 250, 210), c, (0, 0, 1)])
    shot("03_pulled", scene(pulled=True), [(330, 250, 210), c, (0, 0, 1)])
    shot("04_inside_locked", scene(lid=False, box=False), [(150, 60, 260), (52, 40, 20), (0, 0, 1)])
    shot("05_inside_pulled", scene(pulled=True, lid=False, box=False), [(150, 60, 260), (52, 40, 20), (0, 0, 1)])
    shot("06_from_tailgate", scene(beauty=True), [(400, 20, 60), (50, 15, 22), (0, 0, 1)])


if __name__ == "__main__":
    main()
