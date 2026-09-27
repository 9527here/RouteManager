#!/usr/bin/env python3
"""Generate simplified collision geometry for the humanoid upper body v3
(the 人形上装部分9.9 export: same dual-arm torso + wheeled base as v2,
re-exported 2026-09-09).

The SolidWorks export reuses full-resolution visual STLs as collision meshes
(>6M triangles total).  PhysX cannot handle that: convex decomposition either
takes minutes per link or blows past the 255-vertex-per-hull limit.

Strategy per link, chosen from the measured convex-hull fill ratio
(mesh volume / hull volume) so that no shape is grossly inflated:

  box-like, fill < 0.25   ->  analytic boxes, slab-decomposed along Z
                              base_link (0.061), SX_Link (0.115), XQ (0.141)
  wheels                  ->  analytic cylinder (exact, best contact behaviour)
  fill >= HULL_FILL_MIN   ->  one decimated convex hull
  fill <  HULL_FILL_MIN   ->  several convex hulls, one per slab along the
                              link's longest axis.  A single hull on these
                              concave parts inflates volume up to 5x, which
                              would make the grippers collide with objects
                              before the real geometry does.

base_link is special-cased.  The four castor units hang from it on their own
joints, and the wheels sit in pockets recessed into the chassis silhouette
(wheel solid spans y +-[0.200, 0.294], x +-[0.091, 0.409] below the deck).
A plain hull or full-width slab boxes would swallow that volume: wheel ZQL is
a *grandchild* of base_link, so PhysX does NOT filter that pair, and the wheel
would collide with its own pocket the moment it rolls.  The decomposition
below therefore omits the pockets: below the deck only three boxes per band
survive (centre strip |y| <= 0.19, front/rear bumper strips |x| >= 0.415).
The pocket walls themselves are covered by the castor-bracket hulls, which are
adjacent to base_link and filtered.  The small intrusion measured inside the
wheel cylinder is the axle boss at the wheel's outer face (x +-[0.218, 0.282],
y +[0.2815, 0.2935], z [-0.203, -0.137]) - it mates with the wheel hub bore
and is deliberately not modelled.

Measured wheel geometry (identical for the four units, left/right mirrored):
  wheel r = 0.1592 m, width 0.0936 m, axle z = -0.1701 in base_link frame
  -> ground contact at z = -0.3293.

Outputs:
  meshes/collision/*.STL   convex hulls (possibly several per link)
  collision_shapes.json    full collision spec, consumed by build_isaac_urdf.py
"""

from __future__ import annotations

import json
import pathlib

import numpy as np
import open3d as o3d
import trimesh

PKG = pathlib.Path(__file__).resolve().parent.parent
VISUAL = PKG / "meshes" / "visual"
COLLISION = PKG / "meshes" / "collision"
SPEC_OUT = PKG / "collision_shapes.json"

# Links whose collision is analytic boxes, with explicit Z breakpoints.
# Breakpoints follow measured geometry transitions, not a uniform split.
SLAB_LINKS = {
    # lift carriage: thin guide rail, then the top platform.  9.9 re-anchored
    # this link's frame onto the lift axis (origin moved to z=1.1362 in the
    # v2 mesh frame), so the v2 breaks [0.02, 0.89, 1.11] shift down by
    # exactly that amount; XQ's world pose is unchanged.
    "SX_Link": [-1.1162, -0.2462, -0.0262],
    # torso: barrel, shoulder yoke, neck  (9.9 mesh identical to v2's)
    "XQ": [-0.093, 0.071, 0.188, 0.323],
}

# Wheels: exact cylinders.  Circular cross-section lies in XZ, axis along Y,
# so the URDF cylinder (default axis +Z) needs rpy="pi/2 0 0".
WHEEL_LINKS = ("ZQL", "ZHL", "YQL", "YHL")

# Chassis decomposition (measured, see module docstring).
CHASSIS = {
    # below-deck bands, in wheel-pocket country
    "low_bands": [(-0.28, -0.23), (-0.23, -0.18), (-0.18, -0.13),
                  (-0.13, -0.08), (-0.08, -0.03), (-0.03, -0.005)],
    "mid_y": 0.19,      # centre strip kept: wheel inner face is at 0.1999
    "strip_x": 0.415,   # bumper strips kept: wheel tips reach x +-0.4092
    # deck: full-width slabs (wheels end at z=-0.0108, 5 mm clearance below)
    "deck_bands": [(-0.005, 0.07), (0.07, 0.15), (0.15, 0.225)],
    # above the deck only the thin guide column and its wider top remain
    # (top band stops at 1.065: the XQ torso column slab starts at z=1.075
    # and base_link is NOT joint-adjacent to XQ (SX_Link sits between), so
    # PhysX would not filter an overlap there - it must not exist)
    "mast_bands": [(0.225, 0.27), (0.27, 0.92), (0.92, 1.02), (1.02, 1.065)],
}

HULL_LINKS = (
    # castor brackets (one per corner)
    "ZQY", "ZHY", "YQY", "YHY",
    # head
    "TZ", "TF",
    # arms
    "Z1", "Z2", "Z3", "Z4", "Z5", "Z6", "Z7", "ZJ",
    "Y1", "Y2", "Y3", "Y4", "Y5", "Y6", "Y7", "YJ",
)

HULL_MAX_FACES = 128     # per hull; PhysX caps a convex at 255 vertices
HULL_FILL_MIN = 0.45     # below this, split into slab hulls


def load(link: str) -> trimesh.Trimesh:
    return trimesh.load(VISUAL / f"{link}.STL")


def box_from_points(pts: np.ndarray) -> dict:
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return {
        "type": "box",
        "size": [round(float(v), 6) for v in (hi - lo)],
        "origin": [round(float(v), 6) for v in (hi + lo) / 2.0],
    }


def slab_boxes(link: str, breaks: list[float]) -> list[dict]:
    v = load(link).vertices
    out = []
    for lo, hi in zip(breaks[:-1], breaks[1:]):
        band = v[(v[:, 2] >= lo - 1e-9) & (v[:, 2] <= hi + 1e-9)]
        if len(band) < 4:
            continue
        b = box_from_points(band)
        # Snap Z to the requested breakpoints so adjacent slabs meet exactly.
        b["size"][2] = round(hi - lo, 6)
        b["origin"][2] = round((hi + lo) / 2.0, 6)
        b["name"] = f"slab{len(out)}"
        out.append(b)
    return out


def base_link_boxes() -> list[dict]:
    """Chassis slabs that steer clear of the four wheel pockets.

    Below the deck each band yields up to three boxes: the centre strip
    (|y| <= mid_y, clear of the wheel inner faces at +-0.1999) and the
    front/rear bumper strips (|x| >= strip_x, clear of the wheel tips at
    +-0.4092).  The pocket volume |y| in [0.19, 0.30] x |x| in [0.085, 0.415]
    below z = -0.005 is deliberately left collision-free - see module
    docstring.
    """
    v = load("base_link").vertices
    boxes = []
    c = CHASSIS

    def band(lo, hi, keep=None, name=""):
        """One slab box; keep(pts) filters which vertices may contribute."""
        m = (v[:, 2] >= lo) & (v[:, 2] <= hi)
        if keep is not None:
            m &= keep(v)
        q = v[m]
        if len(q) < 4:
            return
        b = box_from_points(q)
        b["size"][2] = round(hi - lo, 6)
        b["origin"][2] = round((hi + lo) / 2.0, 6)
        b["name"] = name
        boxes.append(b)

    # below-deck bands: centre strip + bumper strips, pockets omitted
    for lo, hi in c["low_bands"]:
        band(lo, hi, lambda p: np.abs(p[:, 1]) <= c["mid_y"], f"under_y{lo:+.2f}")
        band(lo, hi, lambda p: p[:, 0] >= c["strip_x"], f"bumper_xp_{lo:+.2f}")
        band(lo, hi, lambda p: p[:, 0] <= -c["strip_x"], f"bumper_xm_{lo:+.2f}")

    # deck and up: full-width boxes
    for lo, hi in (*c["deck_bands"], *c["mast_bands"]):
        band(lo, hi, None, f"chassis_{lo:+.2f}")

    return boxes


def wheel_cylinder(link: str) -> list[dict]:
    v = load(link).vertices
    radius = float(np.sqrt(v[:, 0] ** 2 + v[:, 2] ** 2).max())
    y_lo, y_hi = float(v[:, 1].min()), float(v[:, 1].max())
    return [{
        "type": "cylinder",
        "radius": round(radius, 6),
        "length": round(y_hi - y_lo, 6),
        "origin": [0.0, round((y_hi + y_lo) / 2.0, 6), 0.0],
        # rotate the cylinder's +Z axis onto the wheel's Y rotation axis
        "rpy": [round(float(np.pi / 2), 6), 0.0, 0.0],
        "name": "wheel",
    }]


def cap_faces(hull: trimesh.Trimesh) -> trimesh.Trimesh:
    """Reduce a hull below HULL_MAX_FACES, staying convex."""
    if len(hull.faces) <= HULL_MAX_FACES:
        return hull
    om = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(np.asarray(hull.vertices)),
        o3d.utility.Vector3iVector(np.asarray(hull.faces)),
    )
    om = om.simplify_quadric_decimation(HULL_MAX_FACES)
    simplified = trimesh.Trimesh(
        vertices=np.asarray(om.vertices),
        faces=np.asarray(om.triangles),
    )
    # Decimation can nick the hull concave; re-hull to guarantee convexity.
    return simplified.convex_hull


def _slice_slabs(mesh: trimesh.Trimesh, axis: int, n: int) -> list[trimesh.Trimesh]:
    lo, hi = mesh.bounds[0][axis], mesh.bounds[1][axis]
    edges = np.linspace(lo, hi, n + 1)
    normal = np.zeros(3)
    normal[axis] = 1.0

    pieces = []
    for a, b in zip(edges[:-1], edges[1:]):
        org_a, org_b = np.zeros(3), np.zeros(3)
        org_a[axis], org_b[axis] = a, b
        # slice_plane keeps the half on the normal's positive side.
        # cap=False avoids trimesh's rtree dependency and costs nothing here:
        # a convex hull depends only on the vertex cloud, and capping adds
        # only coplanar interior faces, so the resulting hull is identical.
        piece = mesh.slice_plane(org_a, normal, cap=False)
        if piece is None or piece.is_empty:
            continue
        piece = piece.slice_plane(org_b, -normal, cap=False)
        if piece is None or piece.is_empty or len(piece.vertices) < 4:
            continue
        pieces.append(piece.convex_hull)
    return pieces


def slab_hulls(mesh: trimesh.Trimesh) -> list[trimesh.Trimesh]:
    """Cut into slabs and hull each piece, searching for the tightest fit.

    The longest axis is not reliably the best cut direction -- for the wrist
    and head-pitch links, slicing across the short axis separates the concave
    features better.  Cheap enough to just try all three axes.  Slabs are
    disjoint bands, so summed hull volume is a fair tightness measure.
    """
    orig_vol = abs(mesh.volume)
    best = None
    for axis in range(3):
        for n in (4, 6, 8):
            pieces = _slice_slabs(mesh, axis, n)
            if not pieces:
                continue
            ratio = sum(abs(p.volume) for p in pieces) / orig_vol
            # prefer fewer shapes when the gain is marginal (<5%)
            if best is None or ratio < best[0] * 0.95:
                best = (ratio, pieces)
    return [cap_faces(p) for p in best[1]] if best else []


def hull_shapes(link: str) -> tuple[list[dict], int, float]:
    """Emit one or more hull STLs for a link. Returns (shapes, orig_faces, ratio)."""
    m = load(link)
    orig_vol = abs(m.volume)
    fill = orig_vol / m.convex_hull.volume if m.convex_hull.volume > 0 else 1.0

    hulls = [cap_faces(m.convex_hull)] if fill >= HULL_FILL_MIN \
        else slab_hulls(m)
    if not hulls:                                   # slicing failed, fall back
        hulls = [cap_faces(m.convex_hull)]

    shapes = []
    for i, h in enumerate(hulls):
        name = f"{link}.STL" if len(hulls) == 1 else f"{link}_{i}.STL"
        h.export(COLLISION / name)
        shapes.append({"type": "mesh", "file": name, "name": f"hull{i}"})

    approx_vol = sum(abs(h.volume) for h in hulls)   # ignores slab overlap
    return shapes, len(m.faces), approx_vol / orig_vol if orig_vol > 0 else 0.0


def main() -> None:
    COLLISION.mkdir(parents=True, exist_ok=True)
    for stale in COLLISION.glob("*.STL"):
        stale.unlink()

    spec: dict[str, list[dict]] = {"base_link": base_link_boxes()}
    for link, breaks in SLAB_LINKS.items():
        spec[link] = slab_boxes(link, breaks)
    for link in WHEEL_LINKS:
        spec[link] = wheel_cylinder(link)

    print(f"{'link':10s} {'strategy':16s} {'orig tris':>10s} {'shapes':>7s} {'vol ratio':>10s}")
    tri_before = tri_after = 0

    for link in ("base_link", *SLAB_LINKS, *WHEEL_LINKS):
        orig = len(load(link).faces)
        tri_before += orig
        n_box = sum(1 for s in spec[link] if s["type"] == "box")
        tri_after += 12 * n_box
        kind = "analytic boxes" if n_box else "analytic cylinder"
        print(f"{link:10s} {kind:16s} {orig:10,d} {len(spec[link]):7d} {'-':>10s}")

    for link in HULL_LINKS:
        shapes, orig, ratio = hull_shapes(link)
        spec[link] = shapes
        tri_before += orig
        tri_after += sum(
            len(trimesh.load(COLLISION / s["file"]).faces) for s in shapes
        )
        kind = "convex hull" if len(shapes) == 1 else f"{len(shapes)} slab hulls"
        print(f"{link:10s} {kind:16s} {orig:10,d} {len(shapes):7d} {ratio:9.2f}x")

    SPEC_OUT.write_text(json.dumps(spec, indent=2))
    print(f"\ncollision triangles: {tri_before:,d} -> {tri_after:,d} "
          f"({tri_before / max(tri_after, 1):.0f}x reduction)")
    print(f"spec -> {SPEC_OUT.relative_to(PKG)}")


if __name__ == "__main__":
    main()
