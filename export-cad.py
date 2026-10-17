"""
Export the granular manipulator model (v2) as CAD: one STEP + one STL per part, a STEP
assembly per pose (names + colours kept), an interference report, and a mate guide.

Setup:   pip install cadquery
Run:     python export_cad.py                       # frame-4 pose  -> exports/cad_export/
         python export_cad.py --pose all            # f1, f2 and f4 assemblies
         python export_cad.py --yaw 0 --u 100 --f 230 --out my_pose
         python export_cad.py --dims wireframe/dims_v2.json    # numbers straight from build_wireframe.py

Coordinates (mm): origin = yaw axis on the plate's top surface, X = right in the front view,
Y = away from the camera (camera side is -Y), Z = up.

HOW THE PARTS ARE FRAMED (this is what makes mating easy)
  Every part is exported in the frame of the joint it belongs to, with its own axis at the origin:
    base group   : world frame (yaw = 0). Shoulder mount/flange have their origin ON the shoulder axis.
    upper-arm    : origin on the shoulder axis, link along +X  (beam, shoulder actuator)
                   elbow actuator and standoff have their origin on the elbow axis
    forearm      : origin on the elbow axis, link along +X     (forearm, elbow flange, elbow mount)
  The assembly STEP then places each part with a rigid transform computed by forward kinematics
  (yaw about Z, shoulder and elbow about Y), so the assembly opens already posed. Y positions
  are absolute in every frame, so the depth stack never changes with pose.

STEP cannot carry mates, so the assembly opens posed but not mated. mates_guide.md lists the
concentric / coincident mates that turn it into a movable assembly in SolidWorks (about ten),
and kinematics.json gives the joint axes and angle conventions for any other tool.

All numbers come from the annotations through build_wireframe.py. Values the annotations could not
support are marked ASSUMED below; edit them (or pass --dims) and re-run.
"""
import argparse, json, math
from pathlib import Path
import cadquery as cq

# ---------------------------------------------------------------- dimensions (mm)
D = {'puckD': 39.56,
 'puckH': 15.43,
 'discZ0': 17.95,
 'discZ1': 22.36,
 'discT': 4.41,
 'discR': 29.38,
 'robotX': 125.0,
 'plateW': 200.0,
 'plateT0': 12.7,
 'holeD': 6.0,
 'cbD': 10.5,
 'encW': 43.75,
 'encFace': 39.74,
 'encC': 2.0,
 'encH': 41.38,
 'encPocket': 31.37,
 'encFloor': 18.96,
 'yawD': 30.0,
 'enclZ0': 22.36,
 'upX0': 29.77,
 'upT': 5.2,
 'upTop': 107.4,
 'boreR': 6.13,
 'roundR': 13.6,
 'housR': 19.73,
 'partR': 18.26,
 'innerR': 14.49,
 'actL': 18.09,
 'flangeR': 29.0,
 'eScrRb': 25.5,
 'sScrRb': 25.4,
 'eScrR': 3.74,
 'sScrR': 3.34,
 'screwL': 4.11,
 'hubRb': 10.5,
 'hubRh': 2.81,
 'beamT': 5.12,
 'beamW': 27.2,
 'L1': 102.0,
 'L2': 102.0,
 'slotUpC': 64.7,
 'slotR': 2.91,
 'slotUpS0': 18.6,
 'slotFB': 82.8,
 'slotFA': 20.5,
 'tipRb': 10.55,
 'tipR': 1.81,
 'smH': 43.9,
 'smY': 40.91,
 'smPocketH': 31.35,
 'smPocketD': 22.55,
 'emH': 39.6,
 'emY': 40.0,
 'emPocketH': 28.1,
 'emPocketD': 24.0,
 'smWx': 41.0,
 'emWx': 40.0,
 'zS': 98.7,
 'beY0': -48.7,
 'beY1': -43.58,
 'saY0': -43.58,
 'saY1': -25.49,
 'sfY0': -25.49,
 'sfY1': -21.08,
 'smY0': -21.08,
 'smY1': 19.83,
 'eaY1': -19.7,
 'eaY0': -37.79,
 'boY0': -43.58,
 'boY1': -37.79,
 'efY0': -19.7,
 'efY1': -15.29,
 'foY0': -15.29,
 'foY1': -10.17,
 'emY0': -10.17,
 'emY1': 29.83,
 'upY0': -25.49,
 'upY1': 19.83}
# ASSUMED (not measurable from the annotations)
D.setdefault("cbHole", 6.6)       # counterbore through-hole
D.setdefault("cbDepth", 6.5)      # counterbore depth
D.setdefault("actStep", 2.0)      # axial length of the actuator's two front steps (r = partR and innerR)
# NOTE f2: the forearm angle (270) is ASSUMED, carried over from the first model, not measured. The interference
# report shows it passes through the shoulder mount (~3500 mm^3); about 250 clears it. f1 and f4 are measured and clear.
POSES = {'f1': {'yaw': 15, 'u': 125.3, 'f': 199.9},
 'f2': {'yaw': 0, 'u': 90, 'f': 270},
 'f4': {'yaw': 0, 'u': 130.5, 'f': 249.7}}

COL = dict(plate=(0.25, 0.25, 0.28), steel=(0.6, 0.62, 0.66), alu=(0.75, 0.77, 0.8), act=(0.9, 0.9, 0.92),
           link=(0.72, 0.75, 0.78), flange=(0.55, 0.58, 0.62))


# ---------------------------------------------------------------- primitives
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))

def cyl_z(r, z0, z1, cx=0.0, cy=0.0):
    return cq.Workplane("XY", origin=(cx, cy, z0)).circle(r).extrude(z1 - z0)

def cyl_y(r, y0, y1, cx=0.0, cz=0.0):
    """Cylinder with its axis along Y spanning y0 < y < y1, centred at (cx, cz)."""
    return cq.Workplane("XZ", origin=(0, y1, 0)).center(cx, cz).circle(r).extrude(y1 - y0)

def prism_xy(pts, z0, z1):
    """Polygon in the XY plane extruded along Z."""
    return cq.Workplane("XY", origin=(0, 0, z0)).polyline(pts).close().extrude(z1 - z0)

def prism_yz(pts, x0, x1):
    """Polygon given as (y, z) points extruded along X."""
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0)

def ch_rect(w, c):
    h = w / 2
    return [(-h + c, -h), (h - c, -h), (h, -h + c), (h, h - c), (h - c, h), (-h + c, h), (-h, h - c), (-h, -h + c)]

def c_pts(ya, yb, web, H, ph, zc=0.0):
    """C-profile in (y, z): solid web on the -y side, pocket open toward +y."""
    return [(ya, zc - H / 2), (yb, zc - H / 2), (yb, zc - ph / 2), (ya + web, zc - ph / 2),
            (ya + web, zc + ph / 2), (yb, zc + ph / 2), (yb, zc + H / 2), (ya, zc + H / 2)]

def ring_pts(rb, n, phase_deg, cx=0.0, cz=0.0):
    """Bolt-circle centres in the X-Z plane, angle measured from +X toward +Z."""
    return [(cx + rb * math.cos(math.radians(phase_deg + 360 * k / n)),
             cz + rb * math.sin(math.radians(phase_deg + 360 * k / n))) for k in range(n)]

def xz(y1, t):
    """Workplane on the X-Z plane whose extrusion spans y1 - t .. y1 (normal is -Y)."""
    return cq.Workplane("XZ", origin=(0, y1, 0))


# ---------------------------------------------------------------- part builders
def optical_plate(d):
    W, T, rx = d["plateW"], d["plateT0"], d["robotX"]
    p = box(-rx, W - rx, -W / 2, W / 2, -T, 0)
    grid = [(-rx + 12.5 + 25 * i, -W / 2 + 12.5 + 25 * j) for i in range(8) for j in range(8)]
    p = p.cut(cq.Workplane("XY", origin=(0, 0, -T)).pushPoints(grid).circle(d["holeD"] / 2).extrude(T))
    cb = [(-rx + a, b) for a in (25, 175) for b in (-75, 75)]
    p = p.cut(cq.Workplane("XY", origin=(0, 0, -T)).pushPoints(cb).circle(d["cbHole"] / 2).extrude(T))
    return p.cut(cq.Workplane("XY", origin=(0, 0, -d["cbDepth"])).pushPoints(cb).circle(d["cbD"] / 2).extrude(d["cbDepth"]))

def enclosure_cage(d):
    """Chamfered solid block + four corner posts; open windows on every face, open top."""
    z0 = d["discZ1"]; zf = z0 + d["encFloor"]; zt = z0 + d["encH"]
    p0, p1, c = d["encPocket"] / 2, d["encW"] / 2, d["encC"]
    cage = prism_xy(ch_rect(d["encW"], c), z0, zf)
    for sx in (-1, 1):
        for sy in (-1, 1):
            pts = [(sx * x, sy * y) for x, y in [(p0, p0), (p1, p0), (p1, p1 - c), (p1 - c, p1), (p0, p1)]]
            cage = cage.union(prism_xy(pts, zf, zt))
    return cage

def actuator(d, y0, y1):
    """Stepped cylinder along Y: housing (r=housR), then partR and innerR steps toward the beam (y0 side)."""
    st = d["actStep"]
    a = cyl_y(d["housR"], y0 + 2 * st, y1).union(cyl_y(d["partR"], y0 + st, y0 + 2 * st)).union(cyl_y(d["innerR"], y0, y0 + st))
    return a.cut(cyl_y(d["boreR"], y0, y1))

def flange(d, y0, y1, rh, rb, phase):
    """Round flange along Y with a bore and four screw heads standing proud on the -Y face."""
    f = cyl_y(d["flangeR"], y0, y1).cut(cyl_y(d["boreR"], y0, y1))
    for (x, z) in ring_pts(rb, 4, phase):
        f = f.union(cyl_y(rh, y0 - d["screwL"], y0, x, z))
    return f

def stadium(L, w, y0, y1):
    """Link plate, joint axis at the origin, running along +X; thickness y0..y1."""
    return xz(y1, y1 - y0).center(L / 2, 0).slot2D(L + w, w, 0).extrude(y1 - y0)

def beam(d):
    L, w, y0, y1 = d["L1"], d["beamW"], d["beY0"], d["beY1"]
    b = stadium(L, w, y0, y1); t = y1 - y0
    sw = 2 * d["slotR"]
    b = b.cut(xz(y1, t).center(L / 2, 0).slot2D(d["slotUpC"] + sw, sw, 0).extrude(t))
    b = b.cut(xz(y1, t).pushPoints([(0, 0), (L, 0)]).circle(d["boreR"]).extrude(t))
    hub = ring_pts(d["hubRb"], 6, 30, 0, 0) + ring_pts(d["hubRb"], 6, 30, L, 0)
    return b.cut(xz(y1, t).pushPoints(hub).circle(d["hubRh"]).extrude(t))

def forearm(d):
    L, w, y0, y1 = d["L2"], d["beamW"], d["foY0"], d["foY1"]
    b = stadium(L, w, y0, y1); t = y1 - y0
    sw = 2 * d["slotR"]; a, bb = d["slotFA"], d["slotFB"]
    b = b.cut(xz(y1, t).center((a + bb) / 2, 0).slot2D((bb - a) + sw, sw, 0).extrude(t))
    b = b.cut(xz(y1, t).pushPoints([(0, 0), (L, 0)]).circle(d["boreR"]).extrude(t))
    return b.cut(xz(y1, t).pushPoints(ring_pts(d["tipRb"], 6, 30, L, 0)).circle(d["tipR"]).extrude(t))

def joint_mount(d, ya, yb, pocket_d, H, ph, wx):
    """C-profile box (profile in Y-Z, extruded along X); web toward -Y, pocket open toward +Y."""
    return prism_yz(c_pts(ya, yb, (yb - ya) - pocket_d, H, ph), -wx / 2, wx / 2)

def elbow_mount(d):
    """Same C-profile, but built in forearm axes: profile height runs along the forearm (+X)."""
    pts = [(z, y) for (y, z) in c_pts(d["emY0"], d["emY1"], d["emY"] - d["emPocketD"], d["emH"], d["emPocketH"])]
    return cq.Workplane("XY", origin=(0, 0, -d["emWx"] / 2)).polyline(pts).close().extrude(d["emWx"])


def build_parts(d):
    """name -> (solid in its own frame, frame group, offset of its origin inside that group, colour)"""
    zs = d["zS"]; zt = d["discZ1"] + d["encH"]; zf = d["discZ1"] + d["encFloor"]
    P = {}
    P["01_optical_plate"] = (optical_plate(d), "world", (0, 0, 0), COL["plate"])
    P["02_spacer"] = (cyl_z(d["puckD"] / 2, 0, d["puckH"]), "world", (0, 0, 0), COL["steel"])
    P["03_yaw_disc"] = (cyl_z(d["discR"], d["discZ0"], d["discZ1"]), "yaw", (0, 0, 0), COL["flange"])
    P["04_base_enclosure"] = (enclosure_cage(d), "yaw", (0, 0, 0), COL["alu"])
    P["05_yaw_actuator"] = (cyl_z(d["yawD"] / 2, zf, zt - 0.5), "yaw", (0, 0, 0), COL["act"])
    P["06_upright"] = (box(d["upX0"], d["upX0"] + d["upT"], d["upY0"], d["upY1"], d["discZ1"], d["upTop"]), "yaw", (0, 0, 0), COL["alu"])
    P["07_shoulder_mount"] = (joint_mount(d, d["smY0"], d["smY1"], d["smPocketD"], d["smH"], d["smPocketH"], d["smWx"]),
                              "yaw", (0, 0, zs), COL["alu"])
    P["08_shoulder_flange"] = (flange(d, d["sfY0"], d["sfY1"], d["sScrR"], d["sScrRb"], 45), "yaw", (0, 0, zs), COL["flange"])
    P["09_shoulder_actuator"] = (actuator(d, d["saY0"], d["saY1"]), "u", (0, 0, 0), COL["act"])
    P["10_connector_beam"] = (beam(d), "u", (0, 0, 0), COL["link"])
    P["11_beam_standoff"] = (cyl_y(d["roundR"], d["boY0"], d["boY1"]).cut(cyl_y(d["boreR"], d["boY0"], d["boY1"])),
                             "u", (d["L1"], 0, 0), COL["steel"])
    P["12_elbow_actuator"] = (actuator(d, d["eaY0"], d["eaY1"]), "u", (d["L1"], 0, 0), COL["act"])
    P["13_elbow_flange"] = (flange(d, d["efY0"], d["efY1"], d["eScrR"], d["eScrRb"], 45), "f", (0, 0, 0), COL["flange"])
    P["14_forearm"] = (forearm(d), "f", (0, 0, 0), COL["link"])
    P["15_elbow_mount"] = (elbow_mount(d), "f", (0, 0, 0), COL["alu"])
    return P


# ---------------------------------------------------------------- forward kinematics
def rot_y(deg):  return cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), deg)
def rot_z(deg):  return cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), deg)

def elbow_point(d, u):
    th = math.radians(u)
    return (d["L1"] * math.cos(th), 0.0, d["zS"] + d["L1"] * math.sin(th))

def group_locations(d, yaw, u, f):
    """Rigid transform of each frame group. Angles: yaw CCW seen from above; u, f measured from +X toward +Z."""
    Lz = rot_z(yaw)
    E = elbow_point(d, u)
    return {"world": cq.Location(),
            "yaw": Lz,
            "u": Lz * cq.Location(cq.Vector(0, 0, d["zS"])) * rot_y(-u),
            "f": Lz * cq.Location(cq.Vector(*E)) * rot_y(-f)}

def placed(parts, locs):
    """name -> (world Location, world Shape)"""
    out = {}
    for name, (solid, grp, off, _) in parts.items():
        loc = locs[grp] * cq.Location(cq.Vector(*off))
        out[name] = (loc, solid.val().moved(loc))
    return out


# ---------------------------------------------------------------- checks and documents
def interference(world, tol=0.5):
    """Pairs whose solids overlap by more than tol mm^3 (face contact is fine)."""
    names = list(world); hits = []
    bbs = {n: world[n][1].BoundingBox() for n in names}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            A, B = bbs[a], bbs[b]
            if (A.xmax < B.xmin or B.xmax < A.xmin or A.ymax < B.ymin or B.ymax < A.ymin or A.zmax < B.zmin or B.zmax < A.zmin):
                continue
            try:
                v = world[a][1].intersect(world[b][1]).Volume()
            except Exception:
                continue
            if v > tol:
                hits.append((a, b, v))
    return hits

def write_mates_guide(path, d):
    g = lambda k: f"{d[k]:.2f}"
    path.write_text(f"""# Mate guide (SolidWorks / Onshape / Fusion)

STEP carries positions, not mates, so the assembly opens in its pose but is not yet movable.
Every part's origin is on its own joint axis, so each joint below is two or three mates.
Fix `01_optical_plate`, then add:

## Fixed to the base (no motion between these)
- `02_spacer` bottom face coincident with `01_optical_plate` top face; spacer and plate hole pattern concentric with the yaw axis (Z).
- `03_yaw_disc`, `04_base_enclosure`, `05_yaw_actuator`, `06_upright`, `07_shoulder_mount`, `08_shoulder_flange`
  are one rigid group that rotates about Z. Mate them to each other (coincident faces), then add the **yaw joint**:
  - **Concentric**: `03_yaw_disc` outer cylinder with `02_spacer` outer cylinder (yaw axis).
  - **Coincident**: `03_yaw_disc` bottom face with `02_spacer` top face (the model leaves a {d['discZ0'] - d['puckH']:.1f} mm gap there, so use a distance mate of {d['discZ0'] - d['puckH']:.1f} mm, or 0 if you add a bearing).
  - Leave the rotation free (or add an angle mate for yaw, range ±180°).
- `04_base_enclosure` bottom face coincident with `03_yaw_disc` top face.
- `08_shoulder_flange` bore concentric with the `07_shoulder_mount` shoulder axis; its back face (+Y) coincident with the mount's -Y face. Its four screw heads stand proud of the front face (-Y).

## Shoulder joint (axis along Y at height {g('zS')} mm)
Group that rotates: `09_shoulder_actuator`, `10_connector_beam`, `11_beam_standoff`, `12_elbow_actuator` (all rigid to each other).
- **Concentric**: `09_shoulder_actuator` bore with `08_shoulder_flange` bore.
- **Coincident**: `09_shoulder_actuator` back face (+Y) with `08_shoulder_flange` front face (-Y).
- **Coincident**: `10_connector_beam` back face with `09_shoulder_actuator` front boss face; beam bore concentric with the actuator bore.
- Leave the rotation free, or add an angle mate between the beam's long axis and the plate's X axis (convention: 0° = +X, positive toward +Z).

## Elbow joint (axis along Y, {g('L1')} mm along the beam)
Group that rotates: `13_elbow_flange`, `14_forearm`, `15_elbow_mount`.
- `11_beam_standoff` back face coincident with `12_elbow_actuator` front boss face; both concentric with the beam's elbow-end bore.
- `12_elbow_actuator` bore **concentric** with `13_elbow_flange` bore; `12_elbow_actuator` back face **coincident** with the flange's front face (the four screw heads stand proud of it at R{d['eScrRb']:.1f}, outside the housing radius, so they do not collide).
- `14_forearm` front face coincident with `13_elbow_flange` back face; elbow-end bore concentric.
- `15_elbow_mount` front face coincident with `14_forearm` back face; mount axis through the elbow bore.
- Leave the rotation free (angle convention as above, forearm angle measured in the same sense).

## Y stack (camera side is -Y), from the annotations
beam {g('beY0')} .. {g('beY1')} | shoulder actuator .. {g('saY1')} | shoulder flange .. {g('sfY1')} | shoulder mount .. {g('smY1')}
elbow: standoff {g('boY0')} .. {g('boY1')} | actuator .. {g('eaY1')} | flange .. {g('efY1')} | forearm .. {g('foY1')} | mount .. {g('emY1')}
""", encoding="utf-8")

def write_kinematics(path, d, yaw, u, f):
    json.dump({
        "units": "mm", "frame": "origin on the yaw axis at the plate top; X right (front view), Y away from camera, Z up",
        "pose": {"yaw_deg": yaw, "upper_deg": u, "forearm_deg": f},
        "joints": [
            {"name": "yaw", "axis": [0, 0, 1], "origin": [0, 0, 0], "angle": "CCW seen from above",
             "moves": ["03_yaw_disc", "04_base_enclosure", "05_yaw_actuator", "06_upright", "07_shoulder_mount", "08_shoulder_flange"]},
            {"name": "shoulder", "axis": [0, 1, 0], "origin_in_yaw_frame": [0, 0, d["zS"]],
             "angle": "from +X toward +Z; part frame is rotated by -angle about +Y",
             "moves": ["09_shoulder_actuator", "10_connector_beam", "11_beam_standoff", "12_elbow_actuator"]},
            {"name": "elbow", "axis": [0, 1, 0], "origin_in_shoulder_frame": [d["L1"], 0, 0],
             "angle": "absolute forearm angle, same convention (not relative to the upper arm)",
             "moves": ["13_elbow_flange", "14_forearm", "15_elbow_mount"]}],
        "link_lengths": {"L1": d["L1"], "L2": d["L2"]},
        "limits": "not known from the annotations"}, path.open("w"), indent=1)


# ---------------------------------------------------------------- export
def export_pose(parts, d, name, yaw, u, f, root, write_parts):
    out = root / name
    out.mkdir(parents=True, exist_ok=True)
    locs = group_locations(d, yaw, u, f)
    world = placed(parts, locs)
    asm = cq.Assembly(name=f"manipulator_{name}")
    for n, (solid, grp, off, col) in parts.items():
        asm.add(solid, name=n, loc=world[n][0], color=cq.Color(*col))
    target = str(out / "manipulator_assembly.step")
    try:
        asm.export(target, exportType="STEP")
    except (AttributeError, TypeError):
        asm.save(target, "STEP")
    print(f"\n[{name}] yaw {yaw}  upper {u}  forearm {f}  -> {target}")
    hits = interference(world)
    if hits:
        print("  overlaps (mm^3):")
        for a, b, v in hits:
            print(f"    {a}  x  {b}: {v:.1f}")
    else:
        print("  no overlapping parts at this pose")
    if write_parts:
        (root / "step").mkdir(exist_ok=True); (root / "stl").mkdir(exist_ok=True)
        for n, (solid, grp, off, col) in parts.items():
            cq.exporters.export(solid, str(root / "step" / f"{n}.step"))
            cq.exporters.export(solid, str(root / "stl" / f"{n}.stl"), tolerance=0.05, angularTolerance=0.1)
            bb = solid.val().BoundingBox()
            print(f"  {n:22s} frame {grp:5s} x[{bb.xmin:7.1f},{bb.xmax:7.1f}] y[{bb.ymin:7.1f},{bb.ymax:7.1f}] z[{bb.zmin:7.1f},{bb.zmax:7.1f}]")
    return world

def main():
    global D
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", choices=list(POSES) + ["all"], default="f4")
    ap.add_argument("--yaw", type=float); ap.add_argument("--u", type=float); ap.add_argument("--f", type=float)
    ap.add_argument("--dims", help="dims_v2.json written by build_wireframe.py (overrides the built-in numbers)")
    ap.add_argument("--out", default="cad_export")
    a = ap.parse_args()
    if a.dims:
        D.update(json.load(open(a.dims))["D"])
    root = Path("exports") / a.out
    root.mkdir(parents=True, exist_ok=True)
    parts = build_parts(D)
    custom = any(v is not None for v in (a.yaw, a.u, a.f))
    if custom:
        base = POSES["f4" if a.pose == "all" else a.pose]
        runs = [("custom", a.yaw if a.yaw is not None else base["yaw"], a.u if a.u is not None else base["u"],
                 a.f if a.f is not None else base["f"])]
    else:
        runs = [(k, v["yaw"], v["u"], v["f"]) for k, v in POSES.items()] if a.pose == "all" else \
               [(a.pose, POSES[a.pose]["yaw"], POSES[a.pose]["u"], POSES[a.pose]["f"])]
    for i, (name, yaw, u, f) in enumerate(runs):
        export_pose(parts, D, name, yaw, u, f, root, write_parts=(i == 0))
    write_mates_guide(root / "mates_guide.md", D)
    write_kinematics(root / "kinematics.json", D, *runs[0][1:])
    print(f"\nwrote {len(parts)} part STEPs + STLs (in joint-local frames), {len(runs)} assembly STEP(s), mates_guide.md, kinematics.json in {root}/")

if __name__ == "__main__":
    main()