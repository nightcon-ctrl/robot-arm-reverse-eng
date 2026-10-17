"""
Build the granular wireframe viewer from the annotation JSON.

    python build_wireframe.py [annotations.json] [out.html]

Every dimension is derived from the annotations (px / one shared scale) and
printed with its provenance. Anything the annotations cannot support is a
named ASSUMED constant in the ASSUMED block below -- edit it and re-run.

Frames (new annotations):  F1 = t5.37   F2 = t14.25   F3 = t16.12   F4 = t21.72
Frames 1, 3, 4 look along +Y (arm plane face-on); frame 2 looks along the arm
plane (edge-on), so its horizontal axis is the depth (Y) stack.
"""
import json, math, sys, statistics as st
from pathlib import Path

SRC = sys.argv[1] if len(sys.argv) > 1 else "annotations_1.json"
OUT = sys.argv[2] if len(sys.argv) > 2 else "wireframe-model-v2.html"
d = json.load(open(SRC))
F1, F2, F3, F4 = "frame_t0005.37s.png", "frame_t0014.25s.png", "frame_t0016.12s.png", "frame_t0021.72s.png"
S = d[F2][5]["px_per_mm"]                       # the one shared scale: frame-2 hole pitch
assert d[F2][5]["type"] == "scale"
T = {F1: "f1", F2: "f2", F3: "f3", F4: "f4"}

def ann(f, i, prefix=None):
    a = d[f][i]
    if prefix:
        assert a["name"].startswith(prefix), (f, i, a["name"])
    return a
def rr(f, i, prefix=None):  return ann(f, i, prefix)["radius_px"] / S
def cc(f, i, prefix=None):  return tuple(ann(f, i, prefix)["center"])
def ln(f, i, prefix=None):  return ann(f, i, prefix)["px"] / S
def xs(f, i):               return [p[0] for p in d[f][i]["pts"]]
def ys(f, i):               return [p[1] for p in d[f][i]["pts"]]

ROWS = []                                      # [label, value, n, range, source]  or  [group]
def grp(t): ROWS.append([t])
def row(label, vals, src, digits=1, unit=""):
    vals = list(vals)
    m = st.mean(vals)
    rng = f"{min(vals):.{digits}f}–{max(vals):.{digits}f}" if len(vals) > 1 else "—"
    ROWS.append([label, f"{m:.{digits}f}{unit}", len(vals), rng, src])
    return m
def note_row(label, value, src, n=1):
    ROWS.append([label, value, n, "—", src]); return value

# ------------------------------------------------------------------ reference plane / camera
P = ann(F1, 64, "rotatable plate spacer")["pts"]
x0 = min(p[0] for p in P); ybot = max(p[1] for p in P)
left = [p for p in P if p[0] - x0 < 2.0]
py_ref = max(p[1] for p in left)               # z = 0 at the yaw-axis depth plane
puck_h = (max(p[1] for p in left) - min(p[1] for p in left)) / S
puck_d = (max(p[0] for p in P) - x0) / S
sinphi = ((ybot - py_ref) / S) / (puck_d / 2)  # camera tilt from the puck's bottom ellipse
phi = math.degrees(math.asin(sinphi))
zc = lambda py, Y: (py_ref - py) / S + abs(Y) * sinphi    # tilt-corrected height of a point at depth Y

grp("Scale and camera")
note_row("Scale, all frames", f"{S:.3f} px/mm", "frame 2: ‘table hole pitch’ 78.27 px = 25 mm (shared by every frame)")
note_row("Camera tilt (from the puck’s top/bottom ellipse)", f"{phi:.1f}°", "F1 ‘rotatable plate spacer’: bottom arc sits 2.2 mm below the tangent points of a Ø39.6 cylinder")
note_row("z = 0 reference", "plate top at the yaw axis", "F1 spacer polygon, bottom-left vertex (tangent plane, depth 0)")

# ------------------------------------------------------------------ base
grp("Base: spacer, plate, enclosure")
D = {}
D["puckD"] = row("Spacer (puck) Ø", [puck_d], "F1 ‘rotatable plate spacer from baseplate’ polygon width", 1)
D["puckH"] = row("Spacer height", [puck_h], "F1 same polygon, left silhouette edge (tangent plane)", 1)
a = ann(F1, 63, "thickness of circular")["pts"]
D["discZ0"] = (py_ref - a[1][1]) / S; D["discZ1"] = (py_ref - a[0][1]) / S
D["discT"] = D["discZ1"] - D["discZ0"]
note_row("Movable disc thickness", f"{D['discT']:.1f}", "F1 line ‘thickness of circular movable plate’ (4.4 mm)")
note_row("Movable disc z range", f"{D['discZ0']:.1f} – {D['discZ1']:.1f}", f"same line, tilt-corrected; leaves a {D['discZ0']-D['puckH']:.1f} mm gap above the spacer (unexplained)")
disc_left = min(xs(F1, 61))
yawax_px = st.mean([(min(xs(F1, 64)) + max(xs(F1, 64))) / 2, (min(xs(F4, 3)) + max(xs(F4, 3))) / 2])
D["discR"] = row("Movable disc radius", [(yawax_px - disc_left) / S, (yawax_px - min(xs(F4, 12))) / S],
                 "F1 ‘circular movable plate… top’ and F4 plate polygon left edge, measured from the spacer/enclosure centre line", 1)
D["robotX"] = 125.0; D["plateW"] = 200.0; D["plateT0"] = 12.7; D["holeD"] = 6.0
D["cbD"] = round(33 / S, 1)
note_row("Optical plate", "200 × 200 × 12.7", "carried over, not re-measured (8 × 8 holes at 25 mm; thickness, hole Ø and robot position assumed)")
note_row("Counterbore Ø", f"{D['cbD']:.1f}", "old frame-4 click (33 px) re-scaled; not in the new annotations")

# enclosure: front-face polygons (frames 2, 3, 4) and chamfer polygons
front = [(F2, 0), (F3, 0), (F4, 3)]
fw = [(max(xs(f, i)) - min(xs(f, i))) / S for f, i in front]
fh = [(max(ys(f, i)) - min(ys(f, i))) / S for f, i in front + [(F1, 59)]]
def pocket(f, i):
    P = d[f][i]["pts"]; xb = min(p[0] for p in P); yb = max(p[1] for p in P)
    L = [((p[0] - xb) / S, (yb - p[1]) / S) for p in P]
    inner = sorted(L, key=lambda q: q[1])            # floor vertices are the 2 lowest interior ones
    top = [q for q in L if q[1] > 30]
    flo = [q for q in L if 15 < q[1] < 25]
    return max(q[0] for q in flo) - min(q[0] for q in flo), st.mean(q[1] for q in flo)
pk = [pocket(f, i) for f, i in front + [(F1, 59)]]
ov = [(max(xs(F4, k) + xs(F4, 4) + xs(F4, 5)) - min(xs(F4, 3) + xs(F4, 4) + xs(F4, 5))) / S for k in (3,)]
ov += [(max(xs(F3, 0) + xs(F3, 1) + xs(F3, 2)) - min(xs(F3, 0) + xs(F3, 1) + xs(F3, 2))) / S,
       (max(xs(F2, 0) + xs(F2, 1) + xs(F2, 2)) - min(xs(F2, 0) + xs(F2, 1) + xs(F2, 2))) / S]
D["encW"] = row("Enclosure plan width (incl. chamfers)", ov, "F4 polygons 3+4+5, F3 0+1+2, F2 0+1+2 (front face + both chamfers)", 1)
D["encFace"] = row("Enclosure flat front face width", fw, "F2, F3, F4 ‘front profile of base enclosure’", 1)
D["encC"] = round((D["encW"] - D["encFace"]) / 2, 1)
note_row("Vertical-edge chamfer", f"{D['encC']:.1f}", "(plan width − flat face) / 2")
D["encH"] = row("Enclosure height", fh, "front-profile polygons, F1–F4", 1)
D["encPocket"] = row("Cage window width (between posts)", [p[0] for p in pk[:3]], "front-profile notch, F2–F4", 1)
D["encFloor"] = row("Solid base block height (notch floor)", [p[1] for p in pk], "front-profile notch floor, F1–F4", 1)
D["yawD"] = 30.0
note_row("Yaw actuator Ø × height", f"30 × {D['encH'] - D['encFloor'] - 0.5:.1f}", "ASSUMED: fits the 31 mm window; old frame-4 click gave Ø30.4 (not in the new annotations)")
D["enclZ0"] = D["discZ1"]

# upright (elevated arm): thin strip on the right side of the disc (F4 polygon 12) + top ear (F4 polygon 11, F1 polygon 60)
P12 = ann(F4, 12)["pts"]; x12 = min(p[0] for p in P12); yb12 = max(p[1] for p in P12)
strip = [p for p in P12 if (yb12 - p[1]) / S > 9 and (p[0] - x12) / S > 50]   # the vertical strip only, above the rim band
strip_w = (max(p[0] for p in strip) - min(p[0] for p in strip)) / S
strip_x0 = (min(p[0] for p in strip) - yawax_px) / S
D["upX0"] = strip_x0; D["upT"] = round(strip_w, 1)
top_px = min(ys(F1, 60) + ys(F4, 11))
D["upTop"] = round(zc(top_px, 0) - 20.2 * sinphi * 1.0 + 0.0, 1)   # top silhouette is the back edge
note_row("Upright strip: X start, thickness", f"{D['upX0']:.1f}, {D['upT']:.1f}", "F4 polygon 12 right-hand strip (touches the disc edge; thickness ≈ disc thickness)")
note_row("Upright top", f"{D['upTop']:.1f}", "F1 ‘elevated arm beam’ + F4 polygon 11 top edge, back-edge correction; Y-extent assumed")

# ------------------------------------------------------------------ joint hardware
grp("Joint hardware (identical at shoulder and elbow)")
D["boreR"] = row("Bore radius", [rr(F1, 27), rr(F1, 39), rr(F1, 40), rr(F1, 6), rr(F4, 0), rr(F4, 1), rr(F4, 2)],
                 "F1 ‘inner joint’ ×3, ‘tip main hole’; F4 three unnamed small circles", 2)
D["roundR"] = row("Link end radius (beam half-width)", [rr(F1, 7), rr(F1, 26), rr(F1, 41), rr(F4, 8), rr(F4, 9), rr(F4, 10)],
                  "F1 tip end-effector rounding, connector joint radii; F4 three r≈13.6 circles", 2)
D["housR"] = row("Actuator housing radius", [rr(F1, 23), rr(F1, 55)], "F1 ‘tip beam actuator housing’, shoulder ring 55", 2)
D["partR"] = row("Harmonic-drive step radius", [rr(F1, 24), rr(F1, 54)], "F1 ‘actuator part (probably a harmonic drive)’, shoulder ring 54", 2)
D["innerR"] = row("Inner recess radius", [rr(F1, 25)], "F1 ‘actuator part inner radius’", 2)
D["actL"] = row("Actuator axial length", [ln(F2, 8), ln(F2, 11)], "F2 ‘height of actuator’ 17.9, ‘depth of joint 2 actuator housing’ 18.3", 1)
D["flangeR"] = row("Flange (‘screwplate’) radius", [rr(F1, 8), rr(F1, 56), rr(F4, 6), rr(F4, 7)],
                   "F1 ‘tip beam joint screwplate radius’, shoulder ring 56; F4 two r≈29 circles", 2)
def fit(f, idx):
    pts = [cc(f, i) for i in idx]
    cx = st.mean(p[0] for p in pts); cy = st.mean(p[1] for p in pts)
    ang = [math.degrees(math.atan2(-(p[1] - cy), p[0] - cx)) % 360 for p in pts]
    return (cx, cy), st.mean(math.dist(p, (cx, cy)) for p in pts) / S, ang
(_, eRb, eAng) = fit(F1, range(11, 15)); (_, sRb, sAng) = fit(F1, range(44, 48))
D["eScrRb"] = round(eRb, 1); D["sScrRb"] = round(sRb, 1)
D["eScrR"] = row("Elbow flange screw radius (×4)", [rr(F1, i) for i in range(11, 15)], "F1 ‘tip beam joint screwplate screw 1–4’", 2)
D["sScrR"] = row("Shoulder flange screw radius (×4)", [rr(F1, i) for i in range(44, 48)], "F1 unnamed circles 44–47 (square pattern)", 2)
note_row("Flange screw bolt-circle radius", f"elbow {eRb:.1f}, shoulder {sRb:.1f}", "centroid fit of the four screws; both at 45° to the link / base axes")
D["screwL"] = row("Flange screw head height", [ln(F2, 12)], "F2 ‘depth of tip screwplate screws’", 1)
(_, h1, a1) = fit(F1, range(28, 34)); (_, h2, a2) = fit(F1, range(48, 54))
D["hubRb"] = round(st.mean([h1, h2]), 1)
D["hubRh"] = row("Hub screw radius (×6)", [rr(F1, i) for i in list(range(28, 34)) + list(range(48, 54))], "F1 ‘connector to supporting beam screw 1–6’ + shoulder circles 48–53", 2)
note_row("Hub bolt-circle radius, 6 screws", f"{h1:.1f} (elbow) / {h2:.1f} (shoulder)", "centroid fit; screws sit at link angle ± 30° + 60°·k")
D["beamT"] = row("Beam thickness", [ln(F2, 9), ln(F2, 10)], "F2 ‘depth of beam connecting joint 1 with joint 2’, ‘depth of tip beam’", 2)

# ------------------------------------------------------------------ links, slots, tip
grp("Links, slots, tip")
D["beamW"] = round(2 * D["roundR"], 1)
note_row("Link width", f"{D['beamW']:.1f}", "2 × end radius; F1 ‘tip beam top edge to bottom edge height’ 27.1 agrees", 2)
dist = lambda a, b: math.dist(a, b) / S
l1 = [dist(cc(F1, 56), cc(F1, 8)), dist(cc(F4, 6), cc(F4, 7)), dist(cc(F1, 55), cc(F1, 23)), dist((1038.8, 637.2), (828.4, 394.5))]
l2 = [dist(cc(F1, 8), cc(F1, 6)), dist(cc(F4, 7), cc(F4, 8)), dist(cc(F1, 23), cc(F1, 6)), dist(cc(F4, 10), cc(F4, 8))]
D["L1"] = D["L2"] = round(st.mean(l1 + l2), 1)
ROWS.append(["Upper arm L1 (axis to axis)", f"{st.mean(l1):.1f}", 4, f"{min(l1):.1f}–{max(l1):.1f}", "F1+F4: flange centres and housing-ring centres"])
ROWS.append(["Forearm L2 (axis to tip hole)", f"{st.mean(l2):.1f}", 4, f"{min(l2):.1f}–{max(l2):.1f}", "same, to the tip main hole (frame 1 is yawed ≈15°, so ±2 %)"])
note_row("L1 = L2 used in the model", f"{D['L1']:.1f}", "mean of all eight; the two links are equal within the scatter", 8)
cw = math.dist(cc(F1, 34), cc(F1, 35)) / S
D["slotUpC"] = round(cw, 1)
D["slotR"] = row("Slot end radius", [rr(F1, 34), rr(F1, 35), rr(F1, 15)], "F1 ‘connector beam cutout rounding radius’ ×2, ‘tip beam cutout rounding radius’", 2)
note_row("Upper-arm slot, centre to centre", f"{cw:.1f}", "F1 ‘connector beam cutout rounding radius’ (upper) → ‘…lower’; edge lines read 65.5 / 64.5")
D["slotUpS0"] = round((D["L1"] - cw) / 2, 1)
note_row("Upper-arm slot starts at", f"{D['slotUpS0']:.1f} from each joint", "measured 19.4 from the elbow and 18.6 from the shoulder; centred")
tipc = cc(F1, 6); near = cc(F1, 15)
tip_to_near = math.dist(tipc, near) / S
cut_len = ann(F1, 16)["mm"]
D["slotFB"] = round(D["L2"] - tip_to_near, 1)                     # tip-side slot centre, measured from the elbow
D["slotFA"] = round(D["slotFB"] - cut_len, 1)                      # far slot centre
note_row("Forearm slot centres from the elbow", f"{D['slotFA']:.1f} and {D['slotFB']:.1f}", f"F1: tip-side centre is {tip_to_near:.1f} from the tip hole; line to the ‘joint actuator edge’ = {cut_len:.1f}")
D["tipRb"] = round(st.mean(math.dist(cc(F1, i), tipc) for i in range(6)) / S, 2)
D["tipR"] = row("Tip aux hole radius (×6)", [rr(F1, i) for i in range(6)], "F1 ‘tip aux hole 1–6’", 2)
note_row("Tip aux-hole bolt-circle radius", f"{D['tipRb']:.2f}", "distance from ‘tip main hole’ centre; holes at link angle ± 30° + 60°·k")

# ------------------------------------------------------------------ joint boxes (C-profiles)
grp("Joint mounts (C-profile boxes)")
def cprof(f, i, pat):
    P = d[f][i]["pts"]; xb = min(p[0] for p in P); yb = max(p[1] for p in P)
    return [((p[0] - xb) / S, (yb - p[1]) / S) for p in P]
j1 = cprof(F2, 4, None); j1b = cprof(F3, 3, None); j2 = cprof(F2, 15, None)
def union_h(f, idx):
    y = [v for i in idx for v in ys(f, i)]
    return (max(y) - min(y)) / S
D["smH"] = round(row("Shoulder mount height (face + chamfer bands)", [union_h(F2, (4, 6, 7)), union_h(F3, (3, 4, 5))],
                     "F2 polygons 4+6+7 and F3 polygons 3+4+5 (front face plus top and bottom chamfer bands)", 1), 1)
D["smY"] = row("Shoulder mount Y-width", [(max(xs(F2, 4)) - min(xs(F2, 4))) / S, (max(xs(F3, 3)) - min(xs(F3, 3))) / S], "F2 ‘front profile of 1st joint actuator mount’, F3 ‘…joint 1 actuator enclosure’", 1)
D["smPocketH"] = row("Shoulder mount pocket height", [31.3, 31.4], "C-notch of F2 polygon 4 and F3 polygon 3", 1)
D["smPocketD"] = row("Shoulder mount pocket depth", [21.9, 23.2], "C-notch of F2 polygon 4 and F3 polygon 3", 1)
D["emH"] = 39.6; D["emY"] = 40.0; D["emPocketH"] = 28.1; D["emPocketD"] = 24.0
note_row("Elbow mount H × Y-width, pocket H × D", "39.6 × 40.0, 28.1 × 24.0", "F2 ‘front profile of actuator 2 enclosure’ (45.4 wide incl. the forearm plate, so 5.4 trimmed) + chamfer band 2.9")
D["smWx"] = 41.0; D["emWx"] = 40.0
note_row("Mount X-widths", "41.0 / 40.0", "ASSUMED: no frame shows the X face of these boxes")

# ------------------------------------------------------------------ vertical position of the axes
grp("Heights and angles")
zS = [zc(638.4, 43), zc(638.4, 48), zc(635.7, 23), zc(635.1, 23)]
D["zS"] = round(st.mean(zS), 1)
ROWS.append(["Shoulder axis height", f"{D['zS']:.1f}", 4, f"{min(zS):.1f}–{max(zS):.1f}", "F1+F4 ring/flange centres, corrected for camera tilt at each layer’s depth (was 94.8 uncorrected, old scale)"])

def ang(a, b): return math.degrees(math.atan2(-(b[1] - a[1]), b[0] - a[0])) % 360
u4 = ang(cc(F4, 6), cc(F4, 7)); f4 = ang(cc(F4, 7), cc(F4, 0))
cosyaw = math.cos(math.radians(15))
def ang_yaw(a, b): return math.degrees(math.atan2(-(b[1] - a[1]), (b[0] - a[0]) / cosyaw)) % 360
u1 = ang_yaw(cc(F1, 56), cc(F1, 8)); f1 = ang_yaw(cc(F1, 8), cc(F1, 6))
POSES = {"f1": dict(yaw=15, u=round(u1, 1), f=round(f1, 1)),
         "f2": dict(yaw=0, u=90, f=270),
         "f4": dict(yaw=0, u=round(u4, 1), f=round(f4, 1))}
note_row("Pose, frame 4 (t21.72)", f"upper {u4:.1f}°, forearm {f4:.1f}°", "angles of the flange-centre → flange-centre / tip-hole vectors; arm plane square to the camera")
note_row("Pose, frame 1 (t5.37)", f"upper {u1:.1f}°, forearm {f1:.1f}°, base yaw ≈15°", "same, with x divided by cos 15°; yaw from the enclosure (front face 38.5 of 44.0 wide, left side face visible)")
note_row("Pose, frame 2 (t14.25)", "upper 90°, forearm 270°", f"elbow box centre is {(d[F2][4]['centroid_px'][1] - d[F2][15]['centroid_px'][1]) / S:.1f} mm above the shoulder box centre ≈ L1; forearm hangs below the elbow")

# ------------------------------------------------------------------ depth (Y) stack from frame 2
grp("Depth stack (Y), from frame 2")
xc = (min(xs(F2, 0)) + max(xs(F2, 0))) / 2                              # enclosure front-face centre = Y 0
Y = lambda px: -(px - xc) / S
def yr(label, f, i, ia=0, ib=1, src=""):
    a, b = d[f][i]["pts"][ia][0], d[f][i]["pts"][ib][0]
    lo, hi = sorted([Y(a), Y(b)])
    ROWS.append([label, f"{lo:.1f} … {hi:.1f}", 1, f"{hi - lo:.1f} thick", src])
    return lo, hi
yr("Connector beam", F2, 9, src="F2 ‘depth of beam connecting joint 1 with joint 2’")
yr("Shoulder actuator", F2, 8, src="F2 ‘height of actuator’")
yr("Elbow actuator housing", F2, 11, src="F2 ‘depth of joint 2 actuator housing’")
yr("Elbow flange screws", F2, 12, src="F2 ‘depth of tip screwplate screws’")
yr("Forearm (tip beam)", F2, 10, src="F2 ‘depth of tip beam’")
ROWS.append(["Shoulder mount box", f"{Y(max(xs(F2, 4))):.1f} … {Y(min(xs(F2, 4))):.1f}", 1, "—", "F2 front polygon of the 1st joint actuator mount"])
ROWS.append(["Base enclosure", f"{Y(max(xs(F2, 0))):.1f} … {Y(min(xs(F2, 0))):.1f}", 1, "—", "F2 front polygon (defines Y = 0 at its centre)"])
note_row("Check: shoulder housing front vs. parallax", "−46.3 vs ≈ −46", "F1 vs F4 shift of the shoulder ring (≈11 mm at 15° yaw) puts it ≈46 mm toward the camera: independent of frame 2")

# model layout (mm, no overlaps): beam | shoulder actuator | flange | mount, stacked toward +Y
D["beY0"] = -48.7; D["beY1"] = D["beY0"] + D["beamT"]
D["saY0"] = D["beY1"]; D["saY1"] = D["saY0"] + D["actL"]
D["sfY0"] = D["saY1"]; D["sfY1"] = D["sfY0"] + D["discT"]
D["smY0"] = D["sfY1"]; D["smY1"] = D["smY0"] + D["smY"]
D["eaY1"] = -19.7; D["eaY0"] = D["eaY1"] - D["actL"]
D["boY0"] = D["beY1"]; D["boY1"] = D["eaY0"]
D["efY0"] = D["eaY1"]; D["efY1"] = D["efY0"] + D["discT"]
D["foY0"] = D["efY1"]; D["foY1"] = D["foY0"] + D["beamT"]
D["emY0"] = D["foY1"]; D["emY1"] = D["emY0"] + D["emY"]
D["upY0"] = D["sfY0"]; D["upY1"] = D["smY1"]
for k in D: D[k] = round(D[k], 2) if isinstance(D[k], float) else D[k]
note_row("Layout used (Y, camera at −Y)", "beam −48.7…−43.6 | shoulder actuator …−25.5 | flange …−21.1 | mount …19.9", "stacked without overlap; ≤ 2.7 mm shifted from the raw frame-2 values")
note_row("Elbow side", f"boss {D['boY0']:.1f}…{D['boY1']:.1f} | actuator …{D['eaY1']:.1f} | flange …{D['efY1']:.1f} | forearm …{D['foY1']:.1f}", "boss = 5.7 mm standoff between the beam and the elbow housing (ASSUMED)")

# print provenance
for r in ROWS:
    print("  ", " | ".join(str(x) for x in r))
print("\nD =", json.dumps(D))
print("POSES =", POSES)

tmpl = (Path(__file__).parent / "wireframe-template.html").read_text(encoding="utf-8")
html = (tmpl.replace("__D__", json.dumps(D)).replace("__ROWS__", json.dumps(ROWS, ensure_ascii=False))
            .replace("__POSES__", json.dumps(POSES)).replace("__SCALE__", f"{S:.3f}").replace("__PHI__", f"{phi:.1f}"))
for k, v in {"__L__": f"{D['L1']:.1f}", "__ENCW__": f"{D['encW']:.0f}", "__FLOOR__": f"{D['encFloor']:.0f}",
             "__POST__": f"{(D['encW'] - D['encPocket']) / 2:.1f}", "__ENCH__": f"{D['encH']:.1f}",
             "__HOUSD__": f"{2 * D['housR']:.1f}", "__PARTD__": f"{2 * D['partR']:.1f}", "__INNERD__": f"{2 * D['innerR']:.0f}",
             "__BORED__": f"{2 * D['boreR']:.1f}", "__ACTL__": f"{D['actL']:.1f}", "__FLD__": f"{2 * D['flangeR']:.0f}",
             "__HUBRB__": f"{D['hubRb']:.1f}", "__TIPRB__": f"{D['tipRb']:.1f}", "__SLOTW__": f"{2 * D['slotR']:.1f}",
             "__DISCD__": f"{2 * D['discR']:.0f}"}.items():
    html = html.replace(k, v)
Path(OUT).write_text(html, encoding="utf-8")
print("wrote", OUT, len(html), "bytes")
