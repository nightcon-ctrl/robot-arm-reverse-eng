"""
Checkpoint 1 helper: extract good frames from a reference video, annotate
components, and calibrate scale from a known reference feature.

Setup:
    pip install opencv-python numpy yt-dlp

Workflow:
    python checkpoint1_tool.py download "https://youtu.be/yhdL4jz74WM" -o video.mp4
    python checkpoint1_tool.py extract video.mp4 --interval 0.5 --window 3 --max 40
    # look at frames/contact_sheet.jpg, delete frames you don't want
    python checkpoint1_tool.py annotate frames/
    # annotated images + annotations.json land in annotated/

Annotate controls (click on the image window; most tools take two left clicks,
the free polygon takes as many as you like):
    mouse wheel      zoom in/out around the cursor (annotations stay pinned to the image)
    right/middle drag  pan the view
    + / -            zoom in/out around the window centre (if your wheel doesn't work)
    r                reset the view to fit the window

    b  draw a labeled box around a component (click opposite corners, type label in terminal)
    l  draw a SCALE REFERENCE line (click both ends; type name and known length in mm)
    m  measure a distance using the last scale from this frame (click both ends; optional label)
    d  draw a labeled LINE (axis, link, offset...) with no scale needed (click both ends; type label)
    c  draw a labeled CIRCLE: click the centre, then any point on the edge (radius/diameter in mm
       if the frame has a scale line)
    f  draw a labeled FREE POLYGON: click vertices one after another, then press ENTER to close it
       (the last point auto-connects to the first, so you never have to click on the start point)
    e  rename the label of the last annotation
    u  undo: last pending click if a tool is mid-draw, otherwise the last annotation
    ESC cancel the tool you are in the middle of
    n  save and go to next frame
    p  save and go to previous frame
    q  save and quit

Coordinates in annotations.json are in ORIGINAL image pixels (sub-pixel values are
possible when zoomed in). Lines also store px length and angle_deg
(0 = pointing right, counter-clockwise positive, y up) so you can rebuild geometry later.
Circles store center, edge and radius_px; polygons store pts, area_px, perimeter_px and
centroid_px. When the frame has a scale line, mm values (radius_mm, diameter_mm, area_mm2,
perimeter_mm) are stored too.
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

import cv2
import numpy as np


# ---------------------------------------------------------------- download
def cmd_download(args):
    subprocess.run(
        ["yt-dlp", "-f", "bv*[height<=1080]/b", "-o", args.out, args.url],
        check=True,
    )


# ----------------------------------------------------------------- extract
def sharpness(gray):
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def cmd_extract(args):
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"Could not open {args.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, int(fps * args.interval))
    print(f"Video: {total} frames @ {fps:.1f} fps (~{total / fps:.0f}s). Sampling every {step} frames.")

    # 1) one sequential pass, scoring sharpness at each sample point
    cands = []
    idx = 0
    while cap.grab():
        if idx % step == 0:
            ok, frame = cap.retrieve()
            if ok:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                c = {
                    "idx": idx,
                    "t": idx / fps,
                    "sharp": sharpness(gray),
                    "thumb": cv2.resize(gray, (64, 36)).astype(np.float32),
                }
                cands.append(c)
                print(f"\r  scanning: {100 * idx / max(total, 1):5.1f}%  "
                      f"t={c['t']:7.2f}s  candidates={len(cands)}", end="", flush=True)
        idx += 1
    print(f"\nSampled {len(cands)} candidate frames")

    # 2) sharpest frame per time window
    best = {}
    for c in cands:
        w = int(c["t"] // args.window)
        if w not in best or c["sharp"] > best[w]["sharp"]:
            best[w] = c
    picks = [best[w] for w in sorted(best)]

    # 3) drop near-duplicate views
    kept = []
    for c in picks:
        if all(np.abs(c["thumb"] - k["thumb"]).mean() > args.diff for k in kept):
            kept.append(c)
            print(f"  found frame at t={c['t']:7.2f}s (sharpness {c['sharp']:.0f})")

    # 4) cap the count, keeping the sharpest, then restore time order
    if len(kept) > args.max:
        kept = sorted(kept, key=lambda c: -c["sharp"])[: args.max]
        kept.sort(key=lambda c: c["idx"])
    print(f"Keeping {len(kept)} frames")

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    thumbs = []
    for n, c in enumerate(kept, 1):
        cap.set(cv2.CAP_PROP_POS_FRAMES, c["idx"])
        ok, frame = cap.read()
        if not ok:
            continue
        name = f"frame_t{c['t']:07.2f}s.png"
        cv2.imwrite(str(out / name), frame)
        print(f"  saved {n}/{len(kept)}: {name}")
        small = cv2.resize(frame, (320, 180))
        cv2.putText(small, f"{c['t']:.1f}s", (6, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)
        thumbs.append(small)

    if thumbs:
        cols = 4
        while len(thumbs) % cols:
            thumbs.append(np.zeros_like(thumbs[0]))
        rows = [np.hstack(thumbs[i:i + cols]) for i in range(0, len(thumbs), cols)]
        cv2.imwrite(str(out / "contact_sheet.jpg"), np.vstack(rows))
    print(f"Done. Saved to {out}/ (see contact_sheet.jpg)")


# ---------------------------------------------------------------- annotate
COLORS = {
    "box": (0, 200, 0),
    "scale": (0, 140, 255),
    "measure": (255, 100, 0),
    "line": (255, 0, 255),
    "circle": (0, 215, 255),
    "polygon": (255, 255, 0),
}
TOOL_HINTS = {
    "box": "BOX: click two opposite corners",
    "scale": "SCALE: click both ends of the reference feature",
    "measure": "MEASURE: click both ends",
    "line": "LINE: click both ends",
    "circle": "CIRCLE: click the centre, then a point on the edge",
    "polygon": "POLYGON: click vertices, ENTER closes it (last point joins the first)",
}
# clicks a tool needs before it completes; the polygon is open-ended and finishes on ENTER
REQUIRED_CLICKS = {"box": 2, "scale": 2, "measure": 2, "line": 2, "circle": 2}
TOOL_KEYS = {"b": "box", "l": "scale", "m": "measure", "d": "line", "c": "circle", "f": "polygon"}
HELP_LINES = (
    "b box | c circle | f polygon (ENTER closes) | l scale | m measure | d line | e rename last | u undo",
    "wheel zoom | right-drag pan | +/- zoom | r fit | ESC cancel tool | n/p next/prev frame | q quit",
)


def wheel_delta(flags):
    """Positive = wheel up. Uses OpenCV's helper when this build has it."""
    if hasattr(cv2, "getMouseWheelDelta"):
        try:
            return cv2.getMouseWheelDelta(flags)
        except cv2.error:
            pass
    return 1 if flags > 0 else -1


def seg_info(p1, p2):
    """Pixel length and angle (deg, 0 = right, CCW positive, y up) of a segment."""
    px = math.dist(p1, p2)
    ang = math.degrees(math.atan2(-(p2[1] - p1[1]), p2[0] - p1[0]))
    return px, ang + 0.0  # + 0.0 turns -0.0 into 0.0


def relabel(a):
    """(Re)build the text shown for an annotation from its fields."""
    name = a.get("name", "").strip()
    t = a["type"]
    if t == "box":
        a["label"] = name or "component"
    elif t == "scale":
        a["label"] = f"{name or 'scale'}: {a['mm']:g} mm ({a['px_per_mm']:.2f} px/mm)"
    elif t == "circle":
        name = name or "circle"
        r = a.get("radius_mm")
        a["label"] = f"{name}: r~{r:.1f} mm" if r is not None else name
    elif t == "polygon":
        name = name or "polygon"
        area = a.get("area_mm2")
        a["label"] = f"{name}: ~{area:.0f} sq mm" if area is not None else name
    else:  # measure or line
        mm = a.get("mm")
        dist = f"~{mm:.1f} mm" if mm is not None else ""
        a["label"] = f"{name}: {dist}" if name and dist else (name or dist or t)
    return a


def put_label(img, text, org, color, fs, th):
    """Text with a dark outline so it stays readable over any frame."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, fs, (0, 0, 0), th + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, fs, color, th, cv2.LINE_AA)


def put_hud(img, text, org, fs=0.55):
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fs, 1)
    x, y = org
    cv2.rectangle(img, (x - 4, y - h - 4), (x + w + 4, y + base + 2), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), 1, cv2.LINE_AA)


def draw_annotations(img, anns, tf, fs=0.6, th=2):
    """Draw annotations on img. tf maps image coords -> pixel coords of img."""
    for a in anns:
        color = COLORS.get(a["type"], (0, 200, 0))
        if a["type"] == "box":
            x, y, w, h = a["rect"]
            p1, p2 = tf((x, y)), tf((x + w, y + h))
            cv2.rectangle(img, p1, p2, color, th)
            put_label(img, a["label"], (p1[0], max(15, p1[1] - 6)), color, fs, th)
        elif a["type"] == "circle":
            c, e = tf(tuple(a["center"])), tf(tuple(a["edge"]))
            r = int(round(math.dist(c, e)))
            cv2.circle(img, c, r, color, th, cv2.LINE_AA)
            cv2.line(img, c, e, color, 1, cv2.LINE_AA)
            cv2.drawMarker(img, c, color, cv2.MARKER_CROSS, 10 + 2 * th, th)
            cv2.circle(img, e, th + 1, color, -1)
            put_label(img, a["label"], (e[0] + 8, e[1] - 8), color, fs, th)
        elif a["type"] == "polygon":
            pts = [tf(tuple(p)) for p in a["pts"]]
            cv2.polylines(img, [np.array(pts, np.int32)], True, color, th, cv2.LINE_AA)
            for p in pts:
                cv2.circle(img, p, th + 1, color, -1)
            put_label(img, a["label"], tf(tuple(a["centroid_px"])), color, fs, th)
        else:  # scale, measure or line
            p1, p2 = tf(tuple(a["pts"][0])), tf(tuple(a["pts"][1]))
            cv2.line(img, p1, p2, color, th)
            for p in (p1, p2):
                cv2.circle(img, p, th + 1, color, -1)
            mid = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2 - 8)
            put_label(img, a["label"], mid, color, fs, th)
    return img


def render_full(base, anns):
    """Annotated copy at the original resolution (what gets saved to disk)."""
    k = max(1.0, max(base.shape[:2]) / 1000.0)
    tf = lambda p: (int(round(p[0])), int(round(p[1])))
    return draw_annotations(base.copy(), anns, tf, 0.6 * k, max(2, int(round(2 * k))))


class Viewer:
    """Zoom/pan view onto one image plus the in-progress tool state."""
    ZOOM_STEP = 1.25
    MAX_ZOOM = 64.0

    def __init__(self):
        self.win = None
        self.W, self.H = 1280, 720
        self.iw = self.ih = 0
        self.zoom, self.fit_zoom = 1.0, 1.0
        self.x0 = self.y0 = 0.0          # image coords at the window's top-left corner
        self.mouse = (0, 0)
        self.pan_last = None
        self.need_fit = True
        self.mode = None                 # None | 'box' | 'scale' | 'measure' | 'line'
        self.clicks = []                 # pending clicks, in IMAGE coordinates

    def attach(self, win):
        self.win = win
        cv2.setMouseCallback(win, self.on_mouse)

    def load(self, img):
        self.ih, self.iw = img.shape[:2]
        self.need_fit = True
        self.cancel()

    def cancel(self):
        self.mode, self.clicks = None, []

    # -- geometry
    def to_img(self, sx, sy):
        return (sx / self.zoom + self.x0, sy / self.zoom + self.y0)

    def to_screen(self, p):
        return (int(round((p[0] - self.x0) * self.zoom)),
                int(round((p[1] - self.y0) * self.zoom)))

    def sync_window_size(self):
        try:
            _, _, w, h = cv2.getWindowImageRect(self.win)
            if w > 0 and h > 0:
                self.W, self.H = w, h
        except cv2.error:
            pass

    def fit(self):
        self.fit_zoom = min(self.W / self.iw, self.H / self.ih)
        self.zoom = self.fit_zoom
        self.x0 = self.iw / 2 - self.W / (2 * self.zoom)
        self.y0 = self.ih / 2 - self.H / (2 * self.zoom)
        self.need_fit = False

    def zoom_at(self, sx, sy, factor):
        """Zoom keeping the image point under screen (sx, sy) fixed."""
        ix, iy = self.to_img(sx, sy)
        z = min(max(self.zoom * factor, self.fit_zoom * 0.25), self.MAX_ZOOM)
        self.zoom = z
        self.x0 = ix - sx / z
        self.y0 = iy - sy / z

    def zoom_center(self, factor):
        self.zoom_at(self.W / 2, self.H / 2, factor)

    # -- input
    def on_mouse(self, event, x, y, flags, _):
        self.mouse = (x, y)
        if event == cv2.EVENT_MOUSEWHEEL:
            d = wheel_delta(flags)
            if d:
                self.zoom_at(x, y, self.ZOOM_STEP ** (1 if d > 0 else -1))
        elif event in (cv2.EVENT_RBUTTONDOWN, cv2.EVENT_MBUTTONDOWN):
            self.pan_last = (x, y)
        elif event in (cv2.EVENT_RBUTTONUP, cv2.EVENT_MBUTTONUP):
            self.pan_last = None
        elif event == cv2.EVENT_MOUSEMOVE and self.pan_last is not None:
            px, py = self.pan_last
            self.x0 -= (x - px) / self.zoom
            self.y0 -= (y - py) / self.zoom
            self.pan_last = (x, y)
        elif event == cv2.EVENT_LBUTTONDOWN and self.mode:
            self.clicks.append(self.to_img(x, y))

    # -- drawing
    def render(self, base, anns, title=""):
        z = self.zoom
        M = np.float32([[z, 0, -z * self.x0], [0, z, -z * self.y0]])
        interp = cv2.INTER_NEAREST if z >= 4 else cv2.INTER_LINEAR  # show real pixels when deep
        disp = cv2.warpAffine(base, M, (self.W, self.H), flags=interp,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=(30, 30, 30))
        draw_annotations(disp, anns, self.to_screen)

        if self.mode:
            color = COLORS[self.mode]
            for c in self.clicks:
                cv2.circle(disp, self.to_screen(c), 4, (0, 0, 255), -1)
            if self.clicks:
                p1 = self.to_screen(self.clicks[0])
                if self.mode == "box":
                    cv2.rectangle(disp, p1, self.mouse, color, 1)
                elif self.mode == "circle":
                    r = int(round(math.dist(p1, self.mouse)))
                    cv2.circle(disp, p1, r, color, 1, cv2.LINE_AA)
                    cv2.line(disp, p1, self.mouse, color, 1, cv2.LINE_AA)
                elif self.mode == "polygon":
                    pts = [self.to_screen(c) for c in self.clicks]
                    for a, b in zip(pts, pts[1:]):
                        cv2.line(disp, a, b, color, 2, cv2.LINE_AA)
                    cv2.line(disp, pts[-1], self.mouse, color, 1, cv2.LINE_AA)
                    if len(pts) >= 2:  # where ENTER will close the shape
                        cv2.line(disp, pts[-1], pts[0], (160, 160, 160), 1, cv2.LINE_AA)
                    cv2.circle(disp, pts[0], 9, color, 2, cv2.LINE_AA)  # start marker
                else:
                    cv2.line(disp, p1, self.mouse, color, 1, cv2.LINE_AA)
            mx, my = self.mouse
            cv2.line(disp, (mx - 12, my), (mx + 12, my), (255, 255, 255), 1)
            cv2.line(disp, (mx, my - 12), (mx, my + 12), (255, 255, 255), 1)

        status = f"{title}  zoom {z * 100:.0f}%"
        if self.mode:
            need = REQUIRED_CLICKS.get(self.mode)
            count = f"{len(self.clicks)}/{need}" if need else f"{len(self.clicks)} points"
            status += f"  |  {TOOL_HINTS[self.mode]} ({count}, ESC cancels)"
        put_hud(disp, status, (8, 22))
        for n, line in enumerate(reversed(HELP_LINES)):
            put_hud(disp, line, (8, self.H - 10 - 26 * n), fs=0.5)
        return disp


def finish_tool(mode, clicks, anns):
    """Turn clicks into an annotation, prompting in the terminal. Returns it or None."""
    scales = [s for s in anns if s["type"] == "scale"]
    ppm = scales[-1]["px_per_mm"] if scales else None

    if mode == "polygon":
        pts = [[round(c, 2) for c in p] for p in clicks]
        n = len(pts)
        if n < 3:
            print("  a polygon needs at least 3 points, discarded")
            return None
        nxt = lambda i: pts[(i + 1) % n]  # the last point wraps back to the first
        cross = [pts[i][0] * nxt(i)[1] - nxt(i)[0] * pts[i][1] for i in range(n)]
        signed = 0.5 * sum(cross)
        area = abs(signed)
        perim = sum(math.dist(pts[i], nxt(i)) for i in range(n))
        if area > 1e-9:
            cx = sum((pts[i][0] + nxt(i)[0]) * cross[i] for i in range(n)) / (6 * signed)
            cy = sum((pts[i][1] + nxt(i)[1]) * cross[i] for i in range(n)) / (6 * signed)
        else:  # degenerate (all points on a line): fall back to the mean
            cx, cy = sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n
        name = input("  polygon label (e.g. 'base plate outline'): ").strip() or "polygon"
        a = {"type": "polygon", "pts": pts, "closed": True, "name": name,
             "area_px": round(area, 2), "perimeter_px": round(perim, 2),
             "centroid_px": [round(cx, 2), round(cy, 2)]}
        if ppm:
            a["area_mm2"] = round(area / ppm ** 2, 2)
            a["perimeter_mm"] = round(perim / ppm, 2)
            print(f"  -> {n} points, ~{a['area_mm2']:.0f} mm^2, perimeter ~{a['perimeter_mm']:.1f} mm "
                  "(valid only near the reference's plane/depth)")
        else:
            print(f"  -> {n} points, {area:.0f} px^2, perimeter {perim:.0f} px")
        return relabel(a)

    p1, p2 = [[round(c, 2) for c in p] for p in clicks[:2]]
    if mode == "box":
        x, y = min(p1[0], p2[0]), min(p1[1], p2[1])
        w, h = abs(p2[0] - p1[0]), abs(p2[1] - p1[1])
        if w < 1 or h < 1:
            print("  box too small, discarded")
            return None
        name = input("  component label: ").strip() or "component"
        a = {"type": "box", "rect": [round(x, 2), round(y, 2), round(w, 2), round(h, 2)],
             "name": name}
    elif mode == "scale":
        px, ang = seg_info(p1, p2)
        if px < 1e-6:
            print("  zero-length line, discarded")
            return None
        name = input("  reference feature name (e.g. 'table hole pitch'): ").strip()
        mm = float(input("  known length in mm: "))
        if mm <= 0:
            raise ValueError("length must be positive")
        a = {"type": "scale", "pts": [p1, p2], "name": name, "mm": mm,
             "px": round(px, 3), "px_per_mm": px / mm, "angle_deg": round(ang, 2)}
        print(f"  -> {px / mm:.3f} px/mm")
    elif mode == "circle":
        r = math.dist(p1, p2)
        if r < 1:
            print("  circle too small, discarded")
            return None
        name = input("  circle label (e.g. 'joint 2 housing'): ").strip() or "circle"
        a = {"type": "circle", "center": p1, "edge": p2, "radius_px": round(r, 3), "name": name}
        if ppm:
            a["radius_mm"] = round(r / ppm, 2)
            a["diameter_mm"] = round(2 * r / ppm, 2)
            print(f"  -> radius ~{a['radius_mm']:.1f} mm, diameter ~{a['diameter_mm']:.1f} mm "
                  "(valid only near the reference's plane/depth)")
        else:
            print(f"  -> radius {r:.1f} px, diameter {2 * r:.1f} px")
    else:  # measure or line
        px, ang = seg_info(p1, p2)
        mm = px / ppm if ppm else None
        if mode == "measure":
            name = input("  line label (Enter to skip): ").strip()
        else:
            name = input("  line label (e.g. 'link 2 axis'): ").strip() or "line"
        a = {"type": mode, "pts": [p1, p2], "name": name,
             "px": round(px, 3), "angle_deg": round(ang, 2)}
        if mm is not None:
            a["mm"] = round(mm, 2)
            print(f"  -> ~{mm:.1f} mm (valid only near the reference's plane/depth)")
        print(f"  -> {px:.1f} px, {ang:.1f} deg")
    return relabel(a)


def commit(v, anns):
    """Finish the active tool: prompt in the terminal, store the annotation, reset the tool."""
    try:
        a = finish_tool(v.mode, v.clicks, anns)
        if a:
            anns.append(a)
    except (ValueError, EOFError) as e:
        print(f"  invalid input ({e}); annotation discarded")
    v.cancel()


def cmd_annotate(args):
    src = Path(args.folder)
    files = sorted(p for p in src.glob("*.png"))
    if not files:
        raise SystemExit("No PNG frames found")
    out = Path(args.out)
    out.mkdir(exist_ok=True)
    db_path = out / "annotations.json"
    db = json.loads(db_path.read_text()) if db_path.exists() else {}

    win = "annotate"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    v = Viewer()
    v.attach(win)

    def save(f, base, anns):
        db[f.name] = anns
        cv2.imwrite(str(out / f.name), render_full(base, anns))
        db_path.write_text(json.dumps(db, indent=2))

    i = 0
    while 0 <= i < len(files):
        f = files[i]
        base = cv2.imread(str(f))
        anns = db.get(f.name, [])
        v.load(base)
        print(f"\n[{i + 1}/{len(files)}] {f.name}")
        while True:
            v.sync_window_size()
            if v.need_fit:
                v.fit()
            cv2.imshow(win, v.render(base, anns, f"[{i + 1}/{len(files)}] {f.name}"))
            k = cv2.waitKey(15) & 0xFF

            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:  # window closed
                save(f, base, anns)
                return

            need = REQUIRED_CLICKS.get(v.mode)
            if need and len(v.clicks) >= need:
                commit(v, anns)
                continue

            if k == 255:
                continue
            if k == 27:  # ESC
                v.cancel()
            elif k in (13, 10) and v.mode == "polygon":  # ENTER closes the polygon
                if len(v.clicks) < 3:
                    print("  a polygon needs at least 3 points (keep clicking, then ENTER)")
                else:
                    commit(v, anns)
            elif k < 128 and chr(k) in TOOL_KEYS:
                mode = TOOL_KEYS[chr(k)]
                if mode == "measure" and not any(a["type"] == "scale" for a in anns):
                    print("  draw a scale line first (l)")
                else:
                    v.mode, v.clicks = mode, []
            elif k == ord("e") and anns:
                a = anns[-1]
                print(f"  current label: {a['label']}")
                name = input("  new label: ").strip()
                if name:
                    a["name"] = name
                    relabel(a)
            elif k == ord("u"):
                if v.mode and v.clicks:
                    v.clicks.pop()
                elif anns:
                    anns.pop()
            elif k in (ord("+"), ord("=")):
                v.zoom_center(Viewer.ZOOM_STEP)
            elif k in (ord("-"), ord("_")):
                v.zoom_center(1 / Viewer.ZOOM_STEP)
            elif k == ord("r"):
                v.need_fit = True
            elif k in (ord("n"), ord("p"), ord("q")):
                save(f, base, anns)
                if k == ord("q"):
                    cv2.destroyAllWindows()
                    return
                i += 1 if k == ord("n") else -1
                break
    cv2.destroyAllWindows()


# -------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("download")
    d.add_argument("url")
    d.add_argument("-o", "--out", default="video.mp4")
    d.set_defaults(fn=cmd_download)

    e = sub.add_parser("extract")
    e.add_argument("video")
    e.add_argument("--out", default="frames")
    e.add_argument("--interval", type=float, default=0.5, help="sampling step in seconds")
    e.add_argument("--window", type=float, default=3.0, help="keep sharpest frame per N seconds")
    e.add_argument("--diff", type=float, default=6.0, help="min visual difference to count as a new view")
    e.add_argument("--max", type=int, default=40)
    e.set_defaults(fn=cmd_extract)

    a = sub.add_parser("annotate")
    a.add_argument("folder")
    a.add_argument("--out", default="annotated")
    a.set_defaults(fn=cmd_annotate)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()