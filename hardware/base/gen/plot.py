"""Plot placement: courtyards (F red / B blue), pads, tracks, outline; report overlaps."""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
import pcbnew

b = pcbnew.LoadBoard(sys.argv[1])
out = sys.argv[2]
to = pcbnew.ToMM
fig, ax = plt.subplots(figsize=(9, 13))
ax.set_aspect("equal")
ax.invert_yaxis()

for d in b.GetDrawings():
    if d.GetLayer() == pcbnew.Edge_Cuts:
        pts = [d.GetStart(), d.GetEnd()]
        ax.plot([to(p.x) for p in pts], [to(p.y) for p in pts], "k-", lw=1)

# module bodies / wheels / bar outlines from the Fab layers
for f in b.GetFootprints():
    for g in f.GraphicalItems():
        if g.GetLayer() not in (pcbnew.F_Fab, pcbnew.B_Fab) or g.GetClass() != "PCB_SHAPE":
            continue
        col = "green" if g.GetLayer() == pcbnew.F_Fab else "purple"
        if g.GetShape() == pcbnew.SHAPE_T_RECT or g.GetShape() == pcbnew.SHAPE_T_POLY or g.GetShape() == pcbnew.SHAPE_T_SEGMENT:
            pts = [(to(c.x), to(c.y)) for c in g.GetCorners()] if g.GetShape() != pcbnew.SHAPE_T_SEGMENT else [
                (to(g.GetStart().x), to(g.GetStart().y)), (to(g.GetEnd().x), to(g.GetEnd().y))]
            if len(pts) > 2:
                pts.append(pts[0])
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=col, lw=0.6, alpha=0.7)
        elif g.GetShape() == pcbnew.SHAPE_T_CIRCLE:
            c = g.GetCenter()
            ax.add_patch(Circle((to(c.x), to(c.y)), to(g.GetRadius()), fill=False, ec=col, lw=0.6, alpha=0.7))

crt = {}
for f in b.GetFootprints():
    for layer, col in ((pcbnew.F_CrtYd, "red"), (pcbnew.B_CrtYd, "blue")):
        poly = f.GetCourtyard(layer)
        for i in range(poly.OutlineCount()):
            o = poly.Outline(i)
            pts = [(to(o.CPoint(k).x), to(o.CPoint(k).y)) for k in range(o.PointCount())]
            if pts:
                ax.add_patch(Polygon(pts, closed=True, fill=False, ec=col, lw=0.8))
                crt.setdefault(layer, []).append((f.GetReference(), poly))
    for p in f.Pads():
        c = p.GetPosition()
        r = to(max(p.GetSize().x, p.GetSize().y)) / 2
        col = "orange" if p.GetNetname() else "gray"
        ax.add_patch(Circle((to(c.x), to(c.y)), r, color=col, alpha=0.7))
    c = f.GetPosition()
    ax.text(to(c.x), to(c.y), f.GetReference(), fontsize=7, ha="center",
            color="blue" if f.IsFlipped() else "black")

for t in b.GetTracks():
    if t.GetClass() == "PCB_VIA":
        c = t.GetPosition()
        ax.add_patch(Circle((to(c.x), to(c.y)), 0.4, color="green"))
    else:
        s, e = t.GetStart(), t.GetEnd()
        ax.plot([to(s.x), to(e.x)], [to(s.y), to(e.y)],
                color="red" if t.GetLayer() == pcbnew.F_Cu else "blue",
                lw=max(0.5, to(t.GetWidth()) * 3), alpha=0.6)

ax.grid(True, lw=0.2)
ax.set_xticks(range(-60, 65, 10))
ax.set_yticks(range(-100, 35, 5))
plt.savefig(out, dpi=110, bbox_inches="tight")

# overlap report (same side courtyards)
for layer, items in crt.items():
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, pa = items[i]
            c, pc = items[j]
            if a == c:
                continue
            inter = pa.CloneDropTriangulation()
            inter.BooleanIntersection(pc)
            if inter.OutlineCount() and inter.Area() > 0:
                print("OVERLAP", "F" if layer == pcbnew.F_CrtYd else "B", a, c,
                      round(inter.Area() / 1e12, 1), "mm2")

# THT leads under the caster (B side keep-out circle of H3)
h3 = [f for f in b.GetFootprints() if f.GetReference() == "H3"][0]
cc = h3.GetPosition()
for f in b.GetFootprints():
    for p in f.Pads():
        if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
            d = ((p.GetPosition().x - cc.x) ** 2 + (p.GetPosition().y - cc.y) ** 2) ** 0.5
            if to(d) < 12.5:
                print("PIN UNDER CASTER", f.GetReference(), p.GetNumber())
x1 = min(to(d.GetStart().x) for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts)
x2 = max(to(d.GetStart().x) for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts)
y1 = min(to(d.GetStart().y) for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts)
y2 = max(to(d.GetStart().y) for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts)
for f in b.GetFootprints():
    for p in f.Pads():
        q = p.GetPosition()
        if not (x1 + 1 < to(q.x) < x2 - 1 and y1 + 1 < to(q.y) < y2 - 1):
            print("PAD NEAR/OUT EDGE", f.GetReference(), p.GetNumber(), round(to(q.x), 1), round(to(q.y), 1))
