"""Build base.kicad_pcb from design.py with pcbnew (placement + nets, no tracks)."""
import sys

import pcbnew

from design import MECH, PARTS, symbol_uuid

PRJ = sys.argv[1]  # .../hardware/base
OUTLINE = (-36.5, -95.3, 36.5, 21.5)  # x1 y1 x2 y2
CORNER = 3.0

# pad nets come from KiCad's own netlist export, so the board matches the
# schematic exactly (including unconnected-(...) pins)
from sexpr import first, find, parse
NETLIST = {}
_n = parse(open(sys.argv[2]).read())[0]
for _net in find(first(_n, "nets"), "net"):
    _name = str(first(_net, "name")[1])
    for _node in find(_net, "node"):
        NETLIST[(str(first(_node, "ref")[1]), str(first(_node, "pin")[1]))] = _name
board = pcbnew.BOARD()
board.SetFileName(f"{PRJ}/base.kicad_pcb")
mm = pcbnew.FromMM


def P(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def lib_path(lib):
    if lib == "Base":
        return f"{PRJ}/Base.pretty"
    return f"/usr/share/kicad/footprints/{lib}.pretty"


nets = {}


def net(name):
    if name not in nets:
        n = pcbnew.NETINFO_ITEM(board, name)
        board.Add(n)
        nets[name] = n
    return nets[name]


def add_fp(ref, fpid, pos, value="", pins=None, sym_uuid=None, centre=False):
    lib, name = fpid.split(":")
    f = pcbnew.FootprintLoad(lib_path(lib), name)
    assert f is not None, fpid
    f.SetFPID(pcbnew.LIB_ID(lib, name))
    f.SetReference(ref)
    f.SetValue(value or name)
    if sym_uuid:
        f.SetAttributes(f.GetAttributes() & ~pcbnew.FP_EXCLUDE_FROM_BOM)
    x, y, rot, side = pos
    board.Add(f)
    f.SetPosition(P(x, y))
    f.SetOrientationDegrees(rot)
    if side == "B":
        f.Flip(P(x, y), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    if centre:
        bb = f.GetCourtyard(pcbnew.B_CrtYd if side == "B" else pcbnew.F_CrtYd).BBox()
        c = bb.Centre()
        f.Move(pcbnew.VECTOR2I(mm(x) - c.x, mm(y) - c.y))
        # snap the first pad to the 0.254 mm router grid: 2.54 mm rows then sit
        # on-grid and a 0.2 mm track can pass between neighbouring pins
        p0 = sorted(f.Pads(), key=lambda q: q.GetNumber())[0].GetPosition()
        g = mm(0.254)
        f.Move(pcbnew.VECTOR2I(round(p0.x / g) * g - p0.x, round(p0.y / g) * g - p0.y))
    if sym_uuid:
        f.SetPath(pcbnew.KIID_PATH(f"/{sym_uuid}"))
        f.SetSheetname("/")
        f.SetSheetfile("base.kicad_sch")
    for p in f.Pads():
        num = p.GetNumber()
        if pins is not None and num:
            n = NETLIST.get((ref, num))
            if n:
                p.SetNet(net(n))
    return f


for ref, (lib_id, fpid, value, pins, pos) in PARTS.items():
    add_fp(ref, fpid, pos, value, pins, symbol_uuid(ref), centre=True)
for ref, (fpid, pos) in MECH.items():
    add_fp(ref, fpid, pos)

# outline with rounded corners
x1, y1, x2, y2 = OUTLINE
r = CORNER


def seg(a, b):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(P(*a))
    s.SetEnd(P(*b))
    s.SetLayer(pcbnew.Edge_Cuts)
    s.SetWidth(mm(0.1))
    board.Add(s)


def arc(c, start, angle):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_ARC)
    s.SetCenter(P(*c))
    s.SetStart(P(*start))
    s.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T), True)
    s.SetLayer(pcbnew.Edge_Cuts)
    s.SetWidth(mm(0.1))
    board.Add(s)


seg((x1 + r, y1), (x2 - r, y1))
seg((x2, y1 + r), (x2, y2 - r))
seg((x2 - r, y2), (x1 + r, y2))
seg((x1, y2 - r), (x1, y1 + r))
arc((x2 - r, y1 + r), (x2 - r, y1), 90)
arc((x2 - r, y2 - r), (x2, y2 - r), 90)
arc((x1 + r, y2 - r), (x1 + r, y2), 90)
arc((x1 + r, y1 + r), (x1, y1 + r), 90)

def label(txt, x, y, size=1.0, layer=pcbnew.F_SilkS, mirror=False):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(txt)
    t.SetPosition(P(x, y))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(size * 0.15))
    if mirror:
        t.SetMirrored(True)
    board.Add(t)


def pad_xy(ref, num):
    f = board.FindFootprintByReference(ref)
    q = [p for p in f.Pads() if p.GetNumber() == num][0].GetPosition()
    return pcbnew.ToMM(q.x), pcbnew.ToMM(q.y)


# polarity marks: the board has no reverse-battery protection
x, y = pad_xy("J1", "1"); label("BAT+", x, y - 3.0)
x, y = pad_xy("J1", "2"); label("GND", x, y - 3.0)
x, y = pad_xy("J2", "1"); label("SW", (x + pad_xy("J2", "2")[0]) / 2, y - 3.0)
x, y = pad_xy("J3", "1"); label("GND IR D1 ........ D8 VCC", x + 12.7, y - 2.2, 0.8)
label("LF-Bot base v0.5 - 2026-11", 0, -22.0, 1.2, pcbnew.B_SilkS, mirror=True)

# keep tracks and vias out from under screw heads / nuts (whole slot length):
# real KiCad rule areas, so DRC and any router honour them
import math as _m
for f in board.GetFootprints():
    for p in f.Pads():
        if p.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH:
            continue
        dr = p.GetDrillSize()
        r = pcbnew.ToMM(min(dr.x, dr.y)) / 2
        L = pcbnew.ToMM(max(dr.x, dr.y)) / 2 - r
        head = 3.3 if r > 1.3 else 2.4
        c = p.GetPosition()
        cx, cy = pcbnew.ToMM(c.x), pcbnew.ToMM(c.y)
        ang = _m.radians(p.GetOrientation().AsDegrees())
        ux, uy = (_m.cos(ang), -_m.sin(ang)) if dr.x >= dr.y else (_m.sin(ang), _m.cos(ang))
        from shapely.geometry import LineString, Point
        g = (LineString([(cx - ux * L, cy - uy * L), (cx + ux * L, cy + uy * L)]).buffer(head, 8)
             if L > 0.01 else Point(cx, cy).buffer(head, 8))
        pts = list(g.exterior.coords)[:-1]
        z = pcbnew.ZONE(board)
        z.SetIsRuleArea(True)
        z.SetDoNotAllowTracks(True)
        z.SetDoNotAllowVias(True)
        z.SetDoNotAllowPads(False)
        z.SetDoNotAllowZoneFills(False)
        z.SetDoNotAllowFootprints(False)
        ls = pcbnew.LSET()
        ls.AddLayer(pcbnew.F_Cu)
        ls.AddLayer(pcbnew.B_Cu)
        z.SetLayerSet(ls)
        z.SetZoneName(f"screw {f.GetReference()}")
        ol = z.Outline()
        ol.NewOutline()
        for x, y in pts:
            ol.Append(mm(x), mm(y))
        board.Add(z)

# net classes are defined in base.kicad_pro (patterns); board defaults here
ds = board.GetDesignSettings()
ds.SetCopperLayerCount(2)
pcbnew.SaveBoard(f"{PRJ}/base.kicad_pcb", board)
print("saved", len(PARTS) + len(MECH), "footprints", len(nets), "nets")
