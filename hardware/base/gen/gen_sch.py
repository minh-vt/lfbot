"""Generate a KiCad schematic from design.py, drawn with wires.

Parts sit in functional blocks (LAYOUT); every signal and battery net is
drawn as wires by a small grid router, GND / +3V3 / +5V use power symbols,
and each wired net carries one label, only to keep its name (the board and
its scripts refer to nets by name).

Router: 2.54 mm grid, one net at a time, each pin joined to the net's tree
by A* (bends and crossings cost extra). Wires of different nets may cross
only straight through each other, never touch at an end or bend and never
run along each other, so no two nets can join; the netlist is then checked
against design.py.

usage: gen_sch.py OUT.kicad_sch   (the project name, i.e. the file name,
                                   picks the variant in design.VARIANT_PINS)
"""
import copy
import heapq
import math
import os
import subprocess
import sys
import tempfile
import uuid
from collections import defaultdict

from design import UUID_NS, parts_for, symbol_uuid
from sexpr import Q, dump, find, first, parse

OUT = sys.argv[1]
PROJECT = os.path.splitext(os.path.basename(OUT))[0]  # KiCad keys instances by it
PARTS = parts_for(PROJECT)
G = 2.54
POWER = {"GND": "power:GND", "+3V3": "power:+3V3", "+5V": "power:+5V"}
LIBDIRS = {"PCM_Espressif": os.path.expanduser(
    "~/.local/share/kicad/10.0/3rdparty/symbols/"
    "com_github_espressif_kicad-libraries/Espressif.kicad_sym")}
PAPER, PAGE = "A2", (594, 420)

# ref: (x, y, angle, mirror) in mm; the origin is nudged by 1.27 mm where
# needed so every wired pin lands on the 2.54 mm grid.
IR_ROWS = [3, 8, 7, 6, 5, 4, 2, 1]  # channels in the ESP32's pin order, top down
X_NODE = 330.2
LAYOUT = {
    "U1": (254.0, 203.2, 0, False),
    # IR bar, its dividers (one row per channel) and the IR_EN selector
    "J3": (398.78, 190.5, 0, False),
    "J4": (375.92, 124.46, 0, True),
    # I2C devices above the ESP32
    "J10": (203.2, 109.22, 0, False),
    "J5": (238.76, 109.22, 0, False),
    "J6": (279.4, 109.22, 0, False),
    # motor drivers and encoders left of the ESP32, pins toward it
    "U2": (149.86, 129.54, 0, True),
    "J7": (149.86, 165.1, 0, True),
    "U3": (149.86, 203.2, 0, True),
    "J8": (149.86, 238.76, 0, True),
    # buttons below the ESP32
    "J9": (276.86, 289.56, 0, True),
    "SW1": (304.8, 281.94, 0, False),
    "SW2": (304.8, 297.18, 0, False),
    # power: battery, fuse, switch, bulk cap, buck and diode, battery sense
    "J1": (50.8, 320.04, 0, True),
    "F1": (74.93, 320.04, 90, False),
    "J2": (101.6, 320.04, 0, False),
    "C1": (116.84, 334.01, 0, False),
    "U4": (162.56, 330.2, 0, False),
    "D1": (177.8, 345.44, 180, False),
    "R1": (193.04, 323.85, 0, False),
    "R2": (193.04, 339.09, 0, False),
    "C2": (205.74, 339.09, 0, False),
}
for _row, _ch in enumerate(IR_ROWS):
    _y = 139.7 + 17.78 * _row
    LAYOUT[f"R1{_ch - 1}"] = (X_NODE + 6.35, _y, 270, False)
    LAYOUT[f"R2{_ch - 1}"] = (X_NODE, _y + 6.35, 0, False)
FLAGS_AT = (30.48, 393.7)  # PWR_FLAG for each power net, side by side
SHIFT = (76.2, -30.48)  # the whole drawing, to centre it on the page


# ---------------------------------------------------------------- symbols
_libs = {}


def lib_symbol(lib_id):
    lib, name = lib_id.split(":")
    path = LIBDIRS.get(lib, f"/usr/share/kicad/symbols/{lib}.kicad_sym")
    if path not in _libs:
        _libs[path] = parse(open(path).read())[0]
    for s in find(_libs[path], "symbol"):
        if s[1] == name:
            s = list(s)
            s[1] = Q(lib_id)
            return s
    raise KeyError(lib_id)


def lib_geometry(sym):
    """Pins [(num, x, y, angle, length)] and the body box, in library mm (y up)."""
    pins, pts = [], []

    def walk(n):
        for x in n:
            if not isinstance(x, list) or not x:
                continue
            if x[0] == "pin":
                at, ln = first(x, "at"), first(x, "length")
                pins.append((str(first(x, "number")[1]), float(at[1]), float(at[2]),
                             int(float(at[3])), float(ln[1]) if ln else 0.0))
            elif x[0] in ("rectangle",):
                pts.extend([(float(first(x, k)[1]), float(first(x, k)[2])) for k in ("start", "end")])
            elif x[0] == "polyline":
                pts.extend((float(p[1]), float(p[2])) for p in find(first(x, "pts"), "xy"))
            elif x[0] == "circle":
                c, r = first(x, "center"), float(first(x, "radius")[1])
                pts.extend([(float(c[1]) - r, float(c[2]) - r), (float(c[1]) + r, float(c[2]) + r)])
            elif x[0] == "arc":
                pts.extend((float(first(x, k)[1]), float(first(x, k)[2])) for k in ("start", "mid", "end"))
            elif x[0] == "symbol":
                walk(x)
    walk(sym)
    return pins, pts


def xf(px, py, angle, mirror):
    """Library point (y up) to an offset on the sheet (y down): KiCad rotates,
    then mirrors (checked against a netlist)."""
    x, y = px, -py
    c, s = round(math.cos(math.radians(angle))), round(math.sin(math.radians(angle)))
    x, y = x * c + y * s, -x * s + y * c
    return (-x if mirror else x), y


DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def dir_index(dx, dy):
    dx, dy = round(dx, 6), round(dy, 6)
    return DIRS.index((int(math.copysign(1, dx)) if dx else 0, int(math.copysign(1, dy)) if dy else 0))


class Part:
    def __init__(self, ref, lib_id, x, y, angle, mirror, nets):
        self.ref, self.lib_id, self.angle, self.mirror, self.nets = ref, lib_id, angle, mirror, nets
        self.sym = lib_symbol(lib_id)
        lib_pins, pts = lib_geometry(self.sym)
        # nudge the origin so the first pin off the power nets sits on the grid
        wired = [p for p in lib_pins if self.net(p[0]) not in POWER] or lib_pins
        dx, dy = xf(wired[0][1], wired[0][2], angle, mirror)
        self.x = x + (-(x + dx) % G if abs(-(x + dx) % G - G) > 0.01 else 0)
        self.y = y + (-(y + dy) % G if abs(-(y + dy) % G - G) > 0.01 else 0)
        self.x, self.y = round(self.x, 3), round(self.y, 3)
        self.pins = []  # (num, sheet x, sheet y, outward direction index)
        for num, px, py, ang, ln in lib_pins:
            ox, oy = xf(px, py, angle, mirror)
            bx, by = xf(px + math.cos(math.radians(ang)), py + math.sin(math.radians(ang)), angle, mirror)
            out = dir_index(ox - bx, oy - by)
            self.pins.append((num, round(self.x + ox, 3), round(self.y + oy, 3), out))
            pts.append((px, py))
        corners = [xf(px, py, angle, mirror) for px, py in pts]
        self.box = (self.x + min(c[0] for c in corners), self.y + min(c[1] for c in corners),
                    self.x + max(c[0] for c in corners), self.y + max(c[1] for c in corners))

    def net(self, num):
        return self.nets.get(int(num)) if num.isdigit() else self.nets.get(num)


# ------------------------------------------------------------------- grid
def cell(x, y):
    return (round(x / G), round(y / G))


def on_grid(x, y):
    return abs(x / G - round(x / G)) < 0.01 and abs(y / G - round(y / G)) < 0.01


class Grid:
    def __init__(self):
        self.block = set()           # nothing may enter
        self.pin = {}                # pin end -> net (None: no-connect)
        self.wire = defaultdict(dict)  # cell -> {net: set('H','V','N')}
        self.reserved = {}           # first step out of a pin -> net

    def block_box(self, x0, y0, x1, y1, outer=True):
        """Block the cells over a box: `outer` takes every cell the box touches
        (bodies: no wire grazes them), else only cells whose point is inside
        (text)."""
        lo, hi = (math.floor, math.ceil) if outer else (math.ceil, math.floor)
        for i in range(lo(x0 / G + 0.01), hi(x1 / G - 0.01) + 1):
            for j in range(lo(y0 / G + 0.01), hi(y1 / G - 0.01) + 1):
                if (i, j) not in self.pin:
                    self.block.add((i, j))

    def free_for(self, c, net):
        if c in self.block or (c in self.pin and self.pin[c] != net):
            return False
        if self.reserved.get(c, net) != net:
            return False
        return all(n == net for n in self.wire.get(c, {}))

    def add_path(self, net, cells):
        for k, c in enumerate(cells):
            kinds = self.wire[c].setdefault(net, set())
            if 0 < k < len(cells) - 1 and (cells[k - 1][0] == cells[k + 1][0] or
                                           cells[k - 1][1] == cells[k + 1][1]):
                kinds.add("H" if cells[k - 1][1] == c[1] else "V")
            else:
                kinds.add("N")


def route(grid, net, start, goals, tree):
    """A* from pin `start` (cell, outward dir) to a goal pin {cell: arrive dir}
    or any cell of the net's `tree`. Returns the cell path or None."""
    (sc, sd) = start
    gx = [c[0] for c in list(goals) + list(tree)]
    gy = [c[1] for c in list(goals) + list(tree)]
    bx0, bx1, by0, by1 = min(gx), max(gx), min(gy), max(gy)

    def h(c):
        return max(bx0 - c[0], 0, c[0] - bx1) + max(by0 - c[1], 0, c[1] - by1)

    W, H_ = round(PAGE[0] / G), round(PAGE[1] / G)
    openq = [(h(sc), 0, sc, sd, 0)]
    came, best = {}, {(sc, sd): 0}
    tie = 0
    while openq:
        f, g, c, d, steps = heapq.heappop(openq)
        if best.get((c, d), 1e9) < g:
            continue
        others = {n: k for n, k in grid.wire.get(c, {}).items() if n != net}
        for nd in range(4):
            if steps == 0 and nd != sd:
                continue
            if nd == (d + 2) % 4:
                continue
            if others and nd != d:
                continue  # straight through a crossing
            n = (c[0] + DIRS[nd][0], c[1] + DIRS[nd][1])
            if not (2 <= n[0] < W - 2 and 2 <= n[1] < H_ - 2):
                continue
            here = grid.wire.get(n, {})
            other_here = {k: v for k, v in here.items() if k != net}
            arrive = None
            if n in goals and goals[n] == nd:
                arrive = n
            elif n in tree and net in here and not other_here and n not in grid.pin:
                arrive = n
            elif n in tree and n in grid.pin and grid.pin[n] == net and not other_here:
                arrive = n
            if arrive is None:
                if n in grid.block or n in grid.pin or n in tree or net in here:
                    continue
                if grid.reserved.get(n, net) != net:
                    continue
                if other_here:
                    orient = "H" if nd % 2 == 0 else "V"
                    if any("N" in k or orient in k for k in other_here.values()):
                        continue
            cost = 1 + (4 if nd != d and steps > 0 else 0) + (3 if other_here else 0)
            for side in ((nd + 1) % 4, (nd + 3) % 4):
                m = (n[0] + DIRS[side][0], n[1] + DIRS[side][1])
                if any(k != net for k in grid.wire.get(m, {})):
                    cost += 0.6
            ng = g + cost
            if ng < best.get((n, nd), 1e9):
                best[(n, nd)] = ng
                came[(n, nd)] = (c, d)
                if arrive is not None:
                    path, s = [n], (c, d)
                    while s in came:
                        path.append(s[0])
                        s = came[s]
                    if path[-1] != sc:
                        path.append(sc)
                    return path[::-1]
                tie += 1
                heapq.heappush(openq, (ng + h(n), ng, n, nd, steps + 1))
    return None


# ------------------------------------------------------------- sheet items
_cnt = [0]


def U():
    _cnt[0] += 1
    return Q(str(uuid.uuid5(UUID_NS, f"item{_cnt[0]}")))


def field_angle(angle):
    """KiCad keeps field text at 0 or 90 degrees and turns it with the
    symbol: 90 on a symbol turned a quarter reads level."""
    return 90 if angle % 180 == 90 else 0


def mm(v):
    return f"{v:.3f}".rstrip("0").rstrip(".")


def font(size=1.27, justify=None, hide=False):
    e = ["effects", ["font", ["size", mm(size), mm(size)]]]
    if justify:
        e.append(["justify", *justify])
    out = []
    if hide:
        out.append(["hide", "yes"])
    return out + [e]


def prop(name, value, x, y, hide=False, justify=None, angle=0):
    return ["property", Q(name), Q(value), ["at", mm(x), mm(y), str(angle)],
            *font(justify=justify, hide=hide)]


ROOT = str(uuid.uuid5(UUID_NS, "root"))
items, lib_syms, power_refs = [], {}, defaultdict(int)


def text_len(s, size=1.27):
    return len(s) * size * 0.85


def symbol_item(ref, lib_id, x, y, angle, mirror, fields, uid, pins, in_bom=True):
    lib_syms[lib_id] = lib_symbol(lib_id)
    s = ["symbol", ["lib_id", Q(lib_id)], ["at", mm(x), mm(y), str(angle)]]
    if mirror:
        s.append(["mirror", "y"])
    s += [["unit", "1"], ["body_style", "1"], ["exclude_from_sim", "no"],
          ["in_bom", "yes" if in_bom else "no"], ["on_board", "yes" if in_bom else "no"],
          ["in_pos_files", "yes" if in_bom else "no"], ["dnp", "no"], ["uuid", Q(uid)], *fields]
    for num in dict.fromkeys(pins):
        s.append(["pin", Q(num), ["uuid", U()]])
    s.append(["instances", ["project", Q(PROJECT),
                            ["path", Q("/" + ROOT), ["reference", Q(ref)], ["unit", "1"]]]])
    items.append(s)


def power_symbol(grid, net, x, y, point):
    """Power symbol with its pin at (x, y), body pointing `point` (dir index)."""
    lib_id = "power:PWR_FLAG" if net == "PWR_FLAG" else POWER[net]
    natural = 3 if net != "GND" else 1  # supplies point up, GND down
    angle = {0: 0, 1: 90, 2: 180, 3: 270}[(natural - point) % 4]
    prefix = "#FLG" if net == "PWR_FLAG" else "#PWR"
    power_refs[prefix] += 1
    ref = f"{prefix}{power_refs[prefix]:02d}"
    dx, dy = DIRS[point]
    value = "PWR_FLAG" if net == "PWR_FLAG" else net
    # centred text beyond the tip; the field angle undoes the symbol's turn
    reach = 3.81 + (text_len(value) / 2 if point in (0, 2) else 0.4)
    tx, ty = x + dx * reach, y + dy * reach
    fields = [prop("Reference", ref, x, y, hide=True),
              prop("Value", value, tx, ty, angle=field_angle(angle)),
              prop("Footprint", "", x, y, hide=True), prop("Datasheet", "", x, y, hide=True),
              prop("Description", "", x, y, hide=True)]
    symbol_item(ref, lib_id, x, y, angle, False, fields,
                str(uuid.uuid5(UUID_NS, f"pwr{ref}")), ["1"], in_bom=False)
    c = cell(x, y)
    for k in (1, 2):
        grid.block.add((c[0] + dx * k, c[1] + dy * k))
    if point in (0, 2):  # horizontal text beyond the tip
        for k in range(3, 3 + math.ceil(text_len(value) / G)):
            grid.block.add((c[0] + dx * k, c[1]))
    else:
        for k in (-1, 0, 1):
            grid.block.add((c[0] + k, c[1] + dy * 2))


def wire(a, b):
    items.append(["wire", ["pts", ["xy", mm(a[0]), mm(a[1])], ["xy", mm(b[0]), mm(b[1])]],
                  ["stroke", ["width", "0"], ["type", "default"]], ["uuid", U()]])


# ------------------------------------------------------------------ build
grid = Grid()
parts = {}
for ref, (lib_id, fp, value, nets) in PARTS.items():
    x, y, angle, mirror = LAYOUT[ref]
    parts[ref] = Part(ref, lib_id, x + SHIFT[0], y + SHIFT[1], angle, mirror, nets)

# bodies, pin ends and fields
field_items = {}
for ref, p in parts.items():
    lib_id, fp, value, nets = PARTS[ref]
    x0, y0, x1, y1 = p.box
    grid.block_box(x0, y0, x1, y1)
    sides = {d for _, _, _, d in p.pins}
    ang = field_angle(p.angle)
    w = max(text_len(ref), text_len(value))
    cy = (y0 + y1) / 2
    if ref == "U1":
        # above the top left corner, clear of the 5V / 3V3 stubs
        rf = prop("Reference", ref, x0, y0 - 13.97, justify=["left"])
        vf = prop("Value", value, x0, y0 - 11.43, justify=["left"])
        grid.block_box(x0, y0 - 15.24, x0 + text_len(value), y0 - 10.16, outer=False)
    # centred text: KiCad turns a field's justification with the symbol
    elif sides <= {2} or (1 in sides or 3 in sides) and 0 not in sides:
        tx = x1 + 1.27 + w / 2  # pins left, up or down: text on the right
        rf = prop("Reference", ref, tx, cy - 1.27, angle=ang)
        vf = prop("Value", value, tx, cy + 1.27, angle=ang)
        grid.block_box(x1, cy - 2.6, x1 + 1.27 + w, cy + 2.6, outer=False)
    elif sides <= {0}:  # pins on the right: text on the left
        tx = x0 - 1.27 - w / 2
        rf = prop("Reference", ref, tx, cy - 1.27, angle=ang)
        vf = prop("Value", value, tx, cy + 1.27, angle=ang)
        grid.block_box(x0 - 1.27 - w, cy - 2.6, x0, cy + 2.6, outer=False)
    else:  # pins left and right: text above and below
        cx = (x0 + x1) / 2
        rf = prop("Reference", ref, cx, y0 - 1.9, angle=ang)
        vf = prop("Value", value, cx, y1 + 1.9, angle=ang)
        grid.block_box(cx - w / 2, y0 - 3.2, cx + w / 2, y0 - 0.6, outer=False)
        grid.block_box(cx - w / 2, y1 + 0.6, cx + w / 2, y1 + 3.2, outer=False)
    field_items[ref] = [rf, vf, prop("Footprint", fp, p.x, p.y, hide=True),
                        prop("Datasheet", "~", p.x, p.y, hide=True),
                        prop("Description", "", p.x, p.y, hide=True)]
    for num, px, py, d in p.pins:
        net = p.net(num)
        if net in POWER and not on_grid(px, py):
            continue
        assert on_grid(px, py), (ref, num, px, py)
        c = cell(px, py)
        grid.block.discard(c)
        grid.pin[c] = net
        # the pin line itself
        for k in range(1, 3):
            inner = (c[0] - DIRS[d][0] * k, c[1] - DIRS[d][1] * k)
            if inner not in grid.pin:
                grid.block.add(inner)
        if net is not None and net not in POWER:
            grid.reserved[(c[0] + DIRS[d][0], c[1] + DIRS[d][1])] = net

for ref, p in parts.items():
    lib_id, fp, value, nets = PARTS[ref]
    symbol_item(ref, lib_id, p.x, p.y, p.angle, p.mirror, field_items[ref], symbol_uuid(ref),
                [num for num, *_ in p.pins])

# no-connect flags
for ref, p in parts.items():
    for num, px, py, d in p.pins:
        if p.net(num) is None:
            items.append(["no_connect", ["at", mm(px), mm(py)], ["uuid", U()]])

# power symbols: on the ESP32 top pins by hand (half-grid, side by side),
# elsewhere one per pin end at the end of a short stub
done = set()
u1 = parts["U1"]
for num, px, py, d in u1.pins:
    net = u1.net(num)
    if net in POWER and not on_grid(px, py) and (px, py) not in done:
        done.add((px, py))
        side = -1 if net == "+5V" else 1
        a, b = (px, py - G), (px + side * 2 * G, py - G)
        wire((px, py), a)
        wire(a, b)
        power_symbol(grid, net, b[0], b[1], 3)
        for q in (a, b):
            grid.block.add(cell(q[0], q[1]))


def place_power(net, px, py, d, neighbours):
    """Power symbol on a pin end. A sideways pin gets GND down / supplies up,
    unless a pin of the part sits right beside it: then the symbol points
    along the pin, so it does not block the neighbour's way out."""
    c = cell(px, py)
    prefs = [d]
    if d in (0, 2) and not neighbours:
        prefs = [1 if net == "GND" else 3, d]
    for point in prefs:
        for length in ((0, 1, 2, 3) if point == d else (1, 2, 3)):
            end = (c[0] + DIRS[d][0] * length, c[1] + DIRS[d][1] * length)
            body = [(end[0] + DIRS[point][0] * k, end[1] + DIRS[point][1] * k) for k in (1, 2)]
            stub = [(c[0] + DIRS[d][0] * k, c[1] + DIRS[d][1] * k) for k in range(1, length + 1)]
            if point == (d + 2) % 4 and length < 3:
                continue
            if all(grid.free_for(q, None) and q not in grid.reserved for q in stub + body):
                ex, ey = end[0] * G, end[1] * G
                if length:
                    wire((px, py), (ex, ey))
                    grid.add_path(net, [c] + stub)
                power_symbol(grid, net, ex, ey, point)
                return True
    return False


failed = []
for ref, p in parts.items():
    for num, px, py, d in p.pins:
        net = p.net(num)
        if net in POWER and on_grid(px, py) and (px, py) not in done:
            done.add((px, py))
            side = [(DIRS[d][1], DIRS[d][0]), (-DIRS[d][1], -DIRS[d][0])]
            near = any(abs(qx - px - sx * G) < 0.01 and abs(qy - py - sy * G) < 0.01
                       for _, qx, qy, _ in p.pins for sx, sy in side)
            if not place_power(net, px, py, d, near):
                failed.append(f"power {ref}.{num}")

# PWR_FLAG: each power net once, a power symbol and a flag joined by a wire
fx, fy = FLAGS_AT[0] + SHIFT[0], FLAGS_AT[1] + SHIFT[1]
for i, net in enumerate(POWER):
    x = fx + i * 6 * G
    point = 1 if net == "GND" else 3
    power_symbol(grid, net, x, fy, point)
    wire((x, fy), (x + 2 * G, fy))
    power_symbol(grid, "PWR_FLAG", x + 2 * G, fy, 3)

# wired nets
terminals = defaultdict(list)  # net -> [(cell, outward dir)]
for ref, p in parts.items():
    seen = set()
    for num, px, py, d in p.pins:
        net = p.net(num)
        if net is None or net in POWER or (px, py) in seen:
            continue
        seen.add((px, py))
        terminals[net].append((cell(px, py), d))


def span(net):
    xs = [c[0] for c, _ in terminals[net]]
    ys = [c[1] for c, _ in terminals[net]]
    return (max(xs) - min(xs)) + (max(ys) - min(ys))


def route_net(grid, net):
    """Join the net's pins one by one (nearest to the tree first)."""
    terms = terminals[net]
    tree, edges, fails = {terms[0][0]}, [], []
    todo, first_goal = terms[1:], True
    while todo:
        todo.sort(key=lambda t: min(abs(t[0][0] - c[0]) + abs(t[0][1] - c[1]) for c in tree))
        t = todo.pop(0)
        goals = {terms[0][0]: (terms[0][1] + 2) % 4} if first_goal else {}
        path = route(grid, net, t, goals, set() if first_goal else tree)
        if path is None and first_goal:
            path = route(grid, net, terms[0], {t[0]: (t[1] + 2) % 4}, set())
        if path is None:
            fails.append(f"{net} from {t[0]}")
            continue
        first_goal = False
        grid.add_path(net, path)
        tree.update(path)
        edges.extend(zip(path, path[1:]))
    return tree, edges, fails


# Rip up and retry: a net that found no way is routed earlier next time.
order = sorted(terminals, key=span)
base = copy.deepcopy(grid)
for attempt in range(8):
    grid = copy.deepcopy(base)
    net_cells, route_fails = {}, []
    for net in order:
        tree, edges, fails = route_net(grid, net)
        net_cells[net] = (tree, edges)
        route_fails += fails
    if not route_fails:
        break
    stuck = list(dict.fromkeys(f.split(" from ")[0] for f in route_fails))
    order = stuck + [n for n in order if n not in stuck]
failed += route_fails

# wires from the graph: straight runs between nodes; junctions at 3+ ways
labels_at = {}
for net, (tree, edges) in net_cells.items():
    adj = defaultdict(set)
    for a, b in edges:
        adj[a].add(b)
        adj[b].add(a)
    pins_here = {c for c, _ in terminals[net]}
    seen_edges = set()
    runs = []
    for a in adj:
        for b in adj[a]:
            if (a, b) in seen_edges:
                continue
            # extend the run while the next cell is a straight degree-2 pass
            run = [a, b]
            seen_edges.update({(a, b), (b, a)})
            while True:
                last, prev = run[-1], run[-2]
                nxt = [c for c in adj[last] if c != prev]
                straight = (len(adj[last]) == 2 and last not in pins_here and nxt and
                            (nxt[0][0] - last[0], nxt[0][1] - last[1]) ==
                            (last[0] - prev[0], last[1] - prev[1]))
                if not straight:
                    break
                seen_edges.update({(last, nxt[0]), (nxt[0], last)})
                run.append(nxt[0])
            # and backward from a
            while True:
                first_, second = run[0], run[1]
                prv = [c for c in adj[first_] if c != second]
                straight = (len(adj[first_]) == 2 and first_ not in pins_here and prv and
                            (first_[0] - prv[0][0], first_[1] - prv[0][1]) ==
                            (second[0] - first_[0], second[1] - first_[1]))
                if not straight:
                    break
                seen_edges.update({(first_, prv[0]), (prv[0], first_)})
                run.insert(0, prv[0])
            runs.append((run[0], run[-1]))
    for a, b in runs:
        wire((a[0] * G, a[1] * G), (b[0] * G, b[1] * G))
    for c, nb in adj.items():
        if len(nb) + (1 if c in pins_here else 0) >= 3:
            items.append(["junction", ["at", mm(c[0] * G), mm(c[1] * G)], ["diameter", "0"],
                          ["color", "0", "0", "0", "0"], ["uuid", U()]])
    # one label on the longest horizontal run, clear above
    best = None
    for a, b in runs:
        if a[1] != b[1]:
            continue
        x0, x1 = sorted((a[0], b[0]))
        need = math.ceil(text_len(net, 1.0) / G) + 1
        if x1 - x0 < need:
            continue
        for s in range(x0 + (1 if x1 - x0 > need else 0), x1 - need + 1):
            clear = sum(1 for k in range(s, s + need)
                        if (k, a[1] - 1) not in grid.block and not grid.wire.get((k, a[1] - 1)))
            score = (clear, x1 - x0)
            if best is None or score > best[0]:
                best = (score, s, a[1])
    if best is None:  # vertical run, or the first wire end
        for a, b in runs:
            if a[0] == b[0]:
                y0 = min(a[1], b[1])
                best = ((0, 0), a[0], y0, 90)
                break
    if best is None:
        failed.append(f"label {net}")
        continue
    angle = best[3] if len(best) > 3 else 0
    lx, ly = best[1] * G + (0.6 if angle == 0 else 0), best[2] * G + (0.6 if angle == 90 else 0)
    if angle == 90:
        lx, ly = best[1] * G, (best[2] + 1) * G
    items.append(["label", Q(net), ["at", mm(lx), mm(ly), str(angle)],
                  ["effects", ["font", ["size", "1", "1"]], ["justify", "left", "bottom"]],
                  ["uuid", U()]])
    for k in range(math.ceil(text_len(net, 1.0) / G) + 1):
        grid.block.add((best[1] + k, best[2] - 1) if angle == 0 else (best[1] - 1, best[2] + k))

doc = ["kicad_sch", ["version", "20260306"], ["generator", Q("eeschema")],
       ["generator_version", Q("10.0")], ["uuid", Q(ROOT)], ["paper", Q(PAPER)],
       ["title_block", ["title", Q("LF-Bot base - PCB lam khung xe")], ["rev", Q("0.7")],
        ["comment", "1", Q("Sinh tu design.py bang gen_sch.py; mot nguon cho schematic va PCB")]],
       ["lib_symbols", *lib_syms.values()], *items,
       ["sheet_instances", ["path", Q("/"), ["page", Q("1")]]], ["embedded_fonts", "no"]]
open(OUT, "w").write(dump(doc) + "\n")


# ------------------------------------------------------------------ check
def check_netlist(path):
    """Every pin on the net design.py gives it, and no net joins two."""
    tmp = tempfile.mkdtemp()
    out = os.path.join(tmp, "n.net")
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadsexpr", "-o", out, path],
                   check=True, capture_output=True)
    got = {}
    for n in find(first(parse(open(out).read())[0], "nets"), "net"):
        name = str(first(n, "name")[1]).lstrip("/")
        for node in find(n, "node"):
            got[(str(first(node, "ref")[1]), str(first(node, "pin")[1]))] = name
    bad = []
    for ref, (lib_id, fp, value, nets) in PARTS.items():
        for num, *_ in parts[ref].pins:
            want = parts[ref].net(num)
            have = got.get((ref, num), "")
            if want is None:
                if not have.startswith("unconnected-"):
                    bad.append(f"{ref}.{num}: {have} (want open)")
            elif have != want:
                bad.append(f"{ref}.{num}: {have} (want {want})")
    return bad


bad = check_netlist(OUT)
print("wrote", OUT, "| wires", sum(1 for i in items if i[0] == "wire"),
      "| labels", sum(1 for i in items if i[0] == "label"),
      "| power symbols", sum(power_refs.values()))
if failed or bad:
    print("FAILED:", failed, bad[:20])
    sys.exit(1)
