"""Generate base.kicad_sch from design.py: symbols + net labels on every pin."""
import os
import sys
import uuid

from design import PWR_FLAGS, UUID_NS, parts_for, symbol_uuid
from sexpr import Q, dump, find, first, parse

OUT = sys.argv[1]
# KiCad keys symbol instances by project name, i.e. the file name.
PROJECT = os.path.splitext(os.path.basename(OUT))[0]
PARTS = parts_for(PROJECT)
LIBDIRS = {
    "PCM_Espressif": os.path.expanduser(
        "~/.local/share/kicad/10.0/3rdparty/symbols/"
        "com_github_espressif_kicad-libraries/Espressif.kicad_sym"),
}
_cache = {}


def lib_symbol(lib_id):
    lib, name = lib_id.split(":")
    path = LIBDIRS.get(lib, f"/usr/share/kicad/symbols/{lib}.kicad_sym")
    if path not in _cache:
        _cache[path] = parse(open(path).read())[0]
    for s in find(_cache[path], "symbol"):
        if s[1] == name:
            s = [x for x in s]
            s[1] = Q(lib_id)
            return s
    raise KeyError(lib_id)


def pins_of(sym):
    out = []

    def walk(n):
        for x in n:
            if isinstance(x, list):
                if x and x[0] == "pin":
                    at = first(x, "at")
                    num = first(x, "number")[1]
                    out.append((str(num), float(at[1]), float(at[2]), int(float(at[3]))))
                else:
                    walk(x)
    walk(sym)
    return out


_NS = UUID_NS
_cnt = [0]


def U():
    _cnt[0] += 1
    return Q(str(uuid.uuid5(_NS, f"item{_cnt[0]}")))


ROOT = str(uuid.uuid5(_NS, "root"))
items = []
lib_syms = {}
pin_map = {}  # ref -> {num: (x, y)} sheet coords, for checking


def font(size=1.27):
    return ["effects", ["font", ["size", str(size), str(size)]]]


def prop(name, value, x, y, hide=False):
    p = ["property", Q(name), Q(value), ["at", f"{x:.2f}", f"{y:.2f}", "0"]]
    if hide:
        p.append(["hide", "yes"])
    p.append(font())
    return p


def place(ref, lib_id, value, footprint, sx, sy, nets, in_bom=True):
    sym = lib_symbol(lib_id)
    lib_syms[lib_id] = sym
    pins = pins_of(sym)
    s = ["symbol", ["lib_id", Q(lib_id)], ["at", f"{sx:.2f}", f"{sy:.2f}", "0"],
         ["unit", "1"], ["body_style", "1"], ["exclude_from_sim", "no"],
         ["in_bom", "yes" if in_bom else "no"], ["on_board", "yes" if in_bom else "no"],
         ["in_pos_files", "yes"], ["dnp", "no"], ["uuid", Q(SYM_UUID[ref])],
         prop("Reference", ref, sx + 3, sy - 3),
         prop("Value", value, sx + 3, sy + 3),
         prop("Footprint", footprint, sx, sy, hide=True),
         prop("Datasheet", "~", sx, sy, hide=True),
         prop("Description", "", sx, sy, hide=True)]
    seen = set()
    for num, px, py, ang in pins:
        if num in seen:
            continue
        seen.add(num)
        s.append(["pin", Q(num), ["uuid", U()]])
    s.append(["instances", ["project", Q(PROJECT),
                            ["path", Q("/" + ROOT), ["reference", Q(ref)], ["unit", "1"]]]])
    items.append(s)
    pin_map[ref] = {}
    for num, px, py, ang in pins:
        x, y = sx + px, sy - py
        pin_map[ref][num] = (x, y)
        net = nets.get(int(num)) if num.isdigit() else nets.get(num)
        if net is None:
            items.append(["no_connect", ["at", f"{x:.2f}", f"{y:.2f}"], ["uuid", U()]])
            continue
        la = {0: 180, 180: 0, 90: 270, 270: 90}[ang]
        just = "right" if la == 180 else "left"
        if la in (90, 270):
            just = "left"
        items.append(["label", Q(net), ["at", f"{x:.2f}", f"{y:.2f}", str(la)],
                      ["effects", ["font", ["size", "1", "1"]], ["justify", just, "bottom"]],
                      ["uuid", U()]])


SYM_UUID = {ref: symbol_uuid(ref) for ref in PARTS}
for i, f in enumerate(PWR_FLAGS):
    SYM_UUID[f"#FLG0{i + 1}"] = str(uuid.uuid5(_NS, f"flg{i}"))

# simple column layout on A3
order = list(PARTS)
slots = {"U1": (80, 110), "U2": (190, 50), "U3": (190, 90), "U4": (190, 130)}
_k = 0
for ref in order:
    if ref not in slots:
        slots[ref] = (240 + 40 * (_k % 4), 30 + 28 * (_k // 4))
        _k += 1
for ref in order:
    lib_id, fp, value, nets = PARTS[ref]
    x, y = slots[ref]
    place(ref, lib_id, value, fp, round(x / 2.54) * 2.54, round(y / 2.54) * 2.54, nets)

for i, net in enumerate(PWR_FLAGS):
    ref = f"#FLG0{i + 1}"
    place(ref, "power:PWR_FLAG", "PWR_FLAG", "", round((40 + 25 * i) / 2.54) * 2.54,
          round(260 / 2.54) * 2.54, {1: net}, in_bom=False)

doc = ["kicad_sch", ["version", "20260306"], ["generator", Q("eeschema")],
       ["generator_version", Q("10.0")], ["uuid", Q(ROOT)], ["paper", Q("A2")],
       ["title_block", ["title", Q("LF-Bot base - PCB lam khung xe")], ["rev", Q("0.1")],
        ["comment", "1", Q("Sinh tu design.py; mot nguon cho schematic va PCB")]],
       ["lib_symbols", *lib_syms.values()], *items,
       ["sheet_instances", ["path", Q("/"), ["page", Q("1")]]], ["embedded_fonts", "no"]]
open(OUT, "w").write(dump(doc) + "\n")
print("wrote", OUT, len(items), "items")
