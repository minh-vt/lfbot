"""Write custom footprints into Base.pretty."""
import sys
import uuid

OUT = sys.argv[1]


def U():
    return f'(uuid "{uuid.uuid4()}")'


def fp(name, descr, body, attr="through_hole"):
    return (f'(footprint "{name}"\n\t(version 20260206)\n\t(generator "pcbnew")\n'
            f'\t(generator_version "10.0")\n\t(layer "F.Cu")\n\t(descr "{descr}")\n'
            f'\t(property "Reference" "REF**" (at 0 -3 0) (layer "F.SilkS") {U()} '
            f'(effects (font (size 1 1) (thickness 0.15))))\n'
            f'\t(property "Value" "{name}" (at 0 3 0) (layer "F.Fab") {U()} '
            f'(effects (font (size 1 1) (thickness 0.15))))\n'
            f'\t(attr {attr})\n{body})\n')


def rect(x1, y1, x2, y2, layer, w=0.05):
    return (f'\t(fp_rect (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) '
            f'(stroke (width {w}) (type solid)) (fill no) (layer "{layer}") {U()})\n')


def circle(x, y, r, layer, w=0.05):
    return (f'\t(fp_circle (center {x:.3f} {y:.3f}) (end {x + r:.3f} {y:.3f}) '
            f'(stroke (width {w}) (type solid)) (fill no) (layer "{layer}") {U()})\n')


def line(x1, y1, x2, y2, layer, w=0.1):
    return (f'\t(fp_line (start {x1:.3f} {y1:.3f}) (end {x2:.3f} {y2:.3f}) '
            f'(stroke (width {w}) (type solid)) (layer "{layer}") {U()})\n')


def text(s, x, y, layer, size=0.8):
    return (f'\t(fp_text user "{s}" (at {x:.3f} {y:.3f} 0) (layer "{layer}") {U()} '
            f'(effects (font (size {size} {size}) (thickness {max(0.15, size * 0.15):.3f}))))\n')


def pad(num, x, y, size, drill, shape="circle"):
    return (f'\t(pad "{num}" thru_hole {shape} (at {x:.3f} {y:.3f}) (size {size} {size}) '
            f'(drill {drill}) (layers "*.Cu" "*.Mask") {U()})\n')


def npth(x, y, d):
    return (f'\t(pad "" np_thru_hole circle (at {x:.3f} {y:.3f}) (size {d} {d}) '
            f'(drill {d}) (layers "*.Cu" "*.Mask") {U()})\n')


def slot(x, y, d, travel):
    L = d + 2 * travel
    return (f'\t(pad "" np_thru_hole oval (at {x:.3f} {y:.3f} 90) (size {L} {d}) '
            f'(drill oval {L} {d}) (layers "*.Cu" "*.Mask") {U()})\n')


files = {}

# --- ESP32-S3 TDM2533 devkit: Espressif DevKitC pin order, rows 25.4 mm apart
# (owner measured). Pin 1 = 3V3 at antenna end. Body outline estimated from photo.
b = ""
for i in range(22):
    b += pad(i + 1, 0, i * 2.54, 1.7, 1.0, "rect" if i == 0 else "circle")
for i in range(22):
    b += pad(23 + i, 25.4, (21 - i) * 2.54, 1.7, 1.0)
# Courtyard = the two socket strips only: the board sits ~11 mm up on female
# headers, so low parts (<9 mm) may go underneath. Body outline is on F.Fab.
b += rect(-1.35, -1.35, 1.35, 54.69, "F.CrtYd")
b += rect(24.05, -1.35, 26.75, 54.69, "F.CrtYd")
b += rect(-1.35, -9.0, 26.75, 57.0, "F.Fab", 0.1)
b += line(-1.35, -1.5, 26.75, -1.5, "F.Fab")
b += text("ANTEN", 12.7, -5, "F.Fab")
b += text("USB-C x2", 12.7, 55, "F.Fab")
b += text("5Vin", 2.9, 50.8, "F.SilkS", 0.8)
b += rect(-1.27, -1.27, 1.27, 1.27, "F.SilkS", 0.15)
files["ESP32-S3-DevKitC_TDM2533"] = fp(
    "ESP32-S3-DevKitC_TDM2533",
    "TDM2533 ESP32-S3 N16R8 dual USB-C devkit; DevKitC-1 pin order; rows 25.4 mm (owner "
    "measured); female headers 1x22 2.54 mm. Board outline estimated, not measured.", b)

# --- DRV8871 module: 4-pin header IN2 IN1 VM GND (silk on module) carries the
# logic only; VM on the header is left open. Wire pads along the screw-terminal
# edge: M1/M2 from the MOTOR terminal, V+/V- into the POWER terminal (owner's
# photo: MOTOR then POWER +, -), so motor current never crosses the board.
# Body outline estimated from shop photo.
b = ""
for i, n in enumerate(["IN2", "IN1", "VM", "GND"]):
    b += pad(i + 1, i * 2.54, 0, 1.7, 1.0, "rect" if i == 0 else "circle")
    b += text(n, i * 2.54, -2.0, "F.SilkS", 0.8)
for num, x, n in ((5, -3.0, "M1"), (6, 0.81, "M2"), (7, 4.62, "V+"), (8, 8.43, "V-")):
    b += pad(num, x, -22.0, 2.2, 1.2, "rect" if num in (5, 7) else "circle")
    b += text(n, x, -24.3, "F.SilkS", 0.8)
b += rect(-6.7, -21.0, 14.3, 0.0, "F.Fab", 0.1)
# Module rides ~9 mm up on a female header: courtyard covers only what touches
# the board (header + wire pads); body outline stays on F.Fab.
b += rect(-1.4, -1.4, 9.0, 1.4, "F.CrtYd")
b += rect(-4.5, -23.6, 9.9, -20.4, "F.CrtYd")
b += text("DRV8871 21x21 (uoc luong)", 3.8, -10, "F.Fab", 0.7)
b += text("MOTOR      POWER", 4.5, -19.5, "F.Fab", 0.7)
files["DRV8871_Module"] = fp(
    "DRV8871_Module",
    "DRV8871 breakout: header IN2/IN1/VM/GND 2.54 mm carries logic (VM open); pads "
    "5/6 = MOTOR terminal, 7/8 = POWER terminal, short wires. Outline estimated.", b)

# --- MP1584 mini module (17 x 22 mm) soldered flat on short legs (cut
# resistor leads) through its corner pad pairs. Pad positions measured from the
# owner's photo against a ruler (+-0.3 mm); 1.3 mm drills let the legs absorb
# that. Trimmer sits at the OUT+ corner: set 5 V with the devkit unplugged.
b = ""
corner = {1: (1.8, 1.55, "IN+"), 2: (12.7, 1.55, "IN-"), 3: (1.8, 20.45, "OUT+"),
          4: (12.7, 20.45, "OUT-")}
for num, (x, y, n) in corner.items():
    for dx in (0.0, 2.5):
        b += pad(num, x + dx, y, 2.2, 1.3, "rect" if num == 1 and dx == 0 else "circle")
    b += text(n, x + 1.25, y + (2.3 if y < 10 else -2.3), "F.SilkS", 0.8)
b += rect(0, 0, 17, 22, "F.Fab", 0.1)
b += rect(-0.25, -0.25, 17.25, 22.25, "F.CrtYd")
b += text("MP1584 mini", 8.5, 11, "F.Fab", 0.8)
files["MP1584_Wired"] = fp(
    "MP1584_Wired",
    "MP1584 mini buck 17x22 mm on short legs through its corner pads; "
    "1 IN+ 2 IN- 3 OUT+ 4 OUT-. Pad positions from photo, oversize drills.", b)

# --- GY-9250 lying flat, its straight header soldered through the board (no
# socket) so the module stays ~5 mm high and fits under the devkit. Gyro Z is
# then vertical. Pin order from the module silk: VCC GND SCL SDA EDA ECL ADO INT
# NCS FSYNC. Body 25.5 x 15.5 from photo.
b = ""
for i, n in enumerate(["VCC", "GND", "SCL", "SDA", "EDA", "ECL", "ADO", "INT", "NCS", "FSY"]):
    b += pad(i + 1, i * 2.54, 0, 1.7, 1.0, "rect" if i == 0 else "circle")
b += rect(-1.3, -15.5, 24.2, 1.3, "F.Fab", 0.1)
b += text("GY-9250 (Z thang dung)", 11.4, -8, "F.Fab", 0.8)
b += rect(-1.5, -15.7, 24.4, 1.5, "F.CrtYd")
b += text("VCC", 0, 2.3, "F.SilkS", 0.8)
files["GY9250_Flat"] = fp(
    "GY9250_Flat",
    "GY-9250 (MPU-6500) flat, header soldered through the board; pin 1 VCC.", b)

# --- Stack mount: base.dxf M3 pattern 50 x 35 for the printed battery layer.
b = ""
for x in (-25, 25):
    for y in (-17.5, 17.5):
        b += npth(x, y, 3.2)
        b += circle(x, y, 3.3, "F.CrtYd")
        b += circle(x, y, 3.0, "F.SilkS", 0.15)
b += rect(-30, -22.5, 30, 22.5, "Dwgs.User", 0.1)
b += text("tang pin in 3D (base.dxf 60x45)", 0, 0, "Dwgs.User", 0.8)
files["Stack_Mount_50x35"] = fp(
    "Stack_Mount_50x35", "Four M3 holes, 50 x 35 mm, same as hardware/base/mechanical/base.dxf.",
    b, attr="board_only exclude_from_pos_files exclude_from_bom")

# --- IR bar mount: origin = eye line centre. Two M3 slots 37 mm apart, +-3 mm
# fore/aft so the eye-to-axle distance stays tunable. Bar outline from photo.
b = ""
for x in (-18.5, 18.5):
    b += slot(x, 7.3, 3.2, 3.0)
    b += rect(x - 3.0, 7.3 - 6.1, x + 3.0, 7.3 + 6.1, "F.Fab", 0.1)
    b += rect(x - 1.85, 7.3 - 4.85, x + 1.85, 7.3 + 4.85, "F.CrtYd")  # the slot itself
b += rect(-33, -3.6, 33, 13.4, "F.Fab", 0.1)
b += line(-33, 0, 33, 0, "F.Fab", 0.15)
b += text("mat IR (8 mat ~8 mm)", 0, -1.2, "F.Fab", 0.7)
files["IR_Bar_Mount_37mm"] = fp(
    "IR_Bar_Mount_37mm",
    "8-ch IR bar under the board on M3 standoffs; holes 37 mm (owner), slots +-3 mm. "
    "Origin on the eye line. Bar 66x17 and hole offset estimated from photo.",
    b, attr="board_only exclude_from_pos_files exclude_from_bom")

# --- Caster mount: two M2 slots 15 mm apart (owner), +-3 mm; keep THT leads out.
b = ""
for x in (-7.5, 7.5):
    b += slot(x, 0, 2.2, 3.0)
b += circle(0, 0, 12.0, "F.CrtYd")  # becomes B.CrtYd when flipped
b += circle(0, 0, 11.5, "F.Fab", 0.1)
b += text("bi cau: khong chan THT", 0, -9, "F.Fab", 0.8)
files["Caster_Mount_15mm"] = fp(
    "Caster_Mount_15mm",
    "Ball caster under the board, M2 holes 15 mm (owner), slots +-3 mm. Body diameter "
    "assumed 23 mm keep-out for through-hole leads.",
    b, attr="board_only exclude_from_pos_files exclude_from_bom")

# --- N20 bracket (Bracket.step): screw pitch 18 mm, origin on the motor axis at
# the bracket centre; motor runs toward -X (robot centre), wheel on +X.
# Envelope: gearbox+can+encoder ~33 mm (datasheet body 24 mm + encoder board),
# gearbox face ~12.3 mm outboard of the bracket centre so the wheel lands on the
# measured 95.2 mm track. The whole +-7.5 mm band is kept free of parts, so a
# longer encoder only eats into empty board.
b = ""
for y in (-9, 9):
    b += npth(0, y, 2.2)
b += rect(-5.75, -12.5, 5.75, 12.5, "F.Fab", 0.1)
pts = [(-20.5, -7.5), (-5.75, -7.5), (-5.75, -12.75), (5.75, -12.75), (5.75, -7.5),
       (12.5, -7.5), (12.5, 7.5), (5.75, 7.5), (5.75, 12.75), (-5.75, 12.75),
       (-5.75, 7.5), (-20.5, 7.5)]
b += ("\t(fp_poly (pts " + " ".join(f"(xy {x} {y})" for x, y in pts) +
      f') (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd") {U()})\n')
b += rect(-20.5, -6.0, 12.3, 6.0, "F.Fab", 0.1)
b += line(-20.5, 0, 32, 0, "F.Fab", 0.1)
b += rect(13.85, -21.5, 31.85, 21.5, "F.Fab", 0.1)
b += text("banh 43", 22.85, 0, "F.Fab", 0.8)
b += text("encoder", -16, 0, "F.Fab", 0.8)
files["N20_Bracket_Mount"] = fp(
    "N20_Bracket_Mount",
    "Printed N20 bracket from hardware/base/mechanical/Bracket.step: two screws 18 mm "
    "apart, motor on top. Fab shows motor+encoder envelope and 43 mm wheel (track 95.2).",
    b, attr="board_only exclude_from_pos_files exclude_from_bom")

for name, txt in files.items():
    open(f"{OUT}/{name}.kicad_mod", "w").write(txt)
print("wrote", list(files))
