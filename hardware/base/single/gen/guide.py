"""Lay the hand-planned bottom tracks before the router runs.

The router places one net at a time on the shortest path it sees and does
not look for the layout with the fewest crossings. The nets that otherwise
cut across the eight IR divider lines are laid here and locked, so the
router builds around them:

- The I2C/ToF bus. With SDA on U1.8, SCL on U1.9 and XSHUT on U1.10 (the
  single-sided pin order, design.VARIANT_PINS) the three lines dive under
  the ESP32 module side by side, run east under the IMU body, turn north and
  pass straight through the IMU's SDA and SCL pins (J6.4, J6.3) and the
  expansion header's (J10.4, J10.5), XSHUT in the gap beside them, then turn
  west along the front edge and reach the ToF (J5) in its own pin order.
  None of them crosses another track.
- IR_CTRL, through the only channel between the divider columns that no ADC
  line needs (x = 1.27 mm, between IR4 and IR5, and between J3 pins 6 and
  7), to J4 on the front strip.

Each lane keeps 0.25 mm or more from every pad; lanes are 0.8 mm apart.
Under the IMU the lanes run deep enough (XSHUT just clear of the caster
screw keepout) to leave one track's room behind the IMU pins, so IMU_INT can
reach J6.8 from behind while +3V3 reaches J6.9 from the front.

usage: guide.py BOARD.kicad_pcb
"""
import sys

import pcbnew

ROW = -59.69                     # ESP32 header row toward the IR bar
LANE = (-54.25, -53.45, -52.65)  # under the IMU body: SDA, SCL, XSHUT
UP = (26.42, 28.96, 30.23)       # north legs: J6.4/J10.4, J6.3/J10.5, gap
FRONT = (-89.8, -90.6, -91.4)    # along the front edge, clear of the IR bar screw
TOF = (-1.27, 1.27, 6.35)        # J5.3, J5.4, J5.6
PIN = (6.1, 3.56, 1.02)          # U1.8, U1.9, U1.10
TOF_ROW = -92.964
C = 0.6                          # 45 degree corner


def bus_line(i):
    x0, y0, xu, yf, xt = PIN[i], LANE[i], UP[i], FRONT[i], TOF[i]
    return [(x0, ROW), (x0, y0 - C), (x0 + C, y0), (xu - C, y0), (xu, y0 - C),
            (xu, yf + C), (xu - C, yf), (xt + C, yf), (xt, yf - C), (xt, TOF_ROW)]


GUIDES = {
    "/I2C_SDA": [bus_line(0)],
    "/I2C_SCL": [bus_line(1)],
    "/TOF_XSHUT": [bus_line(2)],
    "/IR_CTRL": [[(-1.52, ROW), (-1.52, -60.40), (1.27, -63.19), (1.27, -85.56),
                  (0.67, -86.16), (-8.89, -86.16)]],                     # U1.11 -> J4.4
}
WIDTH = 0.3


def main(path):
    board = pcbnew.LoadBoard(path)
    nets = board.GetNetsByName()
    laid = 0
    for name, lines in GUIDES.items():
        for line in lines:
            for (x0, y0), (x1, y1) in zip(line, line[1:]):
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(x0), pcbnew.FromMM(y0)))
                t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(x1), pcbnew.FromMM(y1)))
                t.SetWidth(pcbnew.FromMM(WIDTH))
                t.SetLayer(pcbnew.B_Cu)
                t.SetNet(nets[name])
                t.SetLocked(True)
                board.Add(t)
                laid += 1
    pcbnew.SaveBoard(path, board)
    print("guide segments laid:", laid)


if __name__ == "__main__":
    main(sys.argv[1])
