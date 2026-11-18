"""Single source for the LF-Bot base board: parts, pin->net, placement.

PCB coordinates: mm, origin = axle centre, -Y = forward (robot front at top
of the KiCad view), +X = robot right. Side "F" = top (motors, modules),
"B" = bottom (caster, IR bar).
"""

import uuid

# Fixed namespace: symbol UUIDs (and so schematic<->PCB links) are reproducible.
UUID_NS = uuid.UUID("6f1c2a52-6a0e-4c5b-9a43-0b1e5a7d2c11")


def symbol_uuid(ref):
    return str(uuid.uuid5(UUID_NS, "sym" + ref))


DEVKIT_NETS = {
    # front row (pins 1-22, toward the IR bar): all eight IR channels in the
    # same left-to-right order as the bar, I2C/ToF/IR_CTRL in the middle gap.
    # SDA, SCL, XSHUT sit in the ToF header's order so the three run side by
    # side through the IR header's pin gaps without crossing
    1: "+3V3", 2: "+3V3", 3: None, 4: "ADC_IR8", 5: "ADC_IR7", 6: "ADC_IR6",
    7: "ADC_IR5", 8: "TOF_XSHUT", 9: "I2C_SCL", 10: "I2C_SDA", 11: "IR_CTRL",
    12: "ADC_IR4", 13: "ADC_IR3", 14: None, 15: "ADC_IR2", 16: "ADC_IR1",
    17: "BTN_START", 18: "BTN_2", 19: "IMU_INT", 20: None, 21: "+5V", 22: "GND",
    # rear row (pins 23-44, toward drivers and motors): motor PWM, encoders,
    # battery sense (GPIO1, ADC1)
    23: "GND", 24: "GND", 25: None, 26: None, 27: "ENC_L_A", 28: "ENC_L_B",
    29: None, 30: None, 31: None, 32: None, 33: None, 34: None,
    35: "MOT_L_IN1", 36: "MOT_L_IN2", 37: "ENC_R_A", 38: "MOT_R_IN1",
    39: "MOT_R_IN2", 40: "ENC_R_B", 41: "VBAT_SENSE", 42: None, 43: None,
    44: "GND",
}


def drv(side):
    """DRV8871 module: header carries logic only (VM pin left open); motor
    power enters at the module's POWER screw terminal, next to MOTOR, so the
    motor current loop stays at the rear of the board."""
    s = side
    return {1: f"MOT_{s}_IN2", 2: f"MOT_{s}_IN1", 3: None, 4: "GND",
            5: f"MOT_{s}_A", 6: f"MOT_{s}_B", 7: "VBAT_SW", 8: "GND"}


def enc(side):
    s = side
    return {1: f"MOT_{s}_A", 2: "GND", 3: f"ENC_{s}_A", 4: f"ENC_{s}_B",
            5: "+3V3", 6: f"MOT_{s}_B"}


# ref: (lib_id, footprint, value, pins{num: net|None}, (cx, cy, rot, side))
# PARTS position = courtyard centre after rotation; MECH position = footprint origin.
PARTS = {
    "U1": ("PCM_Espressif:ESP32-S3-DevKitC", "Base:ESP32-S3-DevKitC_TDM2533",
           "ESP32-S3 N16R8 TDM2533", DEVKIT_NETS, (-2.67, -47.00, 270, "F")),
    "U2": ("Connector_Generic:Conn_01x08", "Base:DRV8871_Module",
           "DRV8871 module L", drv("L"), (-9.40, -20.25, 180, "F")),
    "U3": ("Connector_Generic:Conn_01x08", "Base:DRV8871_Module",
           "DRV8871 module R", drv("R"), (12.00, -20.25, 180, "F")),
    "U4": ("Connector_Generic:Conn_01x04", "Base:MP1584_Wired",
           "MP1584 5V (wired)", {1: 'VBAT_SW', 2: 'GND', 3: '+5V_BUCK', 4: 'GND'},
           (-25.00, -47.00, 0, "F")),
    "J1": ("Connector_Generic:Conn_01x02", "Connector_Wire:SolderWire-1sqmm_1x02_P5.4mm_D1.4mm_OD2.7mm",
           "BAT 3S", {1: 'VBAT', 2: 'GND'},
           (-12.00, 17.50, 0, "F")),
    "F1": ("Device:Polyfuse", "Fuse:Fuse_Bourns_MF-RG500",
           "PTC (chon khi lap)", {1: 'VBAT', 2: 'VBAT_F'},
           (0.00, 11.50, 0, "F")),
    "J2": ("Connector_Generic:Conn_01x02", "Connector_Wire:SolderWire-0.5sqmm_1x02_P4.8mm_D0.9mm_OD2.3mm",
           "SW nguon", {1: 'VBAT_F', 2: 'VBAT_SW'},
           (12.00, 17.50, 0, "F")),
    "C1": ("Device:C_Polarized", "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm",
           "220uF 25V", {1: 'VBAT_SW', 2: 'GND'},
           (0.00, 17.00, 0, "F")),
    "D1": ("Device:D_Schottky", "Diode_SMD:D_SMA_Handsoldering",
           "SS14", {1: '+5V', 2: '+5V_BUCK'},
           (-12.50, -47.00, 90, "F")),
    "R1": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k 1%", {1: 'VBAT_SW', 2: 'VBAT_SENSE'},
           (14.00, -38.50, 0, "F")),
    "R2": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k 1%", {1: 'VBAT_SENSE', 2: 'GND'},
           (18.50, -38.50, 0, "F")),
    "C2": ("Device:C", "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder",
           "100nF", {1: 'VBAT_SENSE', 2: 'GND'},
           (23.00, -38.50, 0, "F")),
    "J3": ("Connector_Generic:Conn_01x11", "Connector_PinHeader_2.54mm:PinHeader_1x11_P2.54mm_Vertical",
           "IR bar", {1: 'GND', 2: 'IR_EN', 3: 'IR_D1', 4: 'IR_D2', 5: 'IR_D3', 6: 'IR_D4', 7: 'IR_D5', 8: 'IR_D6', 9: 'IR_D7', 10: 'IR_D8', 11: '+3V3'},
           (0.00, -78.30, 90, "F")),
    "R10": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D1', 2: 'ADC_IR1'},
           (-19.00, -70.40, 90, "F")),
    "R20": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR1', 2: 'GND'},
           (-19.00, -66.10, 90, "F")),
    "R11": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D2', 2: 'ADC_IR2'},
           (-14.50, -70.40, 90, "F")),
    "R21": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR2', 2: 'GND'},
           (-14.50, -66.10, 90, "F")),
    "R12": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D3', 2: 'ADC_IR3'},
           (-10.00, -70.40, 90, "F")),
    "R22": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR3', 2: 'GND'},
           (-10.00, -66.10, 90, "F")),
    "R13": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D4', 2: 'ADC_IR4'},
           (-5.50, -70.40, 90, "F")),
    "R23": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR4', 2: 'GND'},
           (-5.50, -66.10, 90, "F")),
    "R14": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D5', 2: 'ADC_IR5'},
           (7.50, -70.40, 90, "F")),
    "R24": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR5', 2: 'GND'},
           (7.50, -66.10, 90, "F")),
    "R15": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D6', 2: 'ADC_IR6'},
           (12.00, -70.40, 90, "F")),
    "R25": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR6', 2: 'GND'},
           (12.00, -66.10, 90, "F")),
    "R16": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D7', 2: 'ADC_IR7'},
           (16.50, -70.40, 90, "F")),
    "R26": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR7', 2: 'GND'},
           (16.50, -66.10, 90, "F")),
    "R17": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D8', 2: 'ADC_IR8'},
           (21.00, -70.40, 90, "F")),
    "R27": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR8', 2: 'GND'},
           (21.00, -66.10, 90, "F")),
    "J4": ("Connector_Generic:Conn_02x03_Odd_Even", "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical",
           "IR_EN chon", {1: 'IR_EN', 2: '+3V3', 3: 'IR_EN', 4: 'IR_CTRL', 5: 'IR_EN', 6: 'GND'},
           (-10.50, -85.50, 90, "F")),
    "J5": ("Connector_Generic:Conn_01x06", "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical",
           "ToF400C", {1: '+3V3', 2: 'GND', 3: 'I2C_SDA', 4: 'I2C_SCL', 5: None, 6: 'TOF_XSHUT'},
           (0.00, -92.90, 90, "F")),
    "J6": ("Connector_Generic:Conn_01x10", "Base:GY9250_Flat",
           "GY-9250 (han thang, nam phang)", {1: '+3V3', 2: 'GND', 3: 'I2C_SCL', 4: 'I2C_SDA', 5: None, 6: None, 7: 'GND', 8: 'IMU_INT', 9: '+3V3', 10: 'GND'},
           (22.6, -49.4, 180, "F")),
    "J7": ("Connector_Generic:Conn_01x06",
           "Connector_JST:JST_ZH_B6B-ZR_1x06_P1.50mm_Vertical", "ENC L",
           enc("L"), (-12.60, 11.00, 0, "F")),
    "J8": ("Connector_Generic:Conn_01x06",
           "Connector_JST:JST_ZH_B6B-ZR_1x06_P1.50mm_Vertical", "ENC R",
           enc("R"), (12.60, 11.00, 0, "F")),
    "SW1": ("Switch:SW_Push", "Button_Switch_THT:SW_PUSH_6mm",
           "START", {1: 'BTN_START', 2: 'GND'},
           (-29.50, -89.30, 0, "F")),
    "SW2": ("Switch:SW_Push", "Button_Switch_THT:SW_PUSH_6mm",
           "NUT 2", {1: 'BTN_2', 2: 'GND'},
           (-29.50, -79.30, 0, "F")),
    "J9": ("Connector_Generic:Conn_01x03", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
           "nut ngoai", {1: 'BTN_START', 2: 'BTN_2', 3: 'GND'},
           (-30.00, -70.00, 90, "F")),
    "J10": ("Connector_Generic:Conn_01x05", "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical",
           "I2C mo rong", {1: '+5V', 2: '+3V3', 3: 'GND', 4: 'I2C_SDA', 5: 'I2C_SCL'},
           (28.8, -76.0, 90, "F")),
}

# Footprints with no schematic symbol (mechanical only).
MECH = {
    "H1": ("Base:Stack_Mount_50x35", (0.0, -1.2, 0, "F")),
    "H2": ("Base:IR_Bar_Mount_37mm", (0.0, -90.3, 0, "B")),
    "H3": ("Base:Caster_Mount_15mm", (0.0, -47.00, 0, "B")),
    "H4": ("Base:N20_Bracket_Mount", (-24.75, 0.0, 180, "F")),
    "H5": ("Base:N20_Bracket_Mount", (24.75, 0.0, 0, "F")),
}

PWR_FLAGS = ["GND", "+3V3", "+5V"]

POWER_NETS = {"VBAT", "VBAT_F", "VBAT_SW", "GND", "MOT_L_A", "MOT_L_B",
              "MOT_R_A", "MOT_R_B"}
SUPPLY_NETS = {"+5V", "+5V_BUCK", "+3V3"}
