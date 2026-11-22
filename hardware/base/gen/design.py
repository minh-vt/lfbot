"""Source for the LF-Bot base schematic: parts, footprints and pin->net.

Layout (positions, routing) lives in base.kicad_pcb; gen_sch.py writes the
schematic from this file and the board follows it through KiCad's
schematic-parity check.
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


# ref: (lib_id, footprint, value, pins{num: net|None}). Positions live in
# base.kicad_pcb, which is the source for layout.
PARTS = {
    "U1": ("PCM_Espressif:ESP32-S3-DevKitC", "Base:ESP32-S3-DevKitC_TDM2533",
           "ESP32-S3 N16R8 TDM2533", DEVKIT_NETS),
    "U2": ("Connector_Generic:Conn_01x08", "Base:DRV8871_Module",
           "DRV8871 module L", drv("L")),
    "U3": ("Connector_Generic:Conn_01x08", "Base:DRV8871_Module",
           "DRV8871 module R", drv("R")),
    "U4": ("Connector_Generic:Conn_01x04", "Base:MP1584_Wired",
           "MP1584 5V (wired)", {1: 'VBAT_SW', 2: 'GND', 3: '+5V_BUCK', 4: 'GND'}),
    "J1": ("Connector_Generic:Conn_01x02", "Connector_Wire:SolderWire-1sqmm_1x02_P5.4mm_D1.4mm_OD2.7mm",
           "BAT 3S", {1: 'VBAT', 2: 'GND'}),
    "F1": ("Device:Polyfuse", "Fuse:Fuse_Bourns_MF-RG500",
           "PTC (chon khi lap)", {1: 'VBAT', 2: 'VBAT_F'}),
    "J2": ("Connector_Generic:Conn_01x02", "Connector_Wire:SolderWire-0.5sqmm_1x02_P4.8mm_D0.9mm_OD2.3mm",
           "SW nguon", {1: 'VBAT_F', 2: 'VBAT_SW'}),
    "C1": ("Device:C_Polarized", "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm",
           "220uF 25V", {1: 'VBAT_SW', 2: 'GND'}),
    "D1": ("Device:D_Schottky", "Diode_SMD:D_SMA_Handsoldering",
           "SS14", {1: '+5V', 2: '+5V_BUCK'}),
    "R1": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k 1%", {1: 'VBAT_SW', 2: 'VBAT_SENSE'}),
    "R2": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k 1%", {1: 'VBAT_SENSE', 2: 'GND'}),
    "C2": ("Device:C", "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder",
           "100nF", {1: 'VBAT_SENSE', 2: 'GND'}),
    "J3": ("Connector_Generic:Conn_01x11", "Connector_PinHeader_2.54mm:PinHeader_1x11_P2.54mm_Vertical",
           "IR bar", {1: 'GND', 2: 'IR_EN', 3: 'IR_D1', 4: 'IR_D2', 5: 'IR_D3', 6: 'IR_D4', 7: 'IR_D5', 8: 'IR_D6', 9: 'IR_D7', 10: 'IR_D8', 11: '+3V3'}),
    "R10": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D1', 2: 'ADC_IR1'}),
    "R20": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR1', 2: 'GND'}),
    "R11": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D2', 2: 'ADC_IR2'}),
    "R21": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR2', 2: 'GND'}),
    "R12": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D3', 2: 'ADC_IR3'}),
    "R22": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR3', 2: 'GND'}),
    "R13": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D4', 2: 'ADC_IR4'}),
    "R23": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR4', 2: 'GND'}),
    "R14": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D5', 2: 'ADC_IR5'}),
    "R24": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR5', 2: 'GND'}),
    "R15": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D6', 2: 'ADC_IR6'}),
    "R25": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR6', 2: 'GND'}),
    "R16": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D7', 2: 'ADC_IR7'}),
    "R26": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR7', 2: 'GND'}),
    "R17": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "10k", {1: 'IR_D8', 2: 'ADC_IR8'}),
    "R27": ("Device:R", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",
           "47k", {1: 'ADC_IR8', 2: 'GND'}),
    "J4": ("Connector_Generic:Conn_02x03_Odd_Even", "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical",
           "IR_EN chon", {1: 'IR_EN', 2: '+3V3', 3: 'IR_EN', 4: 'IR_CTRL', 5: 'IR_EN', 6: 'GND'}),
    "J5": ("Connector_Generic:Conn_01x06", "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical",
           "ToF400C", {1: '+3V3', 2: 'GND', 3: 'I2C_SDA', 4: 'I2C_SCL', 5: None, 6: 'TOF_XSHUT'}),
    "J6": ("Connector_Generic:Conn_01x10", "Base:GY9250_Flat",
           "GY-9250 (han thang, nam phang)", {1: '+3V3', 2: 'GND', 3: 'I2C_SCL', 4: 'I2C_SDA', 5: None, 6: None, 7: 'GND', 8: 'IMU_INT', 9: '+3V3', 10: 'GND'}),
    "J7": ("Connector_Generic:Conn_01x06",
           "Connector_JST:JST_ZH_B6B-ZR_1x06_P1.50mm_Vertical", "ENC L",
           enc("L")),
    "J8": ("Connector_Generic:Conn_01x06",
           "Connector_JST:JST_ZH_B6B-ZR_1x06_P1.50mm_Vertical", "ENC R",
           enc("R")),
    "SW1": ("Switch:SW_Push", "Button_Switch_THT:SW_PUSH_6mm",
           "START", {1: 'BTN_START', 2: 'GND'}),
    "SW2": ("Switch:SW_Push", "Button_Switch_THT:SW_PUSH_6mm",
           "NUT 2", {1: 'BTN_2', 2: 'GND'}),
    "J9": ("Connector_Generic:Conn_01x03", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
           "nut ngoai", {1: 'BTN_START', 2: 'BTN_2', 3: 'GND'}),
    "J10": ("Connector_Generic:Conn_01x05", "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical",
           "I2C mo rong", {1: '+5V', 2: '+3V3', 3: 'GND', 4: 'I2C_SDA', 5: 'I2C_SCL'}),
}


PWR_FLAGS = ["GND", "+3V3", "+5V"]

# Pins a board variant changes, keyed by project name (the schematic file
# name). The single-sided board swaps SDA and XSHUT on the ESP32 (U1.8 is
# GPIO15, U1.10 GPIO17) so the three bus lines leave the header in the order
# the ToF takes them and can run side by side to it without crossing; its
# firmware needs SDA = GPIO15 and XSHUT = GPIO17. It also leaves +5V off the
# expansion header J10, a branch that ran around most of the board.
VARIANT_PINS = {
    "single": {"U1": {8: "I2C_SDA", 10: "TOF_XSHUT"}, "J10": {1: None}},
}


def parts_for(project):
    """PARTS with the variant's pin changes applied."""
    parts = dict(PARTS)
    for ref, pins in VARIANT_PINS.get(project, {}).items():
        lib_id, fp, value, nets = parts[ref]
        parts[ref] = (lib_id, fp, value, {**nets, **pins})
    return parts

