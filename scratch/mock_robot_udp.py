#!/usr/bin/env python3
"""
Mock Robot Telemetry UDP Broadcaster
------------------------------------
Gia lap robot ESP32-S3 that dang phat du lieu telemetry ve Ground Station tren cong UDP 9876.
Dung de kiem thu tinh nang Ground Station, Live Trail Rendering va Camera Auto-Scale trong Simulation.
"""

import math
import socket
import struct
import time
import sys

MAGIC = 0x4C463236  # 'LF26'
UDP_IP = "127.0.0.1"
UDP_PORT = 9876
PACK_FORMAT = "=IIiif8fffffffBBBB"  # 76 bytes payload (truoc checksum)

def compute_checksum(payload_bytes: bytes) -> int:
    sum_val = 0x5A5AA5A5
    for b in payload_bytes:
        sum_val = ((sum_val * 31) ^ b) & 0xFFFFFFFF
    return sum_val

def build_packet(
    timestamp_ms: int,
    enc_l: int,
    enc_r: int,
    centroid_mm: float,
    ir_raw: list[float],
    tof_front: float,
    tof_left: float,
    tof_right: float,
    pwm_l: float,
    pwm_r: float,
    battery_v: float,
    mode: int = 4,  # kRun2Sprint
    bypass_state: int = 0,
    line_lost: int = 0,
) -> bytes:
    payload = struct.pack(
        PACK_FORMAT,
        MAGIC,
        timestamp_ms,
        enc_l,
        enc_r,
        centroid_mm,
        *ir_raw,
        tof_front,
        tof_left,
        tof_right,
        pwm_l,
        pwm_r,
        battery_v,
        mode,
        bypass_state,
        line_lost,
        0,  # reserved
    )
    checksum = compute_checksum(payload)
    return payload + struct.pack("=I", checksum)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("=" * 60)
    print(" [MOCK ROBOT] Bat dau phat UDP Telemetry toi 127.0.0.1:9876")
    print(" [INFO] Mo Raylib Sim va bam [F4] de vao Ground Station mode.")
    print(" [INFO] Bam Ctrl+C de dung gia lap.")
    print("=" * 60)

    # Thong so dong hoc robot (khop voi robot_config.hpp)
    wheel_diameter_m = 0.034
    wheel_base_m = 0.138
    ticks_per_rev = 560.0
    m_per_tick = (math.pi * wheel_diameter_m) / ticks_per_rev

    dt = 0.02  # 50 Hz
    t = 0.0
    enc_l = 0
    enc_r = 0

    x = 1200.0  # mm
    y = 800.0   # mm
    theta = 0.0
    battery_v = 8.35

    pkt_count = 0
    start_time = time.time()

    try:
        while True:
            # Quy dao so 8 (Lemniscate): v = 1.0 m/s, w = omega * cos(omega * t)
            # Giup robot re trai, re phai luan phien de ve track hinh so 8 ro net
            v_linear = 0.95  # m/s
            w_angular = 1.25 * math.sin(0.8 * t)  # rad/s

            v_l = v_linear - w_angular * (wheel_base_m * 0.5)
            v_r = v_linear + w_angular * (wheel_base_m * 0.5)

            d_ticks_l = int((v_l * dt) / m_per_tick)
            d_ticks_r = int((v_r * dt) / m_per_tick)
            enc_l += d_ticks_l
            enc_r += d_ticks_r

            # Mo phong 8 mat hong ngoai va sai so centroid
            centroid_err = 3.5 * math.sin(2.0 * t)
            ir = [0.05] * 8
            # Vach den nam o giua (mat 3 va 4)
            ir[3] = 0.85 - 0.2 * (centroid_err / 10.0)
            ir[4] = 0.85 + 0.2 * (centroid_err / 10.0)

            tof_f = 0.55 + 0.25 * math.cos(0.5 * t)
            tof_l = 0.40 + 0.10 * math.sin(t)
            tof_r = 0.40 - 0.10 * math.sin(t)

            battery_v = max(7.4, 8.35 - (t * 0.001))

            pkt = build_packet(
                timestamp_ms=int(t * 1000),
                enc_l=enc_l,
                enc_r=enc_r,
                centroid_mm=centroid_err,
                ir_raw=ir,
                tof_front=tof_f,
                tof_left=tof_l,
                tof_right=tof_r,
                pwm_l=v_l / 2.5,
                pwm_r=v_r / 2.5,
                battery_v=battery_v,
            )

            sock.sendto(pkt, (UDP_IP, UDP_PORT))
            pkt_count += 1

            if pkt_count % 50 == 0:
                print(f"[TX #{pkt_count:05d}] t={t:05.1f}s | EncL={enc_l}, EncR={enc_r} | V={v_linear:.2f}m/s | Bat={battery_v:.2f}V")

            t += dt
            time_to_sleep = dt - ((time.time() - start_time) - t)
            if time_to_sleep > 0.001:
                time.sleep(time_to_sleep)

    except KeyboardInterrupt:
        print("\n[STOP] Da dung phat du lieu telemetry.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
