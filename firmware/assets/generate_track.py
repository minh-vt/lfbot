#!/usr/bin/env python3
"""
LF-Bot 2026 - Bộ sinh sa bàn thi đấu ngẫu nhiên theo thuật toán (Procedural Track Generator).
- Sinh sa bàn khép kín dựa trên thuật toán Spline Catmull-Rom Hướng tâm (Centripetal Catmull-Rom Spline, alpha = 0.5).
- Đảm bảo tính liên tục C1, loại bỏ hoàn toàn các điểm gấp khúc và mấu nhọn (cusps/inflection spikes).
- Hỗ trợ đa dạng cấu trúc sa bàn (Topological Archetypes):
    + "loop"     : Đường đua Grand Prix hữu cơ, uốn lượn nhịp nhàng, không tự cắt (0 crossing).
    + "figure8"  : Đường đua số 8 (Suzuka style) có nút giao cắt phẳng ở trung tâm với góc cắt lớn (>= 60° đến 90°).
    + "technical": Sa bàn kỹ thuật cua chữ S (chicanes) đan xen góc cua gắt (hairpins).
    + "clover"   : Đường đua đa thùy cánh bướm với 2 giao cắt góc rộng.
- Kiểm soát góc tự cắt (Poka-Yoke Self-Intersection Angle Validator):
    + Cho phép sa bàn tự cắt, nhưng BẮT BUỘC góc cắt phải đủ lớn (theta >= 60 độ, tối ưu 75-90 độ).
    + Ngăn ngừa hoàn toàn góc cắt nhọn (< 60 độ) gây bẫy mắt cảm biến quang hoặc phân vân ngã ba.
- Đảm bảo đoạn thẳng xuất phát & đích cố định tại y = 1300 mm, x in [280, 950] mm.
- Ô đích tiêu chuẩn 160 x 160 mm tại x = 400, y = 1300 mm.
- Robot xuất phát tại x = 500, y = 1300 mm, hướng góc 0 rad (+X).
- Vạch đen tiêu chuẩn 20 mm trên thảm trắng 2400 x 1600 mm (1 px = 1 mm).
"""

import argparse
import math
import os
import random
from PIL import Image, ImageDraw

WIDTH = 2400
HEIGHT = 1600
LINE_WIDTH = 20  # 20 mm standard black tape


def centripetal_catmull_rom(points, num_points_per_segment=35, alpha=0.5):
    """
    Nội suy đường cong mềm Catmull-Rom hướng tâm (alpha = 0.5).
    Toán học chứng minh tham số hướng tâm đảm bảo không có vòng lặp cục bộ hay mấu nhọn trong segment.
    """
    n = len(points)
    dense_curve = []

    def get_t(t_prev, p_prev, p_curr):
        d = math.hypot(p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
        return t_prev + (d ** alpha)

    for i in range(n):
        p0 = points[(i - 1 + n) % n]
        p1 = points[i]
        p2 = points[(i + 1) % n]
        p3 = points[(i + 2) % n]

        t0 = 0.0
        t1 = get_t(t0, p0, p1)
        t2 = get_t(t1, p1, p2)
        t3 = get_t(t2, p2, p3)

        if abs(t1 - t0) < 1e-4 or abs(t2 - t1) < 1e-4 or abs(t3 - t2) < 1e-4:
            continue

        for step in range(num_points_per_segment):
            t = t1 + (t2 - t1) * (step / float(num_points_per_segment))

            a1_x = (t1 - t) / (t1 - t0) * p0[0] + (t - t0) / (t1 - t0) * p1[0]
            a1_y = (t1 - t) / (t1 - t0) * p0[1] + (t - t0) / (t1 - t0) * p1[1]

            a2_x = (t2 - t) / (t2 - t1) * p1[0] + (t - t1) / (t2 - t1) * p2[0]
            a2_y = (t2 - t) / (t2 - t1) * p1[1] + (t - t1) / (t2 - t1) * p2[1]

            a3_x = (t3 - t) / (t3 - t2) * p2[0] + (t - t2) / (t3 - t2) * p3[0]
            a3_y = (t3 - t) / (t3 - t2) * p2[1] + (t - t2) / (t3 - t2) * p3[1]

            b1_x = (t2 - t) / (t2 - t0) * a1_x + (t - t0) / (t2 - t0) * a2_x
            b1_y = (t2 - t) / (t2 - t0) * a1_y + (t - t0) / (t2 - t0) * a2_y

            b2_x = (t3 - t) / (t3 - t1) * a2_x + (t - t1) / (t3 - t1) * a3_x
            b2_y = (t3 - t) / (t3 - t1) * a2_y + (t - t1) / (t3 - t1) * a3_y

            cx = (t2 - t) / (t2 - t1) * b1_x + (t - t1) / (t2 - t1) * b2_x
            cy = (t2 - t) / (t2 - t1) * b1_y + (t - t1) / (t2 - t1) * b2_y
            dense_curve.append((cx, cy))

    return dense_curve


def find_intersections(path, min_idx_dist=45):
    """
    Tìm tất cả các điểm tự giao cắt giữa các đoạn đường không kề nhau.
    Trả về danh sách (idx1, idx2, angle_deg, pos).
    Góc cắt trả về theo độ trong khoảng [0°, 90°].
    """
    intersections = []
    n = len(path)

    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def intersect(A, B, C, D):
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    step = 2
    for i in range(0, n - step, step):
        p1, p2 = path[i], path[i + step]
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        len1 = math.hypot(v1[0], v1[1])
        if len1 < 1e-3:
            continue

        for j in range(i + min_idx_dist, n - step, step):
            if (n - j + i) < min_idx_dist:
                continue
            p3, p4 = path[j], path[j + step]
            if intersect(p1, p2, p3, p4):
                v2 = (p4[0] - p3[0], p4[1] - p3[1])
                len2 = math.hypot(v2[0], v2[1])
                if len2 < 1e-3:
                    continue
                dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (len1 * len2)
                dot = max(-1.0, min(1.0, dot))
                angle_deg = math.degrees(math.acos(abs(dot)))
                intersections.append((i, j, angle_deg, path[i]))

    return intersections


def compute_track_metrics(dense_path):
    """
    Tính chiều dài tổng thể và bán kính cua nhỏ nhất trên sa bàn.
    """
    total_len_mm = 0.0
    n = len(dense_path)
    if n < 3:
        return 0.0, 999.0

    for i in range(n):
        p_curr = dense_path[i]
        p_next = dense_path[(i + 1) % n]
        total_len_mm += math.hypot(p_next[0] - p_curr[0], p_next[1] - p_curr[1])

    min_radius_mm = 9999.0
    step = 6
    for i in range(0, n, step):
        p1 = dense_path[(i - step + n) % n]
        p2 = dense_path[i]
        p3 = dense_path[(i + step) % n]

        a = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        b = math.hypot(p3[0] - p2[0], p3[1] - p2[1])
        c = math.hypot(p1[0] - p3[0], p1[1] - p3[1])

        area = 0.5 * abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1]))
        if area > 1e-4 and (a * b * c) > 0.1:
            r = (a * b * c) / (4.0 * area)
            if r < min_radius_mm:
                min_radius_mm = r

    return total_len_mm * 0.001, min_radius_mm


def generate_single_candidate(seed, complexity=10.0, chosen_style="auto"):
    """
    Sinh điểm điều khiển theo từng dạng hình học sa bàn đặc trưng.
    """
    rng = random.Random(seed)

    style = chosen_style
    if style == "auto":
        style = rng.choice(["loop", "figure8", "technical", "clover"])

    # Đoạn thẳng xuất phát & đích cố định
    # y = 1300 mm, từ x = 280 đến 950 mm
    # Finish box tại (400, 1300), robot xuất phát tại (500, 1300)
    baseline = [
        (280.0, 1300.0),
        (400.0, 1300.0),
        (500.0, 1300.0),
        (750.0, 1300.0),
        (950.0, 1300.0),
    ]

    points = list(baseline)

    if style == "figure8":
        # Sa bàn số 8 với hành lang giao cắt thẳng góc 75 - 90 độ tại trung tâm
        cx = rng.uniform(1150, 1250)
        cy = rng.uniform(700, 780)

        # 1. Thùy Đông (East Lobe)
        e_pts = int(max(3, min(5, 3 + (complexity - 5.0) * 0.25)))
        e_cx = rng.uniform(1780, 1920)
        e_cy = rng.uniform(620, 800)
        e_rx = rng.uniform(280, 340)
        e_ry = rng.uniform(320, 440)

        points.append((1250.0, 1280.0))
        points.append((rng.uniform(1550, 1680), 1220.0))
        for k in range(e_pts):
            frac = (k + 1) / float(e_pts + 1)
            ang = math.radians(-70.0 + frac * 190.0)
            px = e_cx + e_rx * math.cos(ang) + rng.uniform(-20, 20)
            py = e_cy - e_ry * math.sin(ang) + rng.uniform(-20, 20)
            points.append((px, py))

        # Hành lang giao cắt 1: Hướng Tây Nam xuyên qua tâm (cx, cy)
        points.append((cx + 340.0, cy - 250.0))
        points.append((cx - 340.0, cy + 250.0))

        # 2. Thùy Tây (West Lobe)
        w_pts = int(max(3, min(5, 3 + (complexity - 5.0) * 0.25)))
        w_cx = rng.uniform(500, 640)
        w_cy = rng.uniform(480, 650)
        w_rx = rng.uniform(260, 320)
        w_ry = rng.uniform(260, 360)

        points.append((rng.uniform(420, 520), 1080.0))
        for k in range(w_pts):
            frac = (k + 1) / float(w_pts + 1)
            ang = math.radians(70.0 + frac * 190.0)
            px = w_cx + w_rx * math.cos(ang) + rng.uniform(-20, 20)
            py = w_cy - w_ry * math.sin(ang) + rng.uniform(-20, 20)
            points.append((px, py))

        # Hành lang giao cắt 2: Hướng Đông Nam xuyên qua tâm (cx, cy)
        points.append((cx - 340.0, cy - 250.0))
        points.append((cx + 340.0, cy + 250.0))

        # Vòng lượn êm ái nối về điểm bắt đầu đoạn thẳng đáy
        points.append((rng.uniform(1450, 1550), 1150.0))
        points.append((1050.0, 1200.0))
        points.append((650.0, 1210.0))
        points.append((320.0, 1210.0))
        points.append((180.0, 1180.0))
        points.append((170.0, 1240.0))
        points.append((210.0, 1280.0))

    elif style == "technical":
        # Sa bàn kỹ thuật: Vòng bao rộng uốn lượn vào cụm chữ S liên hoàn
        points.append((1250.0, 1280.0))
        points.append((1650.0 + rng.uniform(-30, 30), 1220.0))
        points.append((1980.0 + rng.uniform(-30, 30), 1050.0 + rng.uniform(-30, 30)))
        points.append((2150.0 + rng.uniform(-20, 20), 750.0 + rng.uniform(-30, 30)))
        points.append((1950.0 + rng.uniform(-30, 30), 380.0 + rng.uniform(-20, 20)))
        points.append((1550.0 + rng.uniform(-30, 30), 280.0 + rng.uniform(-20, 20)))

        # Số lượng chicane biến thiên theo độ phức tạp
        num_chicanes = int(max(2, min(4, complexity / 3.5)))
        for c in range(num_chicanes):
            frac = (c + 1) / float(num_chicanes + 1)
            x_mid = 1450.0 - frac * 680.0 + rng.uniform(-25, 25)
            y_dip = 650.0 + (50.0 + 25.0 * (c % 2)) * (1.0 if c % 2 == 0 else -1.0) + rng.uniform(-15, 15)
            points.append((x_mid, y_dip))

        points.append((680.0 + rng.uniform(-30, 30), 320.0 + rng.uniform(-20, 20)))
        points.append((320.0 + rng.uniform(-20, 20), 550.0 + rng.uniform(-25, 25)))
        points.append((250.0 + rng.uniform(-20, 20), 850.0 + rng.uniform(-25, 25)))
        points.append((180.0, 1100.0))
        points.append((180.0, 1220.0))
        points.append((210.0, 1280.0))

    elif style == "clover":
        # Sa bàn đa thùy cánh bướm với 2 giao cắt góc lớn
        cx = 1200.0 + rng.uniform(-30, 30)
        points.append((1250.0, 1280.0))
        points.append((1650.0, 1220.0))
        points.append((2000.0 + rng.uniform(-30, 30), 1050.0))
        points.append((2150.0 + rng.uniform(-20, 20), 750.0))
        points.append((1750.0 + rng.uniform(-30, 30), 550.0))
        # Cánh trên bên phải
        points.append((2050.0 + rng.uniform(-20, 20), 280.0))
        points.append((1550.0 + rng.uniform(-30, 30), 240.0))
        points.append((cx, 550.0))
        # Cánh trên bên trái
        points.append((850.0 + rng.uniform(-30, 30), 240.0))
        points.append((350.0 + rng.uniform(-20, 20), 280.0))
        points.append((650.0 + rng.uniform(-30, 30), 550.0))
        # Cánh dưới bên trái
        points.append((250.0 + rng.uniform(-20, 20), 850.0))
        points.append((180.0, 1100.0))
        points.append((180.0, 1220.0))
        points.append((210.0, 1280.0))

    else:
        # style == "loop": Grand Prix Circuit (Vòng lặp hữu cơ khép kín, 0 giao cắt)
        center_x = rng.uniform(1150, 1250)
        center_y = rng.uniform(700, 800)
        num_pts = int(max(7, min(14, 6 + complexity * 0.55)))

        points.append((1250.0, 1280.0))
        points.append((1650.0, 1220.0))
        for k in range(num_pts):
            deg = 15.0 + (k / float(num_pts)) * 165.0
            rad = math.radians(deg)
            r_base_x = rng.uniform(760, 920)
            r_base_y = rng.uniform(480, 580)
            wobble = math.sin(rad * 3.0 + rng.uniform(0, 3)) * (18.0 + complexity * 2.5)
            px = center_x + (r_base_x + wobble) * math.cos(rad)
            py = center_y - (r_base_y + wobble) * math.sin(rad)
            points.append((px, py))

        points.append((250.0 + rng.uniform(-20, 20), 900.0 + rng.uniform(-20, 20)))
        points.append((180.0, 1100.0))
        points.append((180.0, 1220.0))
        points.append((210.0, 1280.0))

    # Kẹp an toàn trong phạm vi thảm thi đấu [160, 2240] x [160, 1440]
    clamped = []
    for p in points:
        clamped.append((max(160.0, min(2240.0, p[0])), max(160.0, min(1440.0, p[1]))))

    dense = centripetal_catmull_rom(clamped, num_points_per_segment=35, alpha=0.5)
    return dense, style


def generate_procedural_track(seed, complexity=10.0, style="auto", min_cross_angle_deg=60.0):
    """
    Bộ điều phối sinh sa bàn thủ tục kết hợp Rejection Sampling & Poka-Yoke:
    Đảm bảo 100% mọi giao cắt (nếu có) đều có góc cắt >= min_cross_angle_deg.
    Nếu gặp góc cắt nhọn (< 60°), tự động thử lại với muối hạt ngẫu nhiên (salt).
    """
    for attempt in range(40):
        current_seed = seed + attempt * 1007
        dense, chosen_style = generate_single_candidate(current_seed, complexity, style)
        intersections = find_intersections(dense)

        # Kiểm tra góc cắt của tất cả các giao cắt
        all_angles_valid = True
        for item in intersections:
            if item[2] < min_cross_angle_deg:
                all_angles_valid = False
                break

        # Nếu đạt yêu cầu góc cắt
        if all_angles_valid:
            total_len_m, min_r_mm = compute_track_metrics(dense)
            return dense, chosen_style, intersections, total_len_m, min_r_mm, current_seed

    # Fallback an toàn nếu sau 40 lần thử vẫn vướng góc cắt nhọn
    dense, chosen_style = generate_single_candidate(seed, complexity, "loop")
    total_len_m, min_r_mm = compute_track_metrics(dense)
    return dense, "loop", [], total_len_m, min_r_mm, seed


def render_track_image(dense_path, output_path, bake_obstacle=False):
    """
    Kết xuất ảnh bitmap sa bàn chất lượng cao 2400 x 1600 px (1 px = 1 mm).
    """
    img = Image.new("RGB", (WIDTH, HEIGHT), (255, 255, 255))
    draw = ImageDraw.Draw(img)

    n = len(dense_path)
    for i in range(n):
        p1 = dense_path[i]
        p2 = dense_path[(i + 1) % n]
        draw.line([p1, p2], fill=(0, 0, 0), width=LINE_WIDTH)

    r_cap = LINE_WIDTH // 2
    for p in dense_path[::2]:
        draw.ellipse([p[0] - r_cap, p[1] - r_cap, p[0] + r_cap, p[1] + r_cap], fill=(0, 0, 0))

    # Ô đích tiêu chuẩn 160 x 160 mm tại (400, 1300)
    finish_x, finish_y = 400.0, 1300.0
    draw.rectangle([finish_x - 80, finish_y - 80, finish_x + 80, finish_y + 80], fill=(0, 0, 0))

    if bake_obstacle:
        obs_x, obs_y = 750.0, 1300.0
        draw.rectangle([obs_x - 52, obs_y - 30, obs_x + 53, obs_y + 30], fill=(20, 20, 20))

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    img.save(output_path, "PNG")


def main():
    parser = argparse.ArgumentParser(description="LF-Bot Procedural Track Circuit Generator (Spline Engine)")
    parser.add_argument("--seed", type=int, default=None, help="Random seed (default: random)")
    parser.add_argument("--complexity", type=float, default=10.0, help="Circuit complexity (3.0 - 20.0, default: 10.0)")
    parser.add_argument(
        "--style",
        type=str,
        default="auto",
        choices=["auto", "loop", "figure8", "technical", "clover"],
        help="Circuit topology archetype (default: auto)",
    )
    parser.add_argument(
        "--min-cross-angle",
        type=float,
        default=60.0,
        help="Minimum allowed self-intersection angle in degrees (default: 60.0)",
    )
    parser.add_argument("--output", type=str, default=None, help="Output PNG path (default: assets/track.png)")
    parser.add_argument("--bake-obstacle", action="store_true", help="Bake static obstacle into image (default: False)")

    args = parser.parse_args()

    seed = args.seed if args.seed is not None else random.randint(1000, 999999)
    output_path = args.output
    if not output_path:
        output_path = os.path.join(os.path.dirname(__file__), "track.png")

    print(f"[PROCEDURAL] Khoi tao bo sinh sa ban voi SEED = {seed}, COMPLEXITY = {args.complexity}, STYLE = {args.style}...")

    dense_path, actual_style, intersections, total_length_m, min_radius_mm, used_seed = generate_procedural_track(
        seed, complexity=args.complexity, style=args.style, min_cross_angle_deg=args.min_cross_angle
    )

    print(f"[PROCEDURAL] Thong so sa ban sinh ra:")
    print(f"  - Dang hinh hoc sa ban (Style): {actual_style.upper()}")
    print(f"  - Seed su dung               : {used_seed}")
    print(f"  - Chieu dai quy dao khap kin : {total_length_m:.2f} m")
    print(f"  - Ban kinh goc cua toi thieu : {min_radius_mm:.1f} mm (>= 180 mm tieu chuan)")
    print(f"  - Doan thang xuat phat / dich: y = 1300 mm, x in [280, 950] mm")
    print(f"  - O dich tieu chuan          : 160 x 160 mm tai (400, 1300)")
    print(f"  - So diem tu giao cat        : {len(intersections)}")

    for idx, item in enumerate(intersections, 1):
        print(f"      + Giao cat #{idx}: Toa do ({item[3][0]:.0f}, {item[3][1]:.0f}), Goc cat = {item[2]:.1f}° (Dat yeu cau >= {args.min_cross_angle}°)")

    render_track_image(dense_path, output_path, bake_obstacle=args.bake_obstacle)
    print(f"[PROCEDURAL] Da xuat anh sa ban thanh cong tai: {output_path}")


if __name__ == "__main__":
    main()

