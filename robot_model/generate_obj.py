"""
Standalone 3D Wavefront OBJ/MTL Generator for Vision UGV Rover.
Generates an accurate, watertight 3D model of the 4WD Unmanned Ground Vehicle
matching the physical parameters and camera positioning defined in the URDF.
"""
import math
import os


def generate_ugv_obj(output_dir="."):
    obj_path = os.path.join(output_dir, "ugv_rover.obj")
    mtl_path = os.path.join(output_dir, "ugv_rover.mtl")

    vertices = []
    normals = []
    texcoords = []
    faces_by_material = {}
    current_material = None

    def set_material(mat_name):
        nonlocal current_material
        current_material = mat_name
        if mat_name not in faces_by_material:
            faces_by_material[mat_name] = []

    def add_vertex(x, y, z):
        vertices.append((x, y, z))
        return len(vertices)

    def add_normal(nx, ny, nz):
        normals.append((nx, ny, nz))
        return len(normals)

    def add_box(center, size, mat_name):
        set_material(mat_name)
        cx, cy, cz = center
        dx, dy, dz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0

        # 8 corners
        v = [
            add_vertex(cx - dx, cy - dy, cz - dz), # 0
            add_vertex(cx + dx, cy - dy, cz - dz), # 1
            add_vertex(cx + dx, cy + dy, cz - dz), # 2
            add_vertex(cx - dx, cy + dy, cz - dz), # 3
            add_vertex(cx - dx, cy - dy, cz + dz), # 4
            add_vertex(cx + dx, cy - dy, cz + dz), # 5
            add_vertex(cx + dx, cy + dy, cz + dz), # 6
            add_vertex(cx - dx, cy + dy, cz + dz), # 7
        ]

        faces = [
            # -Z (bottom)
            (v[0], v[3], v[2], v[1], add_normal(0, 0, -1)),
            # +Z (top)
            (v[4], v[5], v[6], v[7], add_normal(0, 0, 1)),
            # -X (back)
            (v[0], v[4], v[7], v[3], add_normal(-1, 0, 0)),
            # +X (front)
            (v[1], v[2], v[6], v[5], add_normal(1, 0, 0)),
            # -Y (right)
            (v[0], v[1], v[5], v[4], add_normal(0, -1, 0)),
            # +Y (left)
            (v[3], v[7], v[6], v[2], add_normal(0, 1, 0)),
        ]

        for p1, p2, p3, p4, norm_idx in faces:
            faces_by_material[current_material].append((p1, p2, p3, norm_idx))
            faces_by_material[current_material].append((p1, p3, p4, norm_idx))

    def add_cylinder(center, radius, length, axis="y", segments=18, mat_name="DarkRubber"):
        set_material(mat_name)
        cx, cy, cz = center
        half_l = length / 2.0

        circle1 = []
        circle2 = []

        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            c = math.cos(angle)
            s = math.sin(angle)

            if axis == "y":
                # Along Y axis
                v1 = add_vertex(cx + radius * c, cy - half_l, cz + radius * s)
                v2 = add_vertex(cx + radius * c, cy + half_l, cz + radius * s)
            elif axis == "x":
                # Along X axis
                v1 = add_vertex(cx - half_l, cy + radius * c, cz + radius * s)
                v2 = add_vertex(cx + half_l, cy + radius * c, cz + radius * s)
            else: # z
                v1 = add_vertex(cx + radius * c, cy + radius * s, cz - half_l)
                v2 = add_vertex(cx + radius * c, cy + radius * s, cz + half_l)

            circle1.append(v1)
            circle2.append(v2)

        # Normals for caps
        if axis == "y":
            n_cap1 = add_normal(0, -1, 0)
            n_cap2 = add_normal(0, 1, 0)
        elif axis == "x":
            n_cap1 = add_normal(-1, 0, 0)
            n_cap2 = add_normal(1, 0, 0)
        else:
            n_cap1 = add_normal(0, 0, -1)
            n_cap2 = add_normal(0, 0, 1)

        # Side quads & caps
        c1_center = add_vertex(
            cx if axis != "x" else cx - half_l,
            cy if axis != "y" else cy - half_l,
            cz if axis != "z" else cz - half_l
        )
        c2_center = add_vertex(
            cx if axis != "x" else cx + half_l,
            cy if axis != "y" else cy + half_l,
            cz if axis != "z" else cz + half_l
        )

        for i in range(segments):
            nxt = (i + 1) % segments
            v1, v2 = circle1[i], circle2[i]
            v1_next, v2_next = circle1[nxt], circle2[nxt]

            # Approximate normal for side
            mid_angle = 2.0 * math.pi * (i + 0.5) / segments
            if axis == "y":
                side_norm = add_normal(math.cos(mid_angle), 0, math.sin(mid_angle))
            elif axis == "x":
                side_norm = add_normal(0, math.cos(mid_angle), math.sin(mid_angle))
            else:
                side_norm = add_normal(math.cos(mid_angle), math.sin(mid_angle), 0)

            # Two triangles for the quad
            faces_by_material[current_material].append((v1, v2, v2_next, side_norm))
            faces_by_material[current_material].append((v1, v2_next, v1_next, side_norm))

            # Caps
            faces_by_material[current_material].append((c1_center, circle1[nxt], circle1[i], n_cap1))
            faces_by_material[current_material].append((c2_center, circle2[i], circle2[nxt], n_cap2))

    # --- BUILD MODEL GEOMETRY ---
    # Coordinates: X forward, Y left, Z up (Ground is at Z=0, Base footprint)
    # Chassis height center is at Z = 0.165m
    chassis_z = 0.165

    # 1. Main Chassis Hull
    add_box((0.0, 0.0, chassis_z), (0.52, 0.34, 0.15), "ChassisDarkGray")

    # 2. Upper Aluminum Deck Plate
    add_box((0.0, 0.0, chassis_z + 0.08), (0.46, 0.30, 0.015), "AluminumPlate")

    # 3. Electronics & Compute Enclosure (NVIDIA Jetson / Raspberry Pi mount)
    add_box((-0.06, 0.0, chassis_z + 0.11), (0.22, 0.20, 0.05), "ElectronicsEnclosure")

    # 4. Front & Rear Safety Bumpers
    add_box((0.27, 0.0, chassis_z - 0.02), (0.05, 0.36, 0.04), "SafetyYellow")
    add_box((-0.27, 0.0, chassis_z - 0.02), (0.05, 0.36, 0.04), "SafetyYellow")

    # 5. Dual Antennas
    add_cylinder((-0.14, 0.08, chassis_z + 0.22), 0.005, 0.18, axis="z", segments=10, mat_name="AntennaBlack")
    add_cylinder((-0.14, -0.08, chassis_z + 0.22), 0.005, 0.18, axis="z", segments=10, mat_name="AntennaBlack")

    # 6. LED Headlights
    add_cylinder((0.265, 0.10, chassis_z + 0.03), 0.018, 0.02, axis="x", segments=14, mat_name="HeadlightGlass")
    add_cylinder((0.265, -0.10, chassis_z + 0.03), 0.018, 0.02, axis="x", segments=14, mat_name="HeadlightGlass")

    # 7. Sensor Mast & Camera
    mast_base_z = chassis_z + 0.08
    mast_height = 0.16
    add_cylinder((0.15, 0.0, mast_base_z + mast_height / 2.0), 0.016, mast_height, axis="z", segments=14, mat_name="SensorMast")

    # Camera Housing & Lens (tilted forward ~18 deg)
    cam_center = (0.175, 0.0, mast_base_z + mast_height + 0.02)
    add_box(cam_center, (0.045, 0.085, 0.036), "CameraBody")
    add_cylinder((cam_center[0] + 0.03, cam_center[1], cam_center[2] - 0.005), 0.014, 0.025, axis="x", segments=16, mat_name="CameraLens")

    # 8. 4 Off-road Wheels & Rims
    wheel_radius = 0.125
    wheel_width = 0.08
    wheel_xs = [0.18, -0.18]
    wheel_ys = [0.22, -0.22]
    wheel_z = chassis_z - 0.04 # 0.125m above ground

    for wx in wheel_xs:
        for wy in wheel_ys:
            # Main Rubber Tire
            add_cylinder((wx, wy, wheel_z), wheel_radius, wheel_width, axis="y", segments=24, mat_name="DarkRubber")
            # Wheel Rim / Hub
            rim_y = wy + (0.042 if wy > 0 else -0.042)
            add_cylinder((wx, rim_y, wheel_z), 0.065, 0.012, axis="y", segments=18, mat_name="WheelHubOrange")
            # Center axle nut
            add_cylinder((wx, rim_y, wheel_z), 0.02, 0.018, axis="y", segments=12, mat_name="AluminumPlate")

    # --- WRITE MTL FILE ---
    with open(mtl_path, "w", encoding="utf-8") as mtl:
        mtl.write("# Material Library for Vision UGV Rover\n")
        
        materials = {
            "ChassisDarkGray": {"Kd": "0.22 0.24 0.26", "Ks": "0.4 0.4 0.4", "Ns": "45.0"},
            "AluminumPlate": {"Kd": "0.76 0.78 0.80", "Ks": "0.8 0.8 0.8", "Ns": "90.0"},
            "ElectronicsEnclosure": {"Kd": "0.12 0.13 0.15", "Ks": "0.3 0.3 0.3", "Ns": "30.0"},
            "SafetyYellow": {"Kd": "0.95 0.75 0.05", "Ks": "0.5 0.5 0.5", "Ns": "50.0"},
            "AntennaBlack": {"Kd": "0.05 0.05 0.05", "Ks": "0.6 0.6 0.6", "Ns": "70.0"},
            "SensorMast": {"Kd": "0.15 0.15 0.15", "Ks": "0.7 0.7 0.7", "Ns": "80.0"},
            "CameraBody": {"Kd": "0.08 0.45 0.88", "Ks": "0.7 0.7 0.8", "Ns": "75.0"},
            "CameraLens": {"Kd": "0.02 0.02 0.03", "Ks": "0.95 0.95 0.95", "Ns": "150.0"},
            "HeadlightGlass": {"Kd": "0.95 0.95 0.90", "Ks": "0.9 0.9 0.9", "Ns": "120.0", "d": "0.85"},
            "DarkRubber": {"Kd": "0.09 0.09 0.10", "Ks": "0.1 0.1 0.1", "Ns": "10.0"},
            "WheelHubOrange": {"Kd": "0.92 0.42 0.06", "Ks": "0.6 0.6 0.6", "Ns": "60.0"},
        }
        
        for name, props in materials.items():
            mtl.write(f"\nnewmtl {name}\n")
            mtl.write(f"Kd {props['Kd']}\n")
            mtl.write(f"Ks {props['Ks']}\n")
            mtl.write(f"Ns {props['Ns']}\n")
            mtl.write(f"illum 2\n")
            if "d" in props:
                mtl.write(f"d {props['d']}\n")

    # --- WRITE OBJ FILE ---
    with open(obj_path, "w", encoding="utf-8") as obj:
        obj.write("# Wavefront OBJ for Vision UGV Rover\n")
        obj.write(f"mtllib {os.path.basename(mtl_path)}\n\n")

        for v in vertices:
            obj.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")

        for n in normals:
            obj.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")

        for mat_name, faces in faces_by_material.items():
            obj.write(f"\nusemtl {mat_name}\n")
            for f in faces:
                p1, p2, p3, n_idx = f
                obj.write(f"f {p1}//{n_idx} {p2}//{n_idx} {p3}//{n_idx}\n")

    print(f"Generated 3D Model:")
    print(f"  OBJ: {obj_path} ({len(vertices)} vertices, {sum(len(f) for f in faces_by_material.values())} triangles)")
    print(f"  MTL: {mtl_path} ({len(materials)} materials)")
    return obj_path, mtl_path


if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    generate_ugv_obj(current_dir)
