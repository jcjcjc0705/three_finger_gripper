#!/usr/bin/env python3
"""Regenerate the gripper description from the Isaac-authored USD.

    pip install usd-core trimesh scipy numpy
    python3 tools/usd2urdf.py

Link and joint names are kept exactly as the CAD authored them so the URDF,
the USD and the mechanical drawings all agree. Motor numbering:

    motor 1  C_2_Joint   thumb  proximal flexion
    motor 2  C_3_Joint   thumb  distal   flexion
    motor 3  A_2_Joint   index  proximal flexion
    motor 4  A_3_Joint   index  distal   flexion
    motor 5  B_2_Joint   middle proximal flexion
    motor 6  B_3_Joint   middle distal   flexion
    motor 7  A_1_Joint   index  spread
    motor 8  B_1_Joint   middle spread
"""

import os
import zipfile

import numpy as np
import trimesh
from pxr import Usd, UsdGeom, UsdPhysics

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USDZ = os.path.join(HERE, 'cad', '3FingerGripper_Blue.usdz')
OUT = os.path.join(HERE, 'src', 'three_finger_gripper_description')
PKG = 'three_finger_gripper_description'
DENSITY = 1200.0        # kg/m^3, 3D-printed plastic

MOTOR = {'C_2_Joint': 1, 'C_3_Joint': 2, 'A_2_Joint': 3, 'A_3_Joint': 4,
         'B_2_Joint': 5, 'B_3_Joint': 6, 'A_1_Joint': 7, 'B_1_Joint': 8}


def open_stage():
    """The stage declares metersPerUnit 0.01 but the geometry is authored in
    metres -- read as centimetres the whole gripper would be 1.7 mm across."""
    work = os.path.join(HERE, 'cad', '.extracted')
    os.makedirs(work, exist_ok=True)
    with zipfile.ZipFile(USDZ) as z:
        inner = z.namelist()[0]
        z.extract(inner, work)
    return Usd.Stage.Open(os.path.join(work, inner))


def to_np(m):
    return np.array([[m[r][c] for c in range(4)] for r in range(4)])


def quat_R(q):
    w, (x, y, z) = q.GetReal(), q.GetImaginary()
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pose(pos, quat):
    M = np.eye(4)
    M[:3, :3] = quat_R(quat)
    M[:3, 3] = pos
    return M


def rpy(R):
    pitch = np.arcsin(np.clip(-R[2][0], -1, 1))
    if abs(R[2][0]) < 1 - 1e-9:
        return np.arctan2(R[2][1], R[2][2]), pitch, np.arctan2(R[1][0], R[0][0])
    return np.arctan2(-R[1][2], R[1][1]), pitch, 0.0


def fmt(v):
    return ' '.join(f'{x:.9g}' for x in v)


def link_mesh(link):
    """Every Mesh under the rigid body, baked into the link's own frame."""
    inv = to_np(UsdGeom.Xformable(link).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).GetInverse())
    parts = []
    for prim in Usd.PrimRange(link):
        if prim.GetTypeName() != 'Mesh':
            continue
        g = UsdGeom.Mesh(prim)
        pts = np.array(g.GetPointsAttr().Get(), dtype=float)
        counts = np.array(g.GetFaceVertexCountsAttr().Get(), dtype=int)
        idx = np.array(g.GetFaceVertexIndicesAttr().Get(), dtype=int)
        if len(pts) == 0 or len(counts) == 0:
            continue
        tris, o = [], 0
        for c in counts:
            for k in range(1, c - 1):
                tris.append([idx[o], idx[o + k], idx[o + k + 1]])
            o += c
        rel = to_np(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())) @ inv
        h = np.hstack([pts, np.ones((len(pts), 1))])
        parts.append(trimesh.Trimesh(vertices=(h @ rel)[:, :3],
                                     faces=np.array(tris), process=False))
    return trimesh.util.concatenate(parts)


def main():
    stage = open_stage()
    links = [p for p in stage.Traverse()
             if 'PhysicsRigidBodyAPI' in p.GetAppliedSchemas()]
    joints = [p for p in stage.Traverse()
              if p.GetTypeName() == 'PhysicsRevoluteJoint']
    os.makedirs(f'{OUT}/meshes', exist_ok=True)
    os.makedirs(f'{OUT}/urdf', exist_ok=True)

    out = ['<?xml version="1.0"?>',
           '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">', '',
           '  <!-- XC330-M288-T, confirm against the datasheet -->',
           '  <xacro:property name="effort" value="0.9"/>',
           '  <xacro:property name="velocity" value="8.0"/>', '',
           '  <xacro:macro name="three_finger_gripper" params="prefix parent *origin">', '',
           '    <joint name="${prefix}gripper_mount_joint" type="fixed">',
           '      <parent link="${parent}"/>',
           '      <child link="${prefix}base_link"/>',
           '      <xacro:insert_block name="origin"/>',
           '    </joint>', '']

    print(f'{"link":18}{"tris":>7}{"mass(g)":>10}')
    for L in links:
        name = L.GetName()
        mesh = link_mesh(L)
        mesh.export(f'{OUT}/meshes/{name}.stl')
        # CAD tessellation leaves the shells open: the divergence-theorem volume
        # survives that and gives a realistic mass, the inertia tensor does not.
        # Take the tensor's shape from the convex hull, rescaled to that mass.
        hull = mesh.convex_hull
        vol = abs(mesh.volume)
        if not (0 < vol <= abs(hull.volume)):
            vol = abs(hull.volume)
        mass = max(vol * DENSITY, 1e-4)
        it = hull.moment_inertia * DENSITY * (vol / abs(hull.volume))
        print(f'{name:18}{len(mesh.faces):7}{mass * 1000:10.2f}')
        out += [f'    <link name="${{prefix}}{name}">',
                '      <visual>',
                '        <origin xyz="0 0 0" rpy="0 0 0"/>',
                f'        <geometry><mesh filename="package://{PKG}/meshes/{name}.stl"/></geometry>',
                '        <material name="gripper_blue"/>',
                '      </visual>',
                '      <collision>',
                '        <origin xyz="0 0 0" rpy="0 0 0"/>',
                f'        <geometry><mesh filename="package://{PKG}/meshes/{name}.stl"/></geometry>',
                '      </collision>',
                '      <inertial>',
                f'        <origin xyz="{fmt(hull.center_mass)}" rpy="0 0 0"/>',
                f'        <mass value="{mass:.9g}"/>',
                f'        <inertia ixx="{it[0][0]:.6g}" ixy="{it[0][1]:.6g}" ixz="{it[0][2]:.6g}"'
                f' iyy="{it[1][1]:.6g}" iyz="{it[1][2]:.6g}" izz="{it[2][2]:.6g}"/>',
                '      </inertial>',
                '    </link>', '']

    for J in sorted(joints, key=lambda p: MOTOR[p.GetName()]):
        j = UsdPhysics.RevoluteJoint(J)
        name = J.GetName()
        # A USD joint pins one frame on each body:
        #   W_child = W_parent . T0 . R(axis, q) . inv(T1)
        # URDF has a single frame, so fold T1 into the origin and rotate the
        # axis with it. A_2_Joint and A_3_Joint carry a 180 deg localRot1;
        # ignoring it mirrors the index finger onto the wrong side.
        T0 = pose(j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get())
        T1 = pose(j.GetLocalPos1Attr().Get(), j.GetLocalRot1Attr().Get())
        origin = T0 @ np.linalg.inv(T1)
        axis = T1[:3, :3] @ {'X': (1, 0, 0), 'Y': (0, 1, 0),
                             'Z': (0, 0, 1)}[j.GetAxisAttr().Get()]
        axis = np.where(np.abs(axis) < 1e-6, 0.0, axis)
        axis = np.where(np.abs(np.abs(axis) - 1) < 1e-6, np.sign(axis), axis)
        out += [f'    <!-- motor {MOTOR[name]} -->',
                f'    <joint name="${{prefix}}{name}" type="revolute">',
                f'      <parent link="${{prefix}}{j.GetBody0Rel().GetTargets()[0].name}"/>',
                f'      <child link="${{prefix}}{j.GetBody1Rel().GetTargets()[0].name}"/>',
                f'      <origin xyz="{fmt(origin[:3, 3])}" rpy="{fmt(rpy(origin[:3, :3]))}"/>',
                f'      <axis xyz="{fmt(axis)}"/>',
                f'      <limit lower="{np.deg2rad(j.GetLowerLimitAttr().Get()):.6g}"'
                f' upper="{np.deg2rad(j.GetUpperLimitAttr().Get()):.6g}"'
                ' effort="${effort}" velocity="${velocity}"/>',
                '    </joint>', '']

    out += ['  </xacro:macro>', '',
            '  <material name="gripper_blue">',
            '    <color rgba="0.15 0.35 0.75 1.0"/>',
            '  </material>', '', '</robot>', '']
    open(f'{OUT}/urdf/three_finger_gripper_macro.xacro', 'w').write('\n'.join(out))
    print(f'\nwrote {OUT}/urdf/three_finger_gripper_macro.xacro')


if __name__ == '__main__':
    main()
