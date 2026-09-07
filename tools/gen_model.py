#!/usr/bin/env python3
"""Generate the static kinematic and collision model gripper_servo compiles in.

Firmware cannot parse a URDF at run time, so the joint frames, a sphere cover of
each link and the pairs worth checking are baked into a header instead. Rerun
after tools/usd2urdf.py whenever the CAD changes.

    python3 tools/gen_model.py
"""

import os
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(HERE, 'src', 'three_finger_gripper_description')
MACRO = os.path.join(PKG, 'urdf', 'three_finger_gripper_macro.xacro')
OUT = os.path.join(HERE, 'src', 'gripper_servo', 'include', 'gripper_servo',
                   'gripper_model.hpp')

# Motor 1..8. The servo's joint vector is in this order everywhere.
MOTOR_ORDER = ['C_2_Joint', 'C_3_Joint', 'A_2_Joint', 'A_3_Joint',
               'B_2_Joint', 'B_3_Joint', 'A_1_Joint', 'B_1_Joint']
CAPSULES_PER_LINK = 3

# Only the three distal links. Over 901 sampled configurations there was not
# one where a proximal link interfered while the distal links did not, so
# nothing is lost by ignoring the rest -- and the palm and the two spread
# housings are blocky parts a principal-axis capsule fits terribly, with radii
# coming out at 21-64 mm that swallow whatever is nearby.
MODEL_LINKS = ['A_3_Link', 'B_3_Link', 'C_3_Link_001']

# Radii are shrunk so the model fires on interference rather than on passing
# close -- normal clearances here are 1-10 mm, below any convex model's error.
# Measured against mesh ground truth over 901 configurations: 1.9% false
# positives, 0.7% false negatives, and all 283 poses that penetrate deeper than
# 5 mm are caught. At nominal size it is 17.2% false positives for the same
# detection.
SHRINK = 0.006

ADJACENT = {tuple(sorted(p)) for p in [
    ('A_1_Link', 'A_2_Link'), ('A_2_Link', 'A_3_Link'),
    ('B_1_Link_001', 'B_2_Link'), ('B_2_Link', 'B_3_Link'),
    ('C_2_Link', 'C_3_Link_001'),
    ('base_link', 'A_1_Link'), ('base_link', 'B_1_Link_001'),
    ('base_link', 'C_2_Link')]}


def read_joints():
    strip = lambda s: s.replace('${prefix}', '')
    out = {}
    for j in ET.parse(MACRO).getroot().iter('joint'):
        if j.get('type') != 'revolute':
            continue
        o = j.find('origin')
        M = np.eye(4)
        M[:3, 3] = [float(v) for v in o.get('xyz').split()]
        x, y, z = [float(v) for v in o.get('rpy').split()]
        cx, sx, cy, sy, cz, sz = np.cos(x), np.sin(x), np.cos(y), np.sin(y), np.cos(z), np.sin(z)
        M[:3, :3] = [[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                     [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                     [-sy, cy * sx, cy * cx]]
        lim = j.find('limit')
        out[strip(j.get('name'))] = dict(
            parent=strip(j.find('parent').get('link')),
            child=strip(j.find('child').get('link')),
            origin=M,
            axis=np.array([float(v) for v in j.find('axis').get('xyz').split()]),
            lower=float(lim.get('lower')), upper=float(lim.get('upper')))
    return out


def cover(mesh, n):
    """Capsules along the link's principal axis, one per slice.

    Elongated fingers fit a segment far better than a sphere: spheres measured
    119% false positives at four per link, capsules 32% at three.
    """
    pts = np.asarray(mesh.vertices, dtype=float)
    centre = pts.mean(axis=0)
    axis = np.linalg.svd(pts - centre, full_matrices=False)[2][0]
    t = (pts - centre) @ axis
    edges = np.linspace(t.min(), t.max(), n + 1)
    out = []
    for i in range(n):
        sel = pts[(t >= edges[i]) & (t <= edges[i + 1])]
        if len(sel) < 2:
            continue
        s = (sel - centre) @ axis
        a, b = centre + axis * s.min(), centre + axis * s.max()
        d = b - a
        length = np.linalg.norm(d)
        u = d / length if length > 1e-9 else np.zeros(3)
        proj = np.clip((sel - a) @ u, 0, length) if length > 1e-9 else np.zeros(len(sel))
        radius = float(np.linalg.norm(sel - (a + np.outer(proj, u)), axis=1).max())
        out.append((a, b, max(radius - SHRINK, 0.001)))
    return out


def main():
    joints = read_joints()
    # base_link is 0; every other link is named by the joint that drives it, so
    # link index i+1 is driven by joint i.
    links = ['base_link'] + [joints[j]['child'] for j in MOTOR_ORDER]
    index = {n: i for i, n in enumerate(links)}

    # Parents come before children so forward kinematics is a single pass.
    order, placed = [0], {0}
    while len(order) < len(links):
        for i in range(1, len(links)):
            if i in placed:
                continue
            if index[joints[MOTOR_ORDER[i - 1]]['parent']] in placed:
                order.append(i)
                placed.add(i)

    capsules = []
    print(f'{"link":18}{"capsules":>10}{"max r (mm)":>12}')
    for i, name in enumerate(links):
        if name not in MODEL_LINKS:
            continue
        mesh = trimesh.load(f'{PKG}/meshes/{name}.stl', process=False)
        got = cover(mesh, CAPSULES_PER_LINK)
        capsules += [(i, a, b, r) for a, b, r in got]
        print(f'{name:18}{len(got):10}{1000 * max(r for _, _, r in got):12.1f}')

    pairs = []
    for a in range(len(links)):
        for b in range(a + 1, len(links)):
            if links[a] not in MODEL_LINKS or links[b] not in MODEL_LINKS:
                continue
            if tuple(sorted((links[a], links[b]))) in ADJACENT:
                continue
            pairs.append((a, b))

    def flt(x):
        # "1f" is not a float literal; the suffix needs a decimal point or an
        # exponent in front of it.
        t = f'{x:.9g}'
        if not any(ch in t for ch in '.eE'):
            t += '.0'
        return t + 'f'

    def vec(v):
        return '{' + ', '.join(flt(x) for x in v) + '}'

    L = ['// Generated by tools/gen_model.py -- do not edit.',
         '#pragma once', '',
         'namespace gripper_servo',
         '{', '',
         f'constexpr int kJoints = {len(MOTOR_ORDER)};',
         f'constexpr int kLinks = {len(links)};',
         f'constexpr int kCapsuleCount = {len(capsules)};', '',
         '/// Joint order everywhere in gripper_servo, and the interface names',
         '/// the controller claims. Motor 1..8.',
         'inline constexpr const char * kJointNames[kJoints] = {',
         '  ' + ', '.join(f'"{n}"' for n in MOTOR_ORDER) + '};', '',
         '// Link i+1 is driven by joint i; link 0 is the palm.',
         'struct JointModel',
         '{',
         '  int parent;         ///< parent link index',
         '  float origin[12];   ///< row-major 3x4, parent frame -> joint frame',
         '  float axis[3];',
         '  float lower;        ///< rad',
         '  float upper;',
         '};', '',
         'struct CapsuleModel',
         '{',
         '  int link;',
         '  float a[3];         ///< segment start, link frame, metres',
         '  float b[3];         ///< segment end',
         '  float radius;       ///< already shrunk, see SHRINK above',
         '};', '',
         'inline constexpr JointModel kJointModel[kJoints] = {']

    for n in MOTOR_ORDER:
        j = joints[n]
        M = j['origin']
        rows = [flt(M[r][c]) for r in range(3) for c in range(4)]
        L.append(f'  {{{index[j["parent"]]}, {{' + ', '.join(rows) + '}, '
                 f'{vec(j["axis"])}, {flt(j["lower"])}, {flt(j["upper"])}}},'
                 f'   // motor {MOTOR_ORDER.index(n) + 1}  {n}')

    L += ['};', '',
          '/// Links in an order where every parent precedes its children.',
          'inline constexpr int kFkOrder[kLinks] = {' +
          ', '.join(str(i) for i in order) + '};', '',
          'inline constexpr CapsuleModel kCapsules[kCapsuleCount] = {']
    for i, a, b, r in capsules:
        L.append(f'  {{{i}, {vec(a)}, {vec(b)}, {flt(r)}}},   // {links[i]}')

    watch = {tuple(sorted(p)) for p in pairs}
    L += ['};', '',
          '/// Symmetric; false where the links share a joint and always touch.',
          'inline constexpr bool kMonitored[kLinks][kLinks] = {']
    for a in range(len(links)):
        row = ['true ' if tuple(sorted((a, b))) in watch else 'false'
               for b in range(len(links))]
        L.append('  {' + ', '.join(row) + f'}},   // {links[a]}')
    L += ['};', '', '}  // namespace gripper_servo', '']

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    open(OUT, 'w').write('\n'.join(L))
    print(f'\n{len(capsules)} capsules, {len(pairs)} monitored pairs')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
