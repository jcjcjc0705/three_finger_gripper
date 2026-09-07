#!/usr/bin/env python3
"""Find which link pairs can touch inside the gripper's joint limits.

The output is the pair list gripper_servo has to check at run time. It is not a
lookup table: closure is not monotonic -- along a naive "curl everything
proportionally" path the index and thumb fingertips interpenetrate by up to
8.6 mm around 55% closure and come out the other side. Any table built by
walking that path measures the path, not the mechanism.

    python3 tools/selfcollision.py
"""

import collections
import os
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(HERE, 'src', 'three_finger_gripper_description')
MACRO = os.path.join(PKG, 'urdf', 'three_finger_gripper_macro.xacro')

CHAIN = {'index': ['A_1_Link', 'A_2_Link', 'A_3_Link'],
         'middle': ['B_1_Link_001', 'B_2_Link', 'B_3_Link'],
         'thumb': ['C_2_Link', 'C_3_Link_001'],
         'palm': ['base_link']}
FLEX = ['A_2_Joint', 'A_3_Joint', 'B_2_Joint', 'B_3_Joint', 'C_2_Joint', 'C_3_Joint']

# Links that share a joint always touch; they are not collisions.
ADJACENT = {tuple(sorted(p)) for p in [
    ('A_1_Link', 'A_2_Link'), ('A_2_Link', 'A_3_Link'),
    ('B_1_Link_001', 'B_2_Link'), ('B_2_Link', 'B_3_Link'),
    ('C_2_Link', 'C_3_Link_001'),
    ('base_link', 'A_1_Link'), ('base_link', 'B_1_Link_001'), ('base_link', 'C_2_Link')]}


def load():
    strip = lambda s: s.replace('${prefix}', '')
    joints = {}
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
        joints[strip(j.find('child').get('link'))] = dict(
            name=strip(j.get('name')), parent=strip(j.find('parent').get('link')), origin=M,
            axis=np.array([float(v) for v in j.find('axis').get('xyz').split()]),
            lower=float(lim.get('lower')), upper=float(lim.get('upper')))
    return joints


def fk(joints, link, q):
    T = np.eye(4)
    while link in joints:
        j = joints[link]
        a, th = j['axis'], q.get(j['name'], 0.0)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        R = np.eye(4)
        R[:3, :3] = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
        T = j['origin'] @ R @ T
        link = j['parent']
    return T


def main():
    joints = load()
    links = [l for c in CHAIN.values() for l in c]
    owner = {l: c for c, ls in CHAIN.items() for l in ls}
    lower = {joints[l]['name']: joints[l]['lower'] for l in joints}

    # One BVH per link, built once; each configuration only moves transforms.
    world = trimesh.collision.CollisionManager()
    for l in links:
        world.add_object(l, trimesh.load(f'{PKG}/meshes/{l}.stl', process=False))

    spreads = np.deg2rad(np.arange(-90, 91, 15.0))
    closures = np.linspace(0.0, 1.0, 11)
    seen = collections.Counter()
    total = 0

    for A1 in spreads:
        for B1 in spreads:
            for c in closures:
                q = {'A_1_Joint': A1, 'B_1_Joint': B1}
                q.update({j: c * lower[j] for j in FLEX})
                for l in links:
                    world.set_transform(l, fk(joints, l, q))
                total += 1
                _, names = world.in_collision_internal(return_names=True)
                for p in names:
                    k = tuple(sorted(p))
                    if k not in ADJACENT:
                        seen[k] += 1

    print(f'swept {total} configurations '
          f'({len(spreads)}x{len(spreads)} spread x {len(closures)} closure)\n')
    print(f'  {"pair":36}{"hits":>7}   chains')
    for (a, b), n in seen.most_common():
        print(f'  {a + " <-> " + b:36}{100.0 * n / total:6.1f}%   {owner[a]}/{owner[b]}')
    print(f'\n  {len(seen)} pairs need a run-time check')
    print('  Sampling is coarse, so treat this as the set of pairs to guard,')
    print('  not as evidence that any particular pose is safe.')
    return seen


if __name__ == '__main__':
    main()
