#!/usr/bin/env python3
"""Check the generated description still matches the USD it came from.

Solves each joint angle from the USD's own link poses and reports the residual.
A non-zero rotation residual means a joint frame was mis-composed -- the failure
mode that silently mirrors one finger onto the wrong side.

    python3 tools/verify_urdf.py
"""

import os
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from pxr import Usd, UsdGeom

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USDZ = os.path.join(HERE, 'cad', '3FingerGripper_Blue.usdz')
MACRO = os.path.join(HERE, 'src', 'three_finger_gripper_description',
                     'urdf', 'three_finger_gripper_macro.xacro')
ROOT = '/Root/ThreeFigV2ASM_Material/ThreeFigV2ASM'


def usd_world(stage, name):
    m = UsdGeom.Xformable(stage.GetPrimAtPath(f'{ROOT}/{name}')).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    return np.array([[m[r][c] for c in range(4)] for r in range(4)]).T


def rpy_R(x, y, z):
    cx, sx, cy, sy, cz, sz = np.cos(x), np.sin(x), np.cos(y), np.sin(y), np.cos(z), np.sin(z)
    return np.array([[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                     [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                     [-sy, cy * sx, cy * cx]])


def main():
    work = os.path.join(HERE, 'cad', '.extracted')
    os.makedirs(work, exist_ok=True)
    with zipfile.ZipFile(USDZ) as z:
        inner = z.namelist()[0]
        z.extract(inner, work)
    stage = Usd.Stage.Open(os.path.join(work, inner))

    strip = lambda s: s.replace('${prefix}', '')
    root = ET.parse(MACRO).getroot()
    ns = {'xacro': 'http://www.ros.org/wiki/xacro'}
    joints = [j for j in root.iter('joint') if j.get('type') == 'revolute']

    print(f'{"joint":12}{"angle":>9}{"trans":>10}{"rot":>9}   verdict')
    print('-' * 50)
    bad = 0
    for j in joints:
        o = j.find('origin')
        M = np.eye(4)
        M[:3, 3] = [float(v) for v in o.get('xyz').split()]
        M[:3, :3] = rpy_R(*[float(v) for v in o.get('rpy').split()])
        a = np.array([float(v) for v in j.find('axis').get('xyz').split()])

        parent = strip(j.find('parent').get('link'))
        child = strip(j.find('child').get('link'))
        D = np.linalg.inv(M) @ (np.linalg.inv(usd_world(stage, parent)) @ usd_world(stage, child))

        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        th = np.arctan2(np.trace(K.T @ D[:3, :3]) / 2, (np.trace(D[:3, :3]) - 1) / 2 + 1e-18)
        R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
        rot = np.degrees(np.arccos(np.clip((np.trace(R.T @ D[:3, :3]) - 1) / 2, -1, 1)))
        trans = np.linalg.norm(D[:3, 3]) * 1000

        ok = trans < 1.0 and rot < 1.0
        bad += not ok
        print(f'{strip(j.get("name")):12}{np.degrees(th):8.2f}°{trans:9.3f}mm{rot:8.3f}°   '
              f'{"ok" if ok else "MISMATCH"}')

    print()
    print('all joints reproduce the USD pose' if not bad else f'{bad} joint(s) mismatched')
    return 1 if bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
