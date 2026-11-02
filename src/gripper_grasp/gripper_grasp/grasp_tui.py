#!/usr/bin/env python3
"""See what the hand is doing, and tell it what to do next.

The grasps run on the board, and what it has been taught is stored there: this shows
its topics and sends it commands, so quitting changes nothing on the hand.

A new object: press n and name it, pick a grasp or add one, hold the object where the
fingers should meet it and press c, then g to show how far they close to get there.
After that, pick it and press a: the nearer the object comes, the further the hand
closes, so that they arrive together.

A new grasp: name it, press t for torque off, bend the fingers to the open pose and
press o, bend them to the closed pose and press k, press t for torque on, then enter.

    ros2 run gripper_grasp grasp_tui
"""

import curses
import math
import re
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from dynamixel_msgs.srv import SetTorque
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float32MultiArray, String

# Motor 1..8, the board's joint order.
JOINTS = ['thumb  prox', 'thumb  dist', 'index  prox', 'index  dist',
          'middle prox', 'middle dist', 'index  spread', 'middle spread']
FORM_COLUMNS = ['th prox', 'th dist', 'ix prox', 'ix dist', 'mi prox', 'mi dist',
                'ix sprd', 'mi sprd']
DIAG_FIELDS = 4          # temperature, voltage, hardware error, torque
STALE_AFTER = 1.0        # s without a message before a source is called stale
ASK_AGAIN_AFTER = 1.0    # s before asking for the catalog again
ANSWER_WITHIN = 3.0      # s for the board to answer a change, NAND write included
OBJECT_WIDTH = 33        # marks, "name -> grasp", calibration, a gap
GRASP_WIDTH = 18         # marks, name, where the fingers block the TOF, a gap
SPREAD_MOVED_DEG = 3.0   # a grasp has one spread, so its closed pose cannot have another
NAME = re.compile(r'[A-Za-z0-9_-]{1,23}\Z')
# A grasp name also works as a command. The board refuses these too, but only after
# the fingers have been taught: kCommandWords in grasp_store.cpp.
COMMAND_WORDS = ('arm', 'disarm', 'grasp', 'release', 'open', 'stop', 'calibrate', 'photo',
                 'catalog', 'measure', 'grasp_save', 'grasp_delete', 'object_save',
                 'object_delete')

MENU, NAMING, PICKING, FORM, CONFIRM, WAITING = range(6)
ENTER_KEYS = (curses.KEY_ENTER, ord('\n'), ord('\r'))
BACKSPACE_KEYS = (curses.KEY_BACKSPACE, 127, 8)
ESCAPE = 27


def parse_status(text):
    """Fields of a /gripper/status line, or None for its other kinds of line.

    The note is free text and comes last, so it takes the rest of the line.
    """
    if not text.startswith('phase='):
        return None
    head, _, note = text.partition(' note=')
    fields = dict(part.split('=', 1) for part in head.split() if '=' in part)
    fields['note'] = note
    return fields


def parse_catalog_row(text):
    """One row of the board's catalog: see catalog_row in grasp_store.hpp."""
    parts = text.split()
    if len(parts) < 3 or parts[0] != 'catalog':
        return None
    kind = parts[1]
    if kind == 'begin' and len(parts) == 3:
        return kind, parts[2]
    if kind == 'grasp' and len(parts) == 22:
        numbers = [float(v) for v in parts[4:]]
        return kind, {
            'name': parts[2], 'added': parts[3] == '1', 'spread': numbers[:2],
            'open': numbers[2:8], 'closed': numbers[8:14], 'current': numbers[14],
            'seconds': numbers[15], 'contacts': int(numbers[16]), 'blind': numbers[17],
        }
    if kind == 'object' and len(parts) == 6:
        return kind, (parts[2], parts[3], float(parts[4]), float(parts[5]))
    if kind == 'end' and len(parts) == 5:
        return kind, parts[2], int(parts[3]), int(parts[4])
    return None


class GraspTui(Node):

    def __init__(self):
        super().__init__('grasp_tui')
        self.status = {}
        self.joints = None
        self.goal = None
        self.diag = None
        self.range_m = None
        self.grasps = []
        self.objects = []
        self.reply = None
        self.message = ''
        self._seen = {}
        self._catalog_revision = None
        self._building = None
        self._asked = -ASK_AGAIN_AFTER

        self._command = self.create_publisher(String, '/gripper/command', 10)
        self._object = self.create_publisher(String, '/gripper/object', 10)
        self._torque = self.create_client(SetTorque, '/gripper_board/set_torque')

        self.create_subscription(String, '/gripper/status', self._on_status, 100)
        self.create_subscription(JointState, '/gripper/joint_states', self._on_joints, 10)
        self.create_subscription(Float32MultiArray, '/gripper/goal', self._on_goal, 10)
        self.create_subscription(Float32MultiArray, '/gripper/motor_diag', self._on_diag, 10)
        self.create_subscription(Range, '/tof/range', self._on_range, 10)

    def _mark(self, key):
        self._seen[key] = time.monotonic()

    def fresh(self, key):
        seen = self._seen.get(key)
        return seen is not None and time.monotonic() - seen < STALE_AFTER

    def _on_status(self, msg):
        text = msg.data
        if text.startswith('catalog '):
            self._on_catalog_row(parse_catalog_row(text))
        elif text.startswith('store '):
            verdict, _, detail = text[len('store '):].partition(' ')
            self.reply = (verdict == 'ok', detail)
        else:
            fields = parse_status(text)
            if fields is not None:
                self.status = fields
                self._mark('status')
                self._keep_catalog_current()

    def _keep_catalog_current(self):
        if self.status.get('rev') == self._catalog_revision:
            return
        now = time.monotonic()
        if now - self._asked >= ASK_AGAIN_AFTER:
            self._asked = now
            self.send('catalog')

    def _on_catalog_row(self, row):
        if row is None:
            return
        if row[0] == 'begin':
            self._building = ([], [])
        elif self._building is None:
            return
        elif row[0] == 'grasp':
            self._building[0].append(row[1])
        elif row[0] == 'object':
            self._building[1].append(row[1])
        elif row[0] == 'end':
            grasps, objects = self._building
            self._building = None
            if len(grasps) == row[2] and len(objects) == row[3]:
                self.grasps, self.objects = grasps, objects
                self._catalog_revision = row[1]

    def _on_joints(self, msg):
        self.joints = msg
        self._mark('joints')

    def _on_goal(self, msg):
        self.goal = msg.data
        self._mark('goal')

    def _on_diag(self, msg):
        self.diag = msg.data
        self._mark('diag')

    def _on_range(self, msg):
        self.range_m = msg.range
        self._mark('tof')

    def send(self, command):
        self._command.publish(String(data=command))

    def phase(self):
        return self.status.get('phase', '') if self.fresh('status') else ''

    def busy(self):
        return self.phase() in ('following', 'closing', 'holding', 'measuring')

    def choose_grasp(self, name):
        if self.busy():
            self.message = 'release first'
            return
        self.send(name)

    def choose_object(self, name, grasp):
        if self.busy():
            self.message = 'release first'
            return
        # The board ignores an object named twice, so a recogniser can repeat itself.
        # Picking it here again means this object's grasp, whatever has happened since.
        if self.status.get('object') == name:
            self.send(grasp)
        else:
            self._object.publish(String(data=name))

    def toggle_arm(self):
        self.send('disarm' if self.phase() in ('armed', 'following') else 'arm')

    def set_torque(self, enable):
        if not self._torque.service_is_ready():
            self.message = 'board_bridge is not running'
            return
        future = self._torque.call_async(SetTorque.Request(id=0, enable=enable))
        future.add_done_callback(self._on_torque)
        self.message = f'turning torque {"on" if enable else "off"}'

    def _on_torque(self, future):
        result = future.result()
        self.message = result.message if result is not None else 'no answer from board_bridge'


class GraspForm:
    """The numbers that make a grasp: taught by hand, typed, or an added grasp's own."""

    ROWS = [[f'open{i}' for i in range(6)] + ['index', 'middle'],
            [f'closed{i}' for i in range(6)],
            ['current', 'seconds', 'contacts']]

    def __init__(self, name, base=None):
        self.name = name
        self.values = dict.fromkeys(self.ROWS[0] + self.ROWS[1], '')
        self.values.update(current='0', seconds='0', contacts='0')
        if base is not None:
            self.values.update(index=f'{base["spread"][0]:.1f}', middle=f'{base["spread"][1]:.1f}',
                               current=f'{base["current"]:.0f}', seconds=f'{base["seconds"]:.1f}',
                               contacts=str(base['contacts']))
            for i in range(6):
                self.values[f'open{i}'] = f'{base["open"][i]:.1f}'
                self.values[f'closed{i}'] = f'{base["closed"][i]:.1f}'
        self.row, self.column = 0, 0
        self.fresh = True

    def field(self):
        return self.ROWS[self.row][self.column]

    def move(self, rows, columns):
        self.row = (self.row + rows) % len(self.ROWS)
        self.column = (self.column + columns) % len(self.ROWS[self.row]) if columns else min(
            self.column, len(self.ROWS[self.row]) - 1)
        self.fresh = True

    def type(self, character):
        if not re.fullmatch(r'[0-9.\-]', character):
            return
        current = '' if self.fresh else self.values[self.field()]
        self.values[self.field()] = (current + character)[:8]
        self.fresh = False

    def erase(self):
        self.values[self.field()] = self.values[self.field()][:-1]
        self.fresh = False

    def copy_pose(self, position, row):
        """The fingers' measured pose into the open or closed row, in degrees."""
        for i in range(6):
            self.values[f'{row}{i}'] = f'{math.degrees(position[i]) + 0.0:.1f}'
        if row == 'open':
            self.values['index'] = f'{math.degrees(position[6]) + 0.0:.1f}'
            self.values['middle'] = f'{math.degrees(position[7]) + 0.0:.1f}'

    def spread_moved(self, position):
        """True if the spread joints are not where the open pose has them."""
        try:
            return (abs(math.degrees(position[6]) - float(self.values['index'])) > SPREAD_MOVED_DEG
                    or abs(math.degrees(position[7]) - float(self.values['middle']))
                    > SPREAD_MOVED_DEG)
        except ValueError:
            return False

    def missing(self):
        """'open' or 'closed' if that pose has not been filled in yet."""
        if any(not self.values[field] for field in self.ROWS[0]):
            return 'open'
        if any(not self.values[field] for field in self.ROWS[1]):
            return 'closed'
        return None

    def command(self):
        """The grasp_save line, or raises ValueError saying what is wrong."""
        missing = self.missing()
        if missing is not None:
            raise ValueError(f'no {missing} pose yet: press {"o" if missing == "open" else "k"}')
        fields = ['index', 'middle'] + self.ROWS[0][:6] + self.ROWS[1] + ['current', 'seconds']
        numbers = []
        for field in fields:
            try:
                numbers.append(f'{float(self.values[field]):.2f}')
            except ValueError:
                raise ValueError(f'{field} is not a number') from None
        if not self.values['contacts'].isdigit():
            raise ValueError('contacts is not a whole number')
        return ' '.join(['grasp_save', self.name, *numbers, self.values['contacts']])


class Ui:
    """What the screen is doing: browsing, or part way through teaching something."""

    def __init__(self, node):
        self.node = node
        self.mode = MENU
        self.column, self.row = 0, 0
        self.text = ''
        self.naming = 'object'   # or 'grasp'
        self.target = None       # the object being set up
        self.form = None
        self.question = None     # (text, what to do on y, mode to return to on n)
        self.waiting_for = None  # 'grasp', 'object' or 'delete'
        self.back_to = MENU
        self.deadline = 0.0
        self.follow = None       # object to put the cursor on once it appears

    def lists(self):
        return (self.node.objects, self.node.grasps)

    def size(self):
        extra = 1 if self.mode == PICKING else 0
        return len(self.lists()[self.column]) + extra

    def highlighted_grasp(self):
        grasps = self.node.grasps
        return grasps[min(self.row, len(grasps) - 1)] if self.column == 1 and grasps else None

    def prompt(self):
        if self.mode == NAMING:
            after = self.node.message or 'enter next, esc cancel'
            return f'New {self.naming}, name: {self.text}_   {after}'
        if self.mode == PICKING:
            return f'Grasp for {self.target}: pick one or add one, enter. Esc cancels'
        if self.mode == CONFIRM:
            return f'{self.question[0]} y/n'
        if self.mode == WAITING:
            return 'saving to the gripper...'
        return None

    def ask(self, waiting_for, command, back_to):
        self.node.reply = None
        self.node.send(command)
        self.waiting_for = waiting_for
        self.back_to = back_to
        self.deadline = time.monotonic() + ANSWER_WITHIN
        self.mode = WAITING

    def poll(self):
        if self.follow is not None:
            for i, row in enumerate(self.node.objects):
                if row[0] == self.follow:
                    self.column, self.row, self.follow = 0, i, None
                    break
        if self.mode != WAITING:
            return
        reply = self.node.reply
        if reply is None:
            if time.monotonic() > self.deadline:
                self.node.message = 'no answer from the board, it may not be saved'
                self.mode = self.back_to
            return
        ok, detail = reply
        self.node.reply = None
        if not ok:
            self.node.message = detail
            self.mode = self.back_to
        elif self.waiting_for == 'grasp' and self.target is not None:
            self.ask('object', f'object_save {self.target} {self.form.name}', MENU)
        elif self.waiting_for == 'grasp':
            self.node.message = f'{self.form.name} saved'
            self.mode = MENU
        elif self.waiting_for == 'object':
            # Without torque the hand cannot go to the preshape, so the object is not in hand yet.
            self.node.message = (
                f'{self.target}: hold it where the hand should close, then press c'
                if self.node.status.get('torque') == 'on'
                else f'{self.target} saved: t for torque on, then enter on it')
            self.follow = self.target
            self.target = None
            self.mode = MENU
        else:
            self.node.message = 'deleted'
            self.column, self.row = self.column, 0
            self.mode = MENU

    def key(self, key):
        """Handles one key. False means quit."""
        if key != curses.ERR:
            self.node.message = ''
        handler = {MENU: self._on_menu, NAMING: self._on_naming, PICKING: self._on_picking,
                   FORM: self._on_form, CONFIRM: self._on_confirm}.get(self.mode)
        if handler is not None and key != curses.ERR:
            return handler(key) is not False
        return True

    def _move(self, key):
        size = self.size()
        if key == curses.KEY_UP and size:
            self.row = (self.row - 1) % size
        elif key == curses.KEY_DOWN and size:
            self.row = (self.row + 1) % size

    def _on_menu(self, key):
        node = self.node
        size = self.size()
        if key == ord('q'):
            return False
        if key in (curses.KEY_UP, curses.KEY_DOWN):
            self._move(key)
        elif key in (curses.KEY_LEFT, curses.KEY_RIGHT):
            self.column = 1 - self.column
            self.row = min(self.row, max(len(self.lists()[self.column]) - 1, 0))
        elif key in ENTER_KEYS and self.row < size:
            if self.column == 0:
                row = node.objects[self.row]
                node.choose_object(row[0], row[1])
            else:
                node.choose_grasp(node.grasps[self.row]['name'])
        elif key == ord('g'):
            node.send('grasp')
        elif key == ord('a'):
            node.toggle_arm()
        elif key == ord('r'):
            node.send('release')
        elif key == ord('s'):
            node.send('stop')
        elif key == ord('c'):
            node.send('calibrate')
        elif key == ord('t'):
            self._toggle_torque()
        elif key == ord('b'):
            self.question = ('Measure where the fingers block the TOF: the empty hand closes.',
                             lambda back: (node.send('measure'), setattr(self, 'mode', back)),
                             MENU)
            self.mode = CONFIRM
        elif key == ord('n'):
            self._new()
        elif key == ord('e') and self.row < size:
            self._edit()
        elif key == ord('d') and self.row < size:
            self._delete()
        return True

    def _toggle_torque(self):
        node = self.node
        if not node.fresh('status'):
            node.message = 'no data from the board'
        elif node.status.get('torque') != 'on':
            node.set_torque(True)
        elif node.busy():
            # This is the one thing here that lets go of what is in the hand.
            self.question = ('Holding something: torque off drops it.',
                             lambda back: (node.set_torque(False), setattr(self, 'mode', back)),
                             self.mode)
            self.mode = CONFIRM
        else:
            node.set_torque(False)

    def _start_naming(self, kind):
        self.naming = kind
        self.text = ''
        self.mode = NAMING

    def _new(self):
        if self.column == 1:
            self.target = None
            self._start_naming('grasp')
        elif self.node.busy():
            self.node.message = 'release first'
        else:
            self._start_naming('object')

    def _edit(self):
        if self.column == 0:
            if self.node.busy():
                self.node.message = 'release first'
                return
            self.target = self.node.objects[self.row][0]
            self._start_picking()
        elif self.highlighted_grasp()['added']:
            self.target = None
            grasp = self.highlighted_grasp()
            self.form = GraspForm(grasp['name'], grasp)
            self.mode = FORM
        else:
            self.node.message = 'built in, cannot be changed: n adds your own'

    def _delete(self):
        if self.column == 0:
            name = self.node.objects[self.row][0]
            self.question = (f'Delete object {name}?',
                             lambda back: self.ask('delete', f'object_delete {name}', back), MENU)
            self.mode = CONFIRM
        elif self.highlighted_grasp()['added']:
            name = self.highlighted_grasp()['name']
            self.question = (f'Delete grasp {name}?',
                             lambda back: self.ask('delete', f'grasp_delete {name}', back), MENU)
            self.mode = CONFIRM
        else:
            self.node.message = 'built in, cannot be deleted'

    def _start_picking(self):
        self.column, self.row = 1, 0
        self.mode = PICKING

    def _named(self):
        text = self.text
        if not NAME.match(text):
            self.node.message = 'name: up to 23 letters, digits, _ or -'
        elif self.naming == 'object':
            if any(row[0] == text for row in self.node.objects):
                self.node.message = f'{text} already exists: e changes its grasp'
            else:
                self.target = text
                self._start_picking()
        elif text in COMMAND_WORDS:
            self.node.message = f'{text} is a command, pick another name'
        elif any(grasp['name'] == text for grasp in self.node.grasps):
            self.node.message = f'{text} already exists: e edits your own grasps'
        else:
            self.form = GraspForm(text)
            self.mode = FORM

    def _on_naming(self, key):
        if key == ESCAPE:
            self.mode = PICKING if self.naming == 'grasp' and self.target is not None else MENU
        elif key in ENTER_KEYS:
            self._named()
        elif key in BACKSPACE_KEYS:
            self.text = self.text[:-1]
        elif 0 <= key < 256 and re.fullmatch('[A-Za-z0-9_-]', chr(key)):
            self.text = (self.text + chr(key))[:23]

    def _on_picking(self, key):
        if key == ESCAPE:
            self.target = None
            self.column, self.row = 0, 0
            self.mode = MENU
        elif key in (curses.KEY_UP, curses.KEY_DOWN):
            self._move(key)
        elif key in ENTER_KEYS:
            if self.row < len(self.node.grasps):
                grasp = self.node.grasps[self.row]['name']
                self.ask('object', f'object_save {self.target} {grasp}', MENU)
            else:
                self._start_naming('grasp')

    def _on_form(self, key):
        form = self.form
        if key == ESCAPE:
            self.mode = PICKING if self.target is not None else MENU
        elif key == curses.KEY_UP:
            form.move(-1, 0)
        elif key == curses.KEY_DOWN:
            form.move(1, 0)
        elif key == curses.KEY_LEFT:
            form.move(0, -1)
        elif key in (curses.KEY_RIGHT, ord('\t')):
            form.move(0, 1)
        elif key in BACKSPACE_KEYS:
            form.erase()
        elif key in ENTER_KEYS:
            try:
                self.ask('grasp', form.command(), FORM)
            except ValueError as error:
                self.node.message = str(error)
        elif key == ord('t'):
            self._toggle_torque()
        elif key in (ord('o'), ord('k')):
            self._save_pose('open' if key == ord('o') else 'closed')
        elif 0 <= key < 256:
            form.type(chr(key))

    def _save_pose(self, row):
        node = self.node
        if node.joints is None or not node.fresh('joints'):
            node.message = 'no joint states from the board'
            return
        self.form.copy_pose(node.joints.position, row)
        self.form.fresh = True
        node.message = f'{row} pose saved'
        if row == 'closed' and self.form.spread_moved(node.joints.position):
            node.message = 'closed pose saved, but the spread moved: the grasp keeps the open one'

    def _on_confirm(self, key):
        text, on_yes, back = self.question
        if key == ord('y'):
            on_yes(FORM if back == FORM else MENU)
        elif key in (ord('n'), ESCAPE):
            self.mode = back


def put(win, y, x, text, attr=0):
    height, width = win.getmaxyx()
    if 0 <= y < height and x < width:
        try:
            win.addnstr(y, x, text, width - x - 1, attr)
        except curses.error:
            pass


def trigger_line(node, status):
    if not node.fresh('tof'):
        shown = '--'
    elif math.isfinite(node.range_m):
        shown = f'{node.range_m * 1000:.0f} mm'
    else:
        shown = 'nothing in range'
    if 'touch' not in status:
        return f'TOF {shown}'
    blind = float(status['blind']) * 100
    text = (f'TOF {shown}   follows {float(status["start"]):.0f}-{float(status["touch"]):.0f} mm'
            f'   touches at {float(status["touch_closure"]) * 100:.0f}%   blind from {blind:.0f}%')
    if status.get('phase') == 'armed' and not status.get('servo', '').startswith('holding'):
        text += '   settling'
    return text


def banner(node, status):
    if not node.fresh('status'):
        return 'NO DATA FROM THE BOARD. Is uros_node running?'
    if status.get('fault') == 'self_collision':
        return 'SELF-COLLISION POSE, TORQUE OFF. Move the fingers apart by hand, then press t.'
    if status.get('store') == 'failed':
        return 'COULD NOT SAVE TO THE GRIPPER. Changes will be lost at power off.'
    if status.get('hw_err', '0') != '0':
        return 'MOTOR HARDWARE ERROR, see hw below. Reboot it: dxl_cli reboot <motor>'
    return ''


def hint(node, status):
    """What is missing from the object or grasp in hand, if anything."""
    for name, _, touch_mm, touch_closure in node.objects:
        if name != status.get('object'):
            continue
        if touch_mm == 0:
            return f'{name} has no distance yet: hold it where the fingers should meet it, press c'
        if touch_closure == 0:
            return f'{name} has no touch point yet: with it in place, press c, then g'
    for grasp in node.grasps:
        if grasp['name'] == status.get('grasp') and grasp['blind'] < 0:
            return f'{grasp["name"]}: where it blocks the TOF is not measured. Hand empty, press b'
    return ''


def draw_header(win, ui, status):
    node = ui.node
    put(win, 0, 0, ' grasp '.ljust(win.getmaxyx()[1] - 1), curses.A_REVERSE)
    closure = float(status.get('closure', 0)) * 100
    put(win, 1, 1, f'{status.get("phase", "?"):<9}', curses.A_BOLD)
    put(win, 1, 11, f'grasp {status.get("grasp", "?")}   object {status.get("object", "?")}'
                    f'   closure {closure:.0f}%   servo {status.get("servo", "?")}'
                    f'   contacts {status.get("contacts", "?")}')
    put(win, 2, 1, trigger_line(node, status))
    put(win, 3, 1, f'torque {status.get("torque", "?")}   loop {status.get("loop", "?")} Hz'
                   f'   bus_err {status.get("bus_err", "?")}   tof_err {status.get("tof_err", "?")}'
                   f'   reach {status.get("reach", "?")}')
    warning = banner(node, status)
    if warning:
        put(win, 4, 1, warning, curses.A_REVERSE)
    else:
        put(win, 4, 1, status.get('note', ''), curses.A_BOLD)
    prompt = ui.prompt()
    if prompt is not None:
        put(win, 5, 1, prompt, curses.A_BOLD)
    else:
        put(win, 5, 1, node.message or hint(node, status))


def draw_menus(win, ui, status):
    node = ui.node
    top = 6
    put(win, top, 1, f'  {"Object -> grasp":<19}{"calibrate":>11}', curses.A_UNDERLINE)
    put(win, top, 1 + OBJECT_WIDTH, f'  {"Grasp":<10}{"blind":>5}', curses.A_UNDERLINE)
    if not node.grasps:
        put(win, top + 1, 1, 'asking the board...')
    for i, (name, grasp, touch_mm, touch_closure) in enumerate(node.objects):
        here = ui.column == 0 and ui.row == i
        now = '*' if status.get('object') == name else ' '
        shown = '-'
        if touch_mm > 0:
            shown = f'{touch_mm:.0f}mm' + (f' {touch_closure * 100:.0f}%' if touch_closure > 0 else '')
        put(win, top + 1 + i, 1,
            f'{">" if here else " "}{now}{f"{name} -> {grasp}"[:19]:<19}{shown:>11}',
            curses.A_REVERSE if here else 0)
    for i, grasp in enumerate(node.grasps):
        here = ui.column == 1 and ui.row == i
        now = '*' if status.get('grasp') == grasp['name'] else ' '
        blind = f'{grasp["blind"] * 100:.0f}%' if grasp['blind'] >= 0 else '-'
        put(win, top + 1 + i, 1 + OBJECT_WIDTH,
            f'{">" if here else " "}{now}{grasp["name"][:10]:<10}{blind:>5}',
            curses.A_REVERSE if here else 0)
    rows = len(node.grasps)
    if ui.mode == PICKING:
        here = ui.row == rows
        put(win, top + 1 + rows, 1 + OBJECT_WIDTH, f'{">" if here else " "} + new grasp',
            curses.A_REVERSE if here else 0)
        rows += 1

    left = 1 + OBJECT_WIDTH + GRASP_WIDTH
    keys = ['arrows pick  enter preshape',
            'g close now  a follow in',
            'r open       s stop, hold',
            'c calibrate  t torque',
            'n new  e edit  d delete',
            'b measure blind  q quit']
    put(win, top, left, 'Keys', curses.A_UNDERLINE)
    for i, text in enumerate(keys):
        put(win, top + 1 + i, left, text)
    return top + 1 + max(rows, len(node.objects), len(keys))


def number(value, width, decimals):
    # Rounds first, so that nothing prints as -0.0.
    return f'{round(value, decimals) + 0.0:>{width}.{decimals}f}'


def draw_joints(win, node, status, top):
    put(win, top, 1, f'{"":<14}{"goal":>7}{"now":>7}{"diff":>7}{"current":>8}{"limit":>6}'
                     f'{"touch":>6}{"trq":>4}{"temp":>5}{"volt":>5}{"hw":>4}', curses.A_UNDERLINE)
    position = node.joints.position if node.joints and node.fresh('joints') else None
    effort = node.joints.effort if node.joints and node.fresh('joints') else None
    active = node.goal is not None and node.fresh('goal') and node.goal[-1] > 0.5
    diag = node.diag if node.diag is not None and node.fresh('diag') else None
    stalled = status.get('stalled', '')
    count = len(JOINTS)
    for i, name in enumerate(JOINTS):
        goal = math.degrees(node.goal[i]) if active else None
        now = math.degrees(position[i]) if position is not None else None
        text = f'{name:<14}' + (number(goal, 7, 1) if active else f'{"-":>7}')
        text += number(now, 7, 1) if now is not None else f'{"-":>7}'
        text += number(goal - now, 7, 1) if active and now is not None else f'{"-":>7}'
        text += number(effort[i], 8, 0) if effort is not None else f'{"-":>8}'
        text += number(node.goal[count + i], 6, 0) if active else f'{"-":>6}'
        touching = i < len(stalled) and stalled[i] == 'X'
        text += f'{"X" if touching else "":>6}'
        if diag is not None:
            row = diag[i * DIAG_FIELDS:(i + 1) * DIAG_FIELDS]
            text += (f'{"on" if row[3] > 0.5 else "off":>4}{row[0]:>5.0f}'
                     f'{row[1]:>5.1f}{row[2]:>4.0f}')
        put(win, top + 1 + i, 1, text, curses.A_BOLD if touching else 0)


def teach_step(form, status):
    """What to do next to teach this grasp by hand."""
    limp = status.get('torque') != 'on'
    missing = form.missing()
    if missing is None:
        return 'Press t for torque on, then enter saves' if limp else 'Enter saves'
    if not limp:
        return f'Press t for torque off, then bend the fingers to the {missing} pose'
    return f'Bend the fingers to the {missing} pose, press {"o" if missing == "open" else "k"}'


def draw_form(win, node, form, status, top):
    left = 8

    def field(y, column, name):
        here = form.field() == name
        shown = form.values[name] or '-'
        put(win, y, left + column * 8, f'{shown:>7}', curses.A_REVERSE if here else 0)

    put(win, top, 1, f'Grasp {form.name}, in degrees', curses.A_UNDERLINE)
    put(win, top + 1, 1, teach_step(form, status), curses.A_BOLD)
    for i, name in enumerate(FORM_COLUMNS):
        put(win, top + 2, left + i * 8, f'{name:>7}')
    put(win, top + 3, 1, 'now')
    if node.joints is not None and node.fresh('joints'):
        for i, value in enumerate(node.joints.position[:8]):
            put(win, top + 3, left + i * 8, number(math.degrees(value), 7, 1))
    put(win, top + 4, 1, 'open')
    for i, name in enumerate(form.ROWS[0]):
        field(top + 4, i, name)
    put(win, top + 5, 1, 'closed')
    for i, name in enumerate(form.ROWS[1]):
        field(top + 5, i, name)
    put(win, top + 6, 1, 'current')
    field(top + 6, 0, 'current')
    put(win, top + 6, 17, 'seconds')
    field(top + 6, 2, 'seconds')
    put(win, top + 6, 32, 'contacts')
    field(top + 6, 4, 'contacts')
    put(win, top + 6, 49, '0 keeps the default')
    put(win, top + 7, 1, 'o, k save where the fingers are. Arrows, digits edit a number. Esc cancels')


def run(win, node):
    curses.curs_set(0)
    curses.set_escdelay(25)
    win.timeout(100)
    ui = Ui(node)
    while True:
        status = node.status if node.fresh('status') else {}
        ui.poll()
        win.erase()
        draw_header(win, ui, status)
        bottom = draw_menus(win, ui, status)
        if ui.mode == FORM or (ui.mode == WAITING and ui.waiting_for == 'grasp'):
            draw_form(win, node, ui.form, status, bottom)
        else:
            draw_joints(win, node, status, bottom)
        win.refresh()
        if not ui.key(win.getch()):
            return


def main():
    rclpy.init()
    node = GraspTui()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        curses.wrapper(run, node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        spin.join(timeout=2.0)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
