#!/usr/bin/env python3
"""Starts uros_node on the board over its serial console, and stops it on SIGTERM.

Stopping it with SIGINT is what makes it drop the torque first. A process that dies
any other way leaves the motors holding with nothing controlling them, so after it
has gone every motor is read back, and turned off if one is still on.

The port is only held while talking to the board, so a terminal can use it otherwise.
"""

import os
import select
import signal
import sys
import termios
import time

DEVICE = '/dev/board_console'
DIRECTORY = '/usr/bin/gripper'
MOTORS = range(1, 9)
# A bad wire reads as a flood of NUL bytes, not as silence.
MAX_NUL = 4096


class Console:

    def __enter__(self):
        self._fd = os.open(DEVICE, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self._fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        # Whatever a terminal left at the prompt, or running in the foreground.
        os.write(self._fd, b'\x03\n')
        time.sleep(0.5)
        termios.tcflush(self._fd, termios.TCIFLUSH)
        return self

    def __exit__(self, *_):
        os.close(self._fd)

    def run(self, command, timeout):
        """Output of `command` on the board, or None if it never finished."""
        # The shell echoes the command back, wrapped at 80 columns, so only what comes
        # between these two counts. Split so that the echo itself does not match.
        begin = f'__BEGIN_{time.monotonic_ns()}__'
        end = f'__END_{time.monotonic_ns()}__'
        line = f"echo {begin[:6]}''{begin[6:]}; {command}; echo {end[:6]}''{end[6:]}\n"
        os.write(self._fd, line.encode())

        output = b''
        nul = 0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self._fd], [], [], 0.2)
            if not ready:
                continue
            chunk = os.read(self._fd, 4096)
            nul += chunk.count(b'\0')
            if nul > MAX_NUL:
                return None
            output += chunk.replace(b'\0', b'')
            if end.encode() in output:
                text = output.decode(errors='replace').replace('\r', '')
                return text.split(begin)[-1].split(end)[0]
        return None


def stop(console):
    """True once uros_node has gone and every motor reads torque off."""
    text = console.run(
        f'cd {DIRECTORY} && killall -INT uros_node 2>/dev/null; i=0; '
        'while pidof uros_node >/dev/null && [ $i -lt 8 ]; do sleep 1; i=$((i+1)); done; '
        'if pidof uros_node >/dev/null; then echo STUCK; killall -KILL uros_node; sleep 1; fi',
        timeout=12)
    if text is None:
        return False
    if 'STUCK' in text:
        print('uros_node did not stop on SIGINT, killed it', flush=True)

    reads = '; '.join(f'./dxl_offline read {motor} 64 1' for motor in MOTORS)
    text = console.run(reads, timeout=6)
    if text is None:
        return False
    torque = [line.strip() for line in text.splitlines() if line.strip().isdigit()]
    print(f'torque after stopping: {" ".join(torque)}', flush=True)
    if len(torque) == len(MOTORS) and all(value == '0' for value in torque):
        return True

    print('a motor still has torque, or did not answer: turning all off', flush=True)
    return console.run('./dxl_offline torque all off', timeout=4) is not None


def start(console, agent, arguments):
    command = f'./uros_node --agent {agent} {arguments}'.strip()
    print(f'starting: {command}', flush=True)
    if console.run(
            f'cd {DIRECTORY} && (nohup {command} > /tmp/uros.log 2>&1 & echo $! > /tmp/uros.pid)',
            timeout=4) is None:
        return False
    time.sleep(4)
    text = console.run('kill -0 $(cat /tmp/uros.pid) 2>/dev/null && echo RUNNING; tail -5 /tmp/uros.log',
                       timeout=4)
    if text is None:
        return False
    print(text.strip(), flush=True)
    return 'RUNNING' in text


def main():
    agent = os.environ.get('AGENT_IP', '192.168.0.1')
    arguments = os.environ.get('UROS_ARGS', '')

    stopping = []
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))

    try:
        with Console() as console:
            if not stop(console) or not start(console, agent, arguments):
                print('no answer from the board console. Is it powered, and is another '
                      'terminal holding the port?', flush=True)
                return 1
    except (OSError, termios.error) as error:
        print(f'cannot use {DEVICE}: {error}', flush=True)
        return 1

    while not stopping:
        signal.pause()

    print('stopping uros_node', flush=True)
    try:
        with Console() as console:
            if stop(console):
                print('stopped, torque off', flush=True)
                return 0
    except (OSError, termios.error) as error:
        print(f'cannot use {DEVICE}: {error}', flush=True)
    print('COULD NOT CONFIRM THE TORQUE IS OFF. Check the board.', flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
