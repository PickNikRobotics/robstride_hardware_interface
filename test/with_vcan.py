#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Run qualification with a real vcan interface, never a physical CAN device.

Use in a dedicated network namespace with CAP_NET_ADMIN. No iproute2 or
third-party Python package is needed. A missing kernel module or permission is
an error, not a skip. Only an interface created by this process is removed.
"""

import errno
import os
import socket
import struct
import subprocess
import sys

# Linux rtnetlink ABI: RTM_{NEW,DEL,GET}LINK and ifinfomsg/rtattr.
NEWLINK, DELLINK, GETLINK = 16, 17, 18
REQUEST, ACK, EXCL, CREATE = 1, 4, 0x200, 0x400
IFLA_IFNAME, IFLA_LINKINFO, IFLA_INFO_KIND = 3, 18, 1
IFF_UP = 1


def attribute(kind, payload):
    length = 4 + len(payload)
    return struct.pack('HH', length, kind) + payload + bytes((-length) % 4)


def attributes(payload):
    while len(payload) >= 4:
        length, kind = struct.unpack_from('HH', payload)
        if length < 4 or length > len(payload):
            raise RuntimeError('malformed rtnetlink attribute')
        yield kind & 0x3FFF, payload[4:length]
        payload = payload[(length + 3) & ~3:]


def request(kind, flags, payload):
    with socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, socket.NETLINK_ROUTE) as net:
        net.settimeout(3)
        net.bind((0, 0))
        net.send(struct.pack('IHHII', 16 + len(payload), kind, flags | REQUEST, 1, 0) + payload)
        while True:
            response = net.recv(65536)
            while len(response) >= 16:
                length, msg_kind, _, sequence, _ = struct.unpack_from('IHHII', response)
                if length < 16 or length > len(response):
                    raise RuntimeError('malformed rtnetlink reply')
                body = response[16:length]
                if sequence == 1:
                    if msg_kind == 2:  # NLMSG_ERROR, including successful ACK.
                        error = struct.unpack_from('i', body)[0]
                        if error:
                            raise OSError(-error, os.strerror(-error))
                        return b''
                    if msg_kind == NEWLINK:
                        return body
                response = response[(length + 3) & ~3:]


def link_message(index=0, flags=0, change=0):
    return struct.pack('BBHiII', socket.AF_UNSPEC, 0, 0, index, flags, change)


def virtual_kind(name):
    try:
        index = socket.if_nametoindex(name)
    except OSError:
        return None
    body = request(GETLINK, 0, link_message(index))
    for kind, payload in attributes(body[16:]):
        if kind == IFLA_LINKINFO:
            for nested_kind, nested in attributes(payload):
                if nested_kind == IFLA_INFO_KIND:
                    return nested.rstrip(b'\0').decode('ascii')
    return 'unknown'


def create(name):
    if not name.startswith('vcan') or len(name.encode()) >= 16:
        raise ValueError('qualification interface must have a vcan-prefixed Linux name')
    kind = virtual_kind(name)
    if kind is not None and kind != 'vcan':
        raise RuntimeError(f'refusing non-virtual interface {name!r}: kind={kind!r}')
    created = kind is None
    if created:
        info = attribute(IFLA_INFO_KIND, b'vcan\0')
        payload = link_message() + attribute(IFLA_IFNAME, name.encode() + b'\0')
        payload += attribute(IFLA_LINKINFO, info)
        request(NEWLINK, ACK | EXCL | CREATE, payload)
    try:
        request(NEWLINK, ACK, link_message(socket.if_nametoindex(name), IFF_UP, IFF_UP))
        if virtual_kind(name) != 'vcan':
            raise RuntimeError('vcan creation did not produce virtual transport')
    except BaseException:
        if created:
            remove(name)
        raise
    return created


def remove(name):
    if virtual_kind(name) != 'vcan':
        raise RuntimeError(f'refusing to remove non-vcan interface {name!r}')
    request(DELLINK, ACK, link_message(socket.if_nametoindex(name)))


def main():
    command = sys.argv[1:]
    if command[:1] == ['--']:
        command = command[1:]
    if not command:
        raise ValueError('usage: with_vcan.py -- COMMAND [ARGS...]')
    name = os.environ.get('ROBSTRIDE_TEST_CAN_IFACE', 'vcan0')
    owned = create(name)
    try:
        environment = dict(os.environ, ROBSTRIDE_TEST_CAN_IFACE=name, ROBSTRIDE_REQUIRE_VCAN='1')
        print(f'qualification transport: {name} (kind=vcan)', flush=True)
        return subprocess.call(command, env=environment)
    finally:
        if owned:
            remove(name)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError) as error:
        print(f'vcan qualification failed: {error}', file=sys.stderr)
        if isinstance(error, OSError) and error.errno == errno.EPERM:
            print('requires CAP_NET_ADMIN in an isolated network namespace', file=sys.stderr)
        sys.exit(2)
