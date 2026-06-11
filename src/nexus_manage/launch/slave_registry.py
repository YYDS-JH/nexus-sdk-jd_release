# Copyright (c) 2025 by Infra Embedded, All Rights Reserved.
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
"""Slave registry helpers for multi-slave teleop launch."""

import socket

import yaml


def get_local_ips():
    """Return a set of local non-loopback IPv4 addresses."""
    ips = set()
    # Primary: walk network interfaces (robust when hostname resolves to 127.x)
    try:
        import netifaces
        for iface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(iface).get(netifaces.AF_INET, [])
            for addr in addrs:
                ip = addr.get('addr', '')
                if ip and not ip.startswith('127.'):
                    ips.add(ip)
    except ImportError:
        pass

    # Fallback: resolve hostname
    if not ips:
        try:
            for info in socket.getaddrinfo(socket.gethostname(), None):
                ip = info[4][0]
                if not ip.startswith('127.') and ':' not in ip:
                    ips.add(ip)
        except socket.gaierror:
            pass
    return ips


def load_registry(path):
    """
    Parse a slave_registry.yaml file and return the list of slave entries.

    Returns an empty list if the file does not exist, so the caller can
    fall back to legacy single-slave behavior.
    """
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        return []
    if data is None:
        return []
    return data.get('slaves', [])


def generate_peers_xml(slaves, local_count, remote_count, port_base,
                       local_ip='127.0.0.1'):
    """
    Generate CycloneDDS Peers XML for local nodes and all slave IPs.

    Each slave gets ``remote_count`` Peer entries at its own IP address
    using the port_base + i scheme.
    """
    lines = []
    for i in range(local_count):
        lines.append(
            f'        <Peer Address="{local_ip}:{port_base + i}"/>')
    for slave in slaves:
        for i in range(remote_count):
            lines.append(
                f'        <Peer Address="{slave["ip"]}:{port_base + i}"/>')
    return '\n'.join(lines)


def detect_role(slaves, local_ip):
    """Return "slave" if local_ip matches a slave entry, otherwise "master"."""
    for slave in slaves:
        if slave.get('ip') == local_ip:
            return 'slave'
    return 'master'


def find_slave_by_ip(slaves):
    """
    Auto-detect which slave this machine is by matching local IPs.

    Returns the matching slave entry dict, or None if no match.
    """
    local_ips = get_local_ips()
    for slave in slaves:
        if slave.get('ip') in local_ips:
            return slave
    return None
