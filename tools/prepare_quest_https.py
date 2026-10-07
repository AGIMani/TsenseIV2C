#!/usr/bin/env python3
"""Prepare certificates for the native Ubuntu web workbench; print its LAN IP."""
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tracking'))
from webxr import ensure_webxr_certificate


def lan_address():
    interfaces = json.loads(subprocess.check_output(['ip', '-j', '-4', 'address', 'show', 'up']))
    assigned = {info['local'] for interface in interfaces for info in interface['addr_info']
                if info.get('scope') == 'global'}
    override = os.environ.get('HAND_QUEST_ADDRESS', '').strip()
    if override:
        address = ipaddress.IPv4Address(override)
        if address.is_loopback or address.is_unspecified or override not in assigned:
            raise ValueError('HAND_QUEST_ADDRESS 必须是本机已配置的局域网 IPv4 地址')
        return override
    # Use physical Ethernet/Wi-Fi interfaces, not a VPN/TUN default route.
    candidates = [info['local'] for interface in interfaces
                  if (Path('/sys/class/net') / interface['ifname'] / 'device').exists()
                  for info in interface['addr_info'] if info.get('scope') == 'global']
    routes = json.loads(subprocess.check_output(['ip', '-j', '-4', 'route', 'show', 'default']))
    for route in sorted(routes, key=lambda route: route.get('metric', 0)):
        if route.get('prefsrc') in candidates:
            return route['prefsrc']
    if len(candidates) == 1:
        return candidates[0]
    print('未确定局域网地址，工作台仍可本机使用。Quest 联网后请重新启动；'
          '多网卡可设置 HAND_QUEST_ADDRESS。', file=sys.stderr)
    return ''


def main():
    address = lan_address()
    directory = ROOT / 'runtime/ubuntu/quest-tls'
    os.umask(0o077)
    ensure_webxr_certificate(directory / 'server.crt', directory / 'server.key',
                             advertise_hosts=(address,) if address else ())
    print(f'Quest HTTPS 信任证书：{directory / "quest_webxr_ca.crt"}', file=sys.stderr)
    print(address)


if __name__ == '__main__':
    main()
