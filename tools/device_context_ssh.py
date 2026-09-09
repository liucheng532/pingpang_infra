"""SSH transport; passwords stay in memory and are read only from root AGENTS.md."""
import os
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HOSTS = {
    'workstation': ('odl@172.16.3.126', 'odl@172.16.3.126', '/home/odl/codebase'),
    'right': ('unitree@172.16.4.66', 'unitree@172.16.4.66_right', '/home/unitree/haoran'),
    'left': ('unitree@172.16.4.198', 'unitree@172.16.4.198_left', '/home/unitree/haoran'),
}
SSH_OPTIONS = ['-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=10',
               '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=3']


def authenticated(args, **kwargs):
    host = kwargs.pop('host')
    if host not in [v[0] for v in HOSTS.values()]:
        raise ValueError('Host not in the device allowlist')
    match = re.search(r'\|[^\n]*`ssh ' + re.escape(host) + r'`\s*\|\s*`([^`]+)`',
                      (ROOT / 'AGENTS.md').read_text())
    if not match:
        raise RuntimeError('Missing credential in root AGENTS.md')
    read_fd, write_fd = os.pipe()
    try:
        os.write(write_fd, (match.group(1) + '\n').encode())
    finally:
        os.close(write_fd)
    try:
        return subprocess.run(['sshpass', '-d', str(read_fd), *args],
                              pass_fds=(read_fd,), **kwargs)
    finally:
        os.close(read_fd)


def ssh(device, command, **kwargs):
    host = HOSTS[device][0]
    return authenticated(['ssh', *SSH_OPTIONS, host, command], host=host, **kwargs)
