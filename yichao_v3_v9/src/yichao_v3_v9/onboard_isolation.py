"""LCM subscription surface for onboard shadow, with no command publication."""
import hashlib
import json
import os
from pathlib import Path


def control_processes(proc_root=Path('/proc')):
    found = []
    for path in proc_root.glob('[0-9]*/cmdline'):
        if path.parent.name == str(os.getpid()):
            continue
        try:
            args = path.read_bytes().decode().split('\0')
        except (OSError, UnicodeDecodeError):
            continue
        if any(Path(arg).name in ('g1_control','deploy_policy.py','run_onboard_movement.py',
                                 'run_onboard_relay.py','run_onboard_active.py',
                                 'run_active_control.py') for arg in args if arg):
            found.append(int(path.parent.name))
    return sorted(found)


def verify_release(root):
    for name in ('onboard_sources.json', 'onboard_environment.json'):
        manifest = json.loads((root/'config'/name).read_text())
        for relative, expected in manifest['sha256'].items():
            path = root/relative
            path.resolve().relative_to(root.resolve())
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(chunk)
            if digest.hexdigest() != expected:
                raise RuntimeError('onboard release changed: '+relative)


class ReadOnlyLCM:
    def __init__(self, bus):
        self._bus = bus

    def publish(self, *args, **kwargs):
        raise RuntimeError('onboard shadow prohibits all LCM publications')

    def subscribe(self, *args, **kwargs):
        return self._bus.subscribe(*args, **kwargs)

    def unsubscribe(self, *args, **kwargs):
        return self._bus.unsubscribe(*args, **kwargs)

    def fileno(self):
        return self._bus.fileno()

    def handle(self):
        return self._bus.handle()

    def handle_timeout(self, *args, **kwargs):
        return self._bus.handle_timeout(*args, **kwargs)
