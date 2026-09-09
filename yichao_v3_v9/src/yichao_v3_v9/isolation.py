"""Fail closed on Python network/process operations in the file-only entrypoint."""
import sys

def install_guard():
    def guard(event,args):
        if event in ('socket.connect','socket.bind','socket.sendto','subprocess.Popen','os.system','os.posix_spawn'):
            raise RuntimeError('offline isolation blocked '+event)
    sys.addaudithook(guard)
