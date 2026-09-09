"""Hash selected regular files; ITEMS is supplied over stdin, no remote writes."""
import hashlib
import json
import os
import stat
import time

result = {'started_ns': time.time_ns(), 'files': [], 'errors': []}
for path in ITEMS:
    try:
        before = os.lstat(path)
        if not stat.S_ISREG(before.st_mode):
            raise ValueError('Not a regular file: ' + path)
        digest = hashlib.sha256()
        with open(path, 'rb') as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b''):
                digest.update(block)
        after = os.lstat(path)
        if (before.st_size, before.st_mtime_ns, before.st_ino) != (after.st_size, after.st_mtime_ns, after.st_ino):
            raise ValueError('Changed during hashing: ' + path)
        result['files'].append({'path':path,'sha256':digest.hexdigest(),'size':after.st_size,
                                'mtime_ns':after.st_mtime_ns,'mode':stat.S_IMODE(after.st_mode)})
    except (OSError, ValueError) as e:
        result['errors'].append(str(e))
result['finished_ns'] = time.time_ns()
print(json.dumps(result))
