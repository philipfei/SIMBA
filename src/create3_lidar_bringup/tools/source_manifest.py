#!/usr/bin/env python3
"""Relative-path SHA-256 audit, with the same exclusions locally and remotely."""
import hashlib
import json
from pathlib import Path
import sys


def manifest(root):
    root=Path(root).resolve();result={}
    for p in sorted(root.rglob('*')):
        if any(part in ('__pycache__','.pytest_cache','.git') for part in p.relative_to(root).parts) or p.suffix=='.pyc':continue
        if p.is_symlink():raise ValueError('Source symlinks are not accepted: '+str(p))
        if p.is_file():result[p.relative_to(root).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
    return result

if __name__=='__main__':
    text=json.dumps(manifest(sys.argv[1]),indent=2,sort_keys=True)+'\n'
    if len(sys.argv)>2:Path(sys.argv[2]).write_text(text)
    else:print(text,end='')
