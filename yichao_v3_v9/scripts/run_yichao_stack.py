#!/usr/bin/env python3
"""Use this release's private Python; no conda activation required."""
import os
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[1]
if os.environ.get('_YICHAO_PRIVATE_PYTHON')!=str(root):
    env=os.environ.copy();env['_YICHAO_PRIVATE_PYTHON']=str(root)
    os.execve(str(root/'.venv/bin/python'),[str(root/'.venv/bin/python'),'-B',__file__,*sys.argv[1:]],env)
sys.path.insert(0,str(root/'src'))
from yichao_v3_v9.two_terminal import main
sys.argv=[sys.argv[0],'stack',*sys.argv[1:]]
raise SystemExit(main())
