#!/usr/bin/env python
import subprocess
import sys
import os

script = sys.argv[1]

print script
subprocess.check_call([script,'--help'])
