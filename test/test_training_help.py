#!/usr/bin/env python

import subprocess
import sys
import os
path = os.path.dirname(sys.argv[0])
subprocess.check_call(['%s/../apps/training'%path,'--help'])
