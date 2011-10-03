#!/usr/bin/env python

import subprocess
import sys
import os
path = os.path.dirname(sys.argv[0])
print path
subprocess.check_call(['%s/../apps/capture'%path,'--help'])