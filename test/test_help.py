#!/usr/bin/env python
import subprocess
import sys
import os
path = os.path.dirname(sys.argv[0])

print 'training'
subprocess.check_call(['%s/../apps/training'%path,'--help'])
print 'training'
subprocess.check_call(['%s/../apps/detection'%path,'--help'])
print 'upload'
subprocess.check_call(['%s/../apps/upload'%path,'--help'])
print 'mesh_object'
subprocess.check_call(['%s/../apps/mesh_object'%path,'--help'])
