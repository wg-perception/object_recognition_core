#!/usr/bin/env python
import os
import subprocess
import sys

new_dir = sys.argv[1]

current_path = os.getcwd()
new_cwd = os.path.join(current_path, new_dir)
os.chdir(new_cwd)
print 'move from ' + current_path + ' to ' + os.getcwd()

for file_name in os.listdir(os.getcwd()):
    if file_name.endswith('.py'):
        module_name = file_name.split('.py')[0]
        print 'module "%s" in %s' % (module_name, os.getcwd())
        __import__('test_opposing_dot_estimator')
        __import__('bilateral')
       # __import__(module_name)
