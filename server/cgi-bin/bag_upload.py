#!/usr/bin/env python

import cgi
import cgitb
cgitb.enable()
import tempfile
import sys
import recognition_msgs

sys.stderr = sys.stdout
print "Content-Type: text/plain"
print

form = cgi.FieldStorage()
try:
  bag_item = form['bag_file']
except KeyError:
  print "You must supply a bag file."
  sys.exit(-1)

if bag_item.file and bag_item.done:
    tmp_file,tmp_name = tempfile.mkstemp(suffix='.bag')
    tmp_file = open(tmp_name,'wb')
    tmp_file.write(bag_item.file.read())
    msg = str(dict(file=tmp_name,object_name=form.getfirst('object_name','NA')))
    recognition_msgs.send_data_ready(msg)
    print "Thank you for your upload."
else:
    print "Your upload was not finished."
