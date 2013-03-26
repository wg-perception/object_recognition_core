#!/bin/sh
#sudo pip install -U couchapp"
DIR="$( cd "$( dirname "$0" )" && pwd )"
couchapp push ${DIR} http://localhost:5984/or_web_ui
