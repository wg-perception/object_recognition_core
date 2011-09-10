#!/bin/sh -e
. $1
cd $2
python -c "__import__('$3'.split('/')[-1].split('.py')[0])"


