#!/bin/sh -e
. $1
$2 --scheduler Singlethreaded --niter 100
$2 --scheduler Threadpool --niter 100
