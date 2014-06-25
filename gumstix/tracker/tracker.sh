#!/bin/sh

prefix=/opt/tracker

while true; do
   ls -l $prefix/tracker
   ls -l $prefix/tracker.cfg
   ls -l $prefix/cam.cfg
   ls -l $prefix/cam.m
   echo "`date` INFO: Start tracker"
   $prefix/tracker --config $prefix/tracker.cfg --camera-control-settings $prefix/cam.cfg --camera-parameters $prefix/cam.m --logger-config $prefix/logger.cfg
   echo "`date` INFO: Tracker has been terminated"
done
