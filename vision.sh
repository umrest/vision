#!/bin/bash

DIR=/home/pi/rest/vision

cd $DIR
while true; do
    ./vision
    echo "Vision Crashed..."
    sleep 2
done