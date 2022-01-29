#!/bin/bash -e

while true; do
    echo "Press Ctrl+C now to exit!"
    sleep 5
    echo "Launching application, please wait!"
    roslaunch mogi_chess_manager auto_manager.launch
    echo "End"
done