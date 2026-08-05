#!/bin/bash
pkill -f 'lathe_control/.venv/bin/python main.py' 2>/dev/null || true
sleep 1
cd /home/pi/lathe_control || exit 1
export DISPLAY=:0
export XAUTHORITY=/home/pi/.Xauthority
nohup .venv/bin/python main.py > /tmp/lathe_gui.log 2>&1 &
echo "PID=$!"
sleep 3
pgrep -af 'main.py' || true
echo '--- log ---'
cat /tmp/lathe_gui.log || true
