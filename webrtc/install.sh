#!/usr/bin/bash
target_dir=$1

# Check pwd
if [ "$PWD" == "$target_dir" ]
then
    echo "In $target_dir"
else
    if ls $target_dir &> /dev/null
    then
        cd $target_dir
        echo "Change directory: $PWD"
    else
        echo "$target_dir path error. Please check directory"
        exit 1
    fi
fi
# pwd in target_dir

# Recover webrtc.js if .tmp exist
if cat webrtc.js.tmp &> /dev/null
then
    cp webrtc.js.tmp webrtc.js
    echo "webrtc.js recovered"
else
    cp webrtc.js webrtc.js.tmp
    echo "Backup webrtc.js: webrtc.js.tmp"
fi
read -p "Enter peer id:" pid
sed -i "s/default_peer_id = 101/default_peer_id = $pid/1" webrtc.js


# Recover webdriver.py if .tmp exist
if cat webdriver.py.tmp &> /dev/null
then
    cp webdriver.py.tmp webdriver.py
    echo "webdriver.py recovered"
else
    cp webdriver.py webdriver.py.tmp
    echo "Backup webdriver.py: webdriver.py.tmp"
fi
sed -i "s|DIRECTORY = '/home/pi/webcam'|DIRECTORY = '$target_dir'|1" webdriver.py


# Recover run.sh if .tmp exist
if cat run.sh.tmp &> /dev/null
then
    cp run.sh.tmp run.sh
    echo "run.sh recovered"
else
    cp run.sh run.sh.tmp
    echo "Backup run.sh: run.sh.tmp"
fi
echo "python3 $target_dir/webdriver.py" >> run.sh