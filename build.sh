#!/bin/bash

if [ $# == "1" ]; then
    jobs=$1
else
    jobs="-j$(nproc)"
fi
echo "Using argument ${jobs}"

# https://stackoverflow.com/questions/24112727/relative-paths-based-on-file-location-instead-of-current-working-directory
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path" # change directories so working directory is where the script is

cd Vocabulary
if [ ! -f "ORBvoc.txt" ]; then
    echo "Uncompress vocabulary ..."
    tar -xf ORBvoc.txt.tar.gz
fi
cd ..

if [ ! -d "build" ] || [ ! -f 'build/config-finished.bool' ]; then
        echo 'Performing first time configuration'
        mkdir build 2> /dev/null
        cd build
        # https://unix.stackexchange.com/questions/31414/how-can-i-pass-a-command-line-argument-into-a-shell-script
        cmake .. -GNinja
        if [ $? -ne 0 ]; then
                rm 'config-finished.bool' 2> /dev/null
                cd ..
                echo "Configuration was not successful"
                exit 1
        fi
        touch 'config-finished.bool'
else
        echo 'Already configured'
        cd build
fi
ninja $jobs
if [ $? -ne 0 ]; then
        cd ..
        echo "Build was not successful"
        exit 2
fi
sudo ninja install
if [ $? -ne 0 ]; then
        cd ..
        echo "Install was not successful"
        exit 3
fi
