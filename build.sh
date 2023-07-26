#!/bin/bash

# https://stackoverflow.com/questions/24112727/relative-paths-based-on-file-location-instead-of-current-working-directory
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path" # change directories so working directory is where the script is

# Initialize lists
cmake_args=()
j_arg="-j$(nproc)"
g_arg=""
# Iterate over arguments
for arg in "$@"; do
  if [[ $arg =~ ^-j[0-9]+$ ]]; then
    j_arg="$arg"
  elif [[ $arg =~ ^-G[0-9a-zA-Z]+$ ]]; then
    g_arg="$arg"
  else
    cmake_args+=("$arg")
  fi
done
# Check if the g_arg is empty (non-user specified generator)
if [ -z "$g_arg" ]; then
    # select ninja if available, make otherwise
    if which ninja >/dev/null 2>&1; then
        g_arg="-GNinja"
    elif which make >/dev/null 2>&1; then
        g_arg="-GMake"
    else
        echo "Please either install Ninja (preffered), Make, or specify an installed Generator?"
        echo "    For ninja `sudo apt install ninja-build`"
        echo "    For make  `sudo apt install build-essential`"
        exit -1
    fi
fi


if [ ! -f "Vocabulary/ORBvoc.txt" ]; then
    cd Vocabulary
    echo "Extracting vocabulary..."
    tar -xf ORBvoc.txt.tar.gz
    echo "ORB Vocabulary extracted"
    cd $parent_path
else
    echo "ORB Vocabulary already extracted"
fi

configCompleteFile='config-finished.bool'
# https://unix.stackexchange.com/questions/31414/how-can-i-pass-a-command-line-argument-into-a-shell-script
if [ ! -d "build" ] || [ ! -f "build/${configCompleteFile}" ]; then
        echo 'Performing first time configuration...'
        echo "Workers: ${j_arg}    Generator: ${g_arg}"
        echo "User Flags: ${cmake_args[@]}"
        mkdir build 2> /dev/null
        cd build
        # https://unix.stackexchange.com/questions/31414/how-can-i-pass-a-command-line-argument-into-a-shell-script
        cmake .. ${g_arg} "${cmake_args[@]}" # pass arguments on to cmake
        if [ $? -ne 0 ]; then
                rm "${configCompleteFile}" 2> /dev/null
                cd ..
                echo "Configuration was not successful"
                exit 1
        fi
        touch "${configCompleteFile}"
        echo -e "    Generator: ${g_arg}\nUser Flags: ${cmake_args[@]}" > "${configCompleteFile}"
else
        echo 'Already configured'
        cd build
        config=$(cat "${configCompleteFile}")
        echo "Workers: ${j_arg}${config}"
fi

if [ $? -ne 0 ]; then
	cd ..
	echo "Configuration was not successful"
	exit 1
fi
echo "Building..."
cmake --build . -- ${j_arg}
if [ $? -ne 0 ]; then
        cd ..
        echo "Build was not successful"
        exit 2
fi
sudo cmake --install .
if [ $? -ne 0 ]; then
        cd ..
        echo "Install was not successful"
        exit 3
fi