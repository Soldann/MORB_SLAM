parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path" # change directories so working directory is where the script is

#apt update for good measure
sudo apt update

sudo apt install -y build-essential cmake ninja-build libeigen3-dev libssl-dev libboost-all-dev libopencv-dev libpython3.9

#pangolin
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git --depth=1 -b v0.8
cd Pangolin

./scripts/install_prerequisites.sh -m recommended 

mkdir build
cd build

set -e # set to abort on error

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig > /dev/null 2> /dev/null

echo "Finished installing all of the dependencies! Now just run build.sh! You can use -jX for choosing the number of workers with build.sh. default:-j$(nproc)"
