echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build 2> /dev/null
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build 2> /dev/null
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build 2> /dev/null
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

cd ../../../


cd Vocabulary
if [ ! -f "ORBvoc.txt" ]; then
    echo "Uncompress vocabulary ..."
    tar -xf ORBvoc.txt.tar.gz
fi
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build 2> /dev/null
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
