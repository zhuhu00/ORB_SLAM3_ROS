echo "Configuring and building the Thirdparty/DBoW3 ..."

cd Thirdparty/DBoW3
mkdir build
cd build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building the Thirdparty/g2o ..."
mkdir build
cd build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../../

echo "Thirdparty lib build success!!===="