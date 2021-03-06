#! /bin/bash
brew install openssl libuv cmake zlib
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < /Users/shankschen/Documents/PythonProject/First-Term-Self-Driving-Car/P5-Extended-Kalman-Filters-Project/CarND-Extended-Kalman-Filter-Project/cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
cd build
OPENSSL_VERSION=`openssl version -v | cut -d' ' -f2`
cmake -DOPENSSL_ROOT_DIR=/usr/local/opt/openssl -DOPENSSL_LIBRARIES=/usr/local/opt/openssl/lib
make 
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
