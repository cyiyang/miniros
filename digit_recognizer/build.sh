rm -rf build
mkdir build
cd build
cmake ..
make
ln -s ../model/yolox.bin yolox.bin
ln -s ../model/yolox.param yolox.param
echo "编译完成!"
