# Unitree SLAM keyDemo with QR pose recording

Modified keyDemo with QR detection.

Build:
cd /unitree/module/unitree_slam/example
rm -rf build
mkdir build
cd build
cmake ..
make -j4

Run:
export LD_LIBRARY_PATH=/usr/local/lib/dds_backup:/usr/local/lib:/unitree/module/unitree_slam/lib:$LD_LIBRARY_PATH
./keyDemo eth0

Keys:
q start mapping
w end mapping
a relocation + QR scan
s add pose
d execute list
f clear list
z pause
x resume
