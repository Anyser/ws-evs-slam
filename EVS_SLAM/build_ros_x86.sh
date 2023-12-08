echo "Building ROS nodes"

cd Examples/ROS/orb_slam2
rm -rf build
mkdir build
cd build
cmake .. -DPYTHON_EXECUTABLE=/usr/bin/python3.6m \
-DPYTHON_LIBRARY=/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu/libpython3.6m.so \
-DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
-DROS_BUILD_TYPE=Release
make -j8
