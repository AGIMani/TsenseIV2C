# CMake generated Testfile for 
# Source directory: /home/hj/hand/hand-ubuntu24.04-20261006/hand/src/a32_driver/native_linux
# Build directory: /home/hj/hand/hand-ubuntu24.04-20261006/hand/src/a32_driver/build-linux
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(linux_offline_transport "/usr/bin/python3" "/home/hj/hand/hand-ubuntu24.04-20261006/hand/src/a32_driver/native_linux/tests/test_offline.py" "/home/hj/hand/hand-ubuntu24.04-20261006/hand/src/a32_driver/build-linux/artifacts/liba32_native.so")
set_tests_properties(linux_offline_transport PROPERTIES  _BACKTRACE_TRIPLES "/home/hj/hand/hand-ubuntu24.04-20261006/hand/src/a32_driver/native_linux/CMakeLists.txt;27;add_test;/home/hj/hand/hand-ubuntu24.04-20261006/hand/src/a32_driver/native_linux/CMakeLists.txt;0;")
subdirs("a3-cpp-sdk")
