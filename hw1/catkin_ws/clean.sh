#!/bin/bash

# clean build files
rm -f -r build/
rm -f -r devel/

# unlink & reinit ros workspace(for anaconda)
unlink src/CMakeLists.txt
(cd src/ && catkin_init_workspace)



