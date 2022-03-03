#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

source $SCRIPT_DIR/install_packages.sh

source $SCRIPT_DIR/colcon_build.sh 

source $SCRIPT_DIR/colcon_test.sh

