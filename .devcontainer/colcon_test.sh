#!/bin/bash

# Create /Dev directory if it doesn't exist
if [ ! -d /Dev ]
then
  mkdir /Dev
fi

# Install colcon tools
if [ ! -d /Dev/colcon_test_tools ]
then
  git clone https://github.com/aalbaali/colcon_test_tools.git /Dev/colcon_test_tools
  cd /Dev/colcon_test_tools
  ./install.sh

  source /usr/local/bin/colcon_tools.sh
fi

# Install packages
colcon test --merge-install

# Run all tests
colcon_test
