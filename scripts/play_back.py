#!/bin/python3
#
# @brief Script used to start a launch file and play back a bag file.
#
# @args
#   -b, --bag-file   (str) Bag file to play back
#   -w, --wait-time  (int) Wait time for bag file to load (default is 5 seconds)
#
# @usage
#   ./play_back.py -b <bag-file> [-w <wait-time>]
#
# @examples
#   ./play_back.py -b rosbag2_2022_04_14-00_52_30_0.db3 -w 5

import os
from multiprocessing import Process
import time
import argparse


def run_launchfile():
    print("\033[96mStarting the \033[96;1mlaunch file\033[0m")
    # Start launch file
    os.system("ros2 launch turtle_nav_cpp turtle_nav_filter.launch.py")


def play_bagfile(bagfile, wait_time=5):
    # Add time delay to allow the launch file to open
    print("\033[95mWaiting for launch file to start\033[0m")
    for i in range(wait_time, 0, -1):
        print(f"\033[93;1m{i}\033[0m")
        time.sleep(1)

    print(f"\033[96mStarting the play back file: \033[96;1m{bagfile}\033[0m")
    # Load bag file
    os.system(f"ros2 bag play {bagfile}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Script for starting dead-reckoning launch file and start a bag file"
    )
    requiredArgs = parser.add_argument_group("Arguments:")
    requiredArgs.add_argument(
        "-b", "--bag-file", required=True, type=str, nargs=1, help="Bag file name, with extension"
    )
    requiredArgs.add_argument(
        "-w",
        "--wait-time",
        type=int,
        nargs=1,
        default=[5],
        help="Wait time before loading bag file",
    )

    args, _ = parser.parse_known_args()
    print(f"Bag file name: {args.bag_file[0]}")
    print(f"Wait time: {args.wait_time[0]}")

    process_launchfile = Process(target=run_launchfile)
    process_launchfile.start()
    process_play_bagfile = Process(
        target=lambda: play_bagfile(args.bag_file[0], args.wait_time[0])
    )
    process_play_bagfile.start()
    process_launchfile.join()
    process_play_bagfile.join()
