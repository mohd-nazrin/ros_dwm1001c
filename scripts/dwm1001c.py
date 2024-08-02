# Part of ros_uwb_dwm1001 project
#
# Copyright (c) 2023 The Human and Intelligent Vehicle Ensembles (HIVE) Lab
# Copyright (c) 2024 Mohd Nazrin Muhammad
#
# Licensed under the MIT License
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# This file is based on work from the Human and Intelligent Vehicle Ensembles (HIVE) Lab.
# Original source: https://github.com/the-hive-lab/pydwm1001/blob/main/dwm1001.py
#
# Modifications by Mohd Nazrin Muhammad:
# - Converted Python code to ROS-compatible Python code
# - Published TagPosition as a PointStamped message on the "uwb_pos" topic
# Standard library imports
from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Tuple

# Third party imports
from serial import Serial

# ROS imports
import rospy
from geometry_msgs.msg import PointStamped, Point

class TagId(str):
    pass

class TagName(str):
    pass

@dataclass
class TagPosition:
    x_m: float
    y_m: float
    z_m: float
    quality: int

    def is_almost_equal(self, other: "TagPosition") -> bool:
        return (
            math.isclose(self.x_m, other.x_m)
            and math.isclose(self.y_m, other.y_m)
            and math.isclose(self.z_m, other.z_m)
            and self.quality == other.quality
        )

@dataclass
class SystemInfo:
    uwb_address: str
    label: str

    @staticmethod
    def from_string(data: str) -> "SystemInfo":
        data_lines = data.splitlines()

        uwb_address_line = data_lines[1].strip()
        address_text_start = uwb_address_line.find("addr=")
        address_string = "0" + uwb_address_line[address_text_start + len("addr=") :]

        label_line = data_lines[5].strip()
        label_text_start = label_line.find("label=")
        label_string = label_line[label_text_start + len("label=") :]

        return SystemInfo(uwb_address=address_string, label=label_string)

class ShellCommand(Enum):
    ENTER = b"\r"
    DOUBLE_ENTER = b"\r\r"
    LEP = b"lep"
    RESET = b"reset"
    SI = b"si"  # System info

class ParsingError(Exception):
    pass

class UartDwm1001:
    # These delay periods were experimentally determined
    RESET_DELAY_PERIOD = 0.1
    SHELL_STARTUP_DELAY_PERIOD = 1.0
    SHELL_COMMAND_DELAY_PERIOD = 0.5

    def __init__(self, serial_handle: Serial) -> None:
        self.serial_handle = serial_handle

    @property
    def system_info(self) -> SystemInfo:
        self.send_shell_command(ShellCommand.SI)
        response = self.get_shell_response()

        return SystemInfo.from_string(response)

    def send_shell_command(self, command: ShellCommand) -> None:
        self.serial_handle.write(command.value)
        self.serial_handle.write(ShellCommand.ENTER.value)
        self.serial_handle.flush()

        time.sleep(self.SHELL_COMMAND_DELAY_PERIOD)

    def get_shell_response(self) -> str:
        raw_data = self.serial_handle.read_until(b"dwm> ")

        # Raw data includes sent command, followed by a new line. The
        # response starts after the new line.
        response_begin = raw_data.find(b"\r\n") + 2

        # Raw data always ends with the shell prompt.
        response_end = raw_data.rfind(b"dwm> ")

        return raw_data[response_begin:response_end].decode().rstrip()

    def reset(self) -> None:
        self.send_shell_command(ShellCommand.RESET)

        time.sleep(self.RESET_DELAY_PERIOD)

    def enter_shell_mode(self) -> None:
        self.serial_handle.write(ShellCommand.DOUBLE_ENTER.value)
        self.serial_handle.flush()

        time.sleep(self.SHELL_STARTUP_DELAY_PERIOD)
        self.serial_handle.reset_input_buffer()

    def exit_shell_mode(self) -> None:
        # If you quit shell mode (with `quit` command) without stopping
        # a running command, the command will automatically continue
        # when re-entering shell mode. This can be confusing, so we
        # reset the device instead to ensure previously-running commands
        # terminate.
        self.reset()

    def start_position_reporting(self) -> None:
        self.send_shell_command(ShellCommand.LEP)

        # The first line after invoking the command will have part of
        # the shell prompt mixed in, which would mess up parsing.
        self.serial_handle.reset_input_buffer()

    def stop_position_reporting(self) -> None:
        self.send_shell_command(ShellCommand.ENTER)

class PassiveTag(UartDwm1001):
    def __init__(self, serial_handle: Serial) -> None:
        super().__init__(serial_handle)

        # Device may not have shutdown properly previously
        self.reset()
        self.enter_shell_mode()

    def wait_for_position_report(self) -> Tuple[TagName, TagPosition]:
        report = self.serial_handle.readline().decode().split(",")

        if len(report) != 8:
            raise ParsingError("Position report has unexpected length.")

        if report[0] != "POS":
            raise ParsingError("Position report has incorrect tag.")

        position_data = [float(substr) for substr in report[3:6]]
        position_data.append(int(report[6]))

        return TagName("DW" + report[2]), TagPosition(*position_data)

class ActiveTag(UartDwm1001):
    def __init__(self, serial_handle: Serial) -> None:
        super().__init__(serial_handle)
        self.reset()
        self.enter_shell_mode()

    @property
    def position(self) -> TagPosition:
        # Read the line from serial and decode it
        report = self.serial_handle.readline().decode().strip()
        rospy.loginfo(f"Received report: {report}")

        # Split the report based on commas and debug
        report_parts = report.split(",")
        rospy.loginfo(f"Report parts: {report_parts}")

        # Example report: "POS,0.0,0.0,0.0,100"
        if len(report_parts) != 5:
            raise ParsingError("Position report has unexpected length.")

        if report_parts[0] != "POS":
            raise ParsingError("Position report has incorrect tag.")

        try:
            # Extract position values
            position_data = [float(report_parts[1]), float(report_parts[2]), float(report_parts[3])]
            quality = int(report_parts[4])
            return TagPosition(*position_data, quality)
        except ValueError:
            raise ParsingError("Invalid data format in position report.")

def main():
    # Initialize ROS node
    rospy.init_node('uwb_position_publisher', anonymous=True)

    # Create a publisher for the 'uwb_pos' topic
    position_pub = rospy.Publisher('uwb_pos', PointStamped, queue_size=10)

    # Assuming a serial connection is established
    serial_handle = Serial('/dev/ttyACM0', 115200, timeout=15)  # Update port as necessary
    active_tag = ActiveTag(serial_handle)

    # Start position reporting
    active_tag.start_position_reporting()

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Get position and construct PointStamped message
            position = active_tag.position
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = "dwm1001"  # Frame ID can be adjusted as needed
            point_msg.point.x = position.x_m
            point_msg.point.y = position.y_m
            point_msg.point.z = position.z_m
            position_pub.publish(point_msg)
            rospy.loginfo(f"Published position: {point_msg}")

        except ParsingError as e:
            rospy.logerr(f"Failed to parse position: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
