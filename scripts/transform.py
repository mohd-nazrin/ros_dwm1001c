# Part of ros_uwb_dwm1001 project
#
# Copyright (c) 2023 The Human and Intelligent Vehicle Ensembles (HIVE) Lab
# Copyright (c) 2024 Mohd Nazrin Muhammad
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This file is based on work from the Human and Intelligent Vehicle Ensembles (HIVE) Lab.
# Original source: https://github.com/the-hive-lab/dwm1001_ros2/blob/integration/dwm1001_transform/dwm1001_transform/dwm1001_transform_node.py
#
# Modifications by Mohd Nazrin Muhammad:
# - Converted the code from ROS 2 to ROS 1
# - Changed input to receive the "uwb_pos" topic
# - Modified output to publish transforms to the "odometry/ips" topic
# Standard library imports
import rospy
import numpy as np

# ROS imports
from geometry_msgs.msg import Point, PointStamped, Quaternion, Transform, Vector3
from nav_msgs.msg import Odometry
from tf import TransformListener, transformations

# Function to convert quaternion message to a rotation matrix
def quaternion_msg_to_rotation_matrix(quaternion: Quaternion) -> np.ndarray:
    quat = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return transformations.quaternion_matrix(quat)[:3, :3]

# Function to convert Vector3 message to a numpy vector
def vector3_msg_to_vector(vector3: Vector3) -> np.ndarray:
    return np.asarray([[vector3.x], [vector3.y], [vector3.z]])

# Function to convert Point message to an augmented vector
def point_msg_to_augmented_vector(point: Point) -> np.ndarray:
    return np.array([[point.x], [point.y], [point.z], [1]])

# Function to convert an augmented vector back to a Point message
def augmented_vector_to_point_msg(vector: np.ndarray) -> Point:
    point = Point()
    point.x = vector[0, 0]
    point.y = vector[1, 0]
    point.z = vector[2, 0]
    return point

# Function to convert a Transform message to a transformation matrix
def transform_msg_to_matrix(transform: Transform) -> np.ndarray:
    rotation_matrix = quaternion_msg_to_rotation_matrix(transform.rotation)
    translation_vector = vector3_msg_to_vector(transform.translation)
    transform_matrix = np.hstack([rotation_matrix, translation_vector])
    transform_matrix = np.vstack([transform_matrix, np.array([0, 0, 0, 1])])
    return transform_matrix

class Dwm1001TransformNode:
    def __init__(self) -> None:
        # Initialize the ROS node
        rospy.init_node('dwm_transform', anonymous=True)

        # Get covariance parameters from the ROS parameter server
        # Correctly fetch the parameter as a string and convert to a list of floats
        covar_param_str = rospy.get_param('~position_cov', "[0.0, 0.0, 0.0]")
        self.covar = self._parse_covariance_string(covar_param_str)

        if self.covar:
            rospy.loginfo(f"DWM position covariance: {self.covar}")
            # Initialize the covariance matrix, only position will be set based on parameters
            self._covar = [0.0] * 36
            # Assuming uncorrelated x,y,z
            self._covar[0] = self.covar[0]  # x
            self._covar[7] = self.covar[1]  # y
            self._covar[14] = self.covar[2]  # z
        else:
            rospy.loginfo("No DWM1001 position covariance supplied.")
            self._covar = None

        # Set up the subscriber for the 'uwb_pos' topic
        self.tag_position_sub = rospy.Subscriber(
            'uwb_pos', PointStamped, self._tag_position_callback
        )

        # Set up the publisher for the 'odometry/ips' topic
        self.odom_pub = rospy.Publisher('odometry/ips', Odometry, queue_size=10)

        # Initialize the TransformListener
        self.tf_listener = TransformListener()

    def _parse_covariance_string(self, covar_str):
        try:
            # Convert the string to a list of floats
            return list(map(float, covar_str.strip('[]').split(',')))
        except ValueError:
            rospy.logerr("Invalid covariance parameter format. Expected a list of floats.")
            return [0.0, 0.0, 0.0]

    def _tag_position_callback(self, msg: PointStamped) -> None:
        try:
            # Look up the transformation from 'dwm1001' to 'map'
            self.tf_listener.waitForTransform('map', 'dwm1001', rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform('map', 'dwm1001', rospy.Time(0))
            transform = Transform()
            transform.translation = Vector3(*trans)
            transform.rotation = Quaternion(*rot)
            
            # Calculate the transformation matrix
            transform_matrix = transform_msg_to_matrix(transform)

            # Transform the UWB point to the map space
            dwm1001_point = point_msg_to_augmented_vector(msg.point)
            map_point = transform_matrix @ dwm1001_point

            # Create and populate the Odometry message
            map_odom = Odometry()
            map_odom.header.stamp = rospy.Time.now()
            map_odom.header.frame_id = "map"
            map_odom.pose.pose.position = augmented_vector_to_point_msg(map_point)

            # Only load covariance if it is supplied
            if self._covar:
                map_odom.pose.covariance = self._covar
            map_odom.twist.covariance[0] = -1

            # Publish the transformed Odometry message
            self.odom_pub.publish(map_odom)
            rospy.loginfo(f"Published transformed odometry: {map_odom}")

        except Exception as e:
            rospy.logerr(f"Could not transform from 'dwm1001' to 'map'. Error: {e}")

def main() -> None:
    # Instantiate the transformation node
    transform_node = Dwm1001TransformNode()

    # Keep the node alive
    rospy.spin()

if __name__ == "__main__":
    main()
