#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import time
from visualization_msgs.msg import Marker


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_follow_gap_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info("Node started successfully")

        self.lidarscan_topic = '/scan'
        self.drive_topic = '/drive'
        
        self.subscription = self.create_subscription(
            LaserScan, self.lidarscan_topic, self.lidar_callback, 10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10)
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)

        self.bubble_radius = 0.16  # radius to keep clear of obstacles
        self.max_distance = 4.0   # max distance to consider for a valid gap
        self.disparity_thresh = 0.10

    def disparity_extender(self, ranges, angle_increment):
        # Start and end indices to process only the elements in front of the car
        start_index = 180
        end_index = len(ranges) - 180
        #self.get_logger().info(f"End index:  {end_index}")
        
        i = start_index
        while i < end_index - 1:
            # Check for disparity between consecutive measurements
            if abs(ranges[i] - ranges[i + 1]) > self.disparity_thresh:
                # Determine the lower element of the disparity
                lower_index = i if ranges[i] < ranges[i + 1] else i + 1
                lower_value = ranges[lower_index]

                # Set a bubble around the lower element
                extend_step = 0
                while True:
                    # Calculate the angle for the current step
                    current_angle = extend_step * angle_increment
                    # Check if the point is within the bubble radius
                    if lower_value * math.tan(current_angle) < self.bubble_radius:
                        # Set elements within the bubble to zero, checking bounds
                        if lower_index - extend_step >= start_index:
                            ranges[lower_index - extend_step] = 0
                        if lower_index + extend_step < end_index:
                            ranges[lower_index + extend_step] = 0
                        extend_step += 1
                    else:
                        break

                # Move index past the extended bubble
                i = lower_index + extend_step
            else:
                i += 1

    def closest_point_zero_out(self, ranges, angle_increment):
        start_index = 180
        end_index = len(ranges) - 180

        # Find the closest non-zero point in the specified range
        min_distance = float('inf')
        closest_index = -1
        for i in range(start_index, end_index):
            if ranges[i] != 0 and ranges[i] < min_distance:
                min_distance = ranges[i]
                closest_index = i

        if closest_index != -1:  # If a closest point was found
            # Calculate the number of steps to extend based on the bubble radius
            extend_step = 0
            while True:
                current_angle = extend_step * angle_increment
                if min_distance * math.tan(current_angle) < self.bubble_radius:
                    # Set elements within the bubble to zero, checking bounds
                    if closest_index - extend_step >= start_index:
                        ranges[closest_index - extend_step] = 0
                    if closest_index + extend_step < end_index:
                        ranges[closest_index + extend_step] = 0
                    extend_step += 1
                else:
                    break


    def preprocess_lidar(self, ranges):
        # Apply a mean filter and threshold the distances
        proc_ranges = np.array(ranges)

        window_size = 3
        proc_ranges = np.convolve(proc_ranges, np.ones(window_size)/window_size, mode='same')
        proc_ranges[proc_ranges > self.max_distance] = self.max_distance
        return proc_ranges


    def find_max_gap(self, free_space_ranges):
        # Find the longest sequence of consecutive non-zero values (max gap)
        gaps = np.split(free_space_ranges, np.where(free_space_ranges == 0)[0])
        max_gap = max(gaps, key=len)
        start_i = np.where(free_space_ranges == max_gap[0])[0][0]
        end_i = start_i + len(max_gap) - 1
        return start_i, end_i

    def find_best_point(self, ranges):
        best_gap_center = -1
        max_depth = -1
        current_depth = 0
        start_index = -1
        weighted_index_sum = 0  # For calculating the weighted center

        max_distance = np.max(ranges)
        max_index = np.argmax(ranges)
        #self.get_logger().info(f"max_distance is: {max_distance}") 
        #self.get_logger().info(f"max_index is: {max_index}") 

        for i in range(180, len(ranges) - 180):
            if ranges[i] > 0:  # Part of a gap
                if start_index == -1:  # This is the start of a new gap
                    start_index = i
                current_depth += ranges[i]
                weighted_index_sum += i * (ranges[i]**2)  # Weight index by square of range value
            else:  # Not part of a gap or gap ended
                if start_index != -1:  # End of a gap
                    gap_length = i - start_index
                    gap_depth = current_depth / gap_length  # Average depth of the gap
                    if gap_depth > max_depth:
                        max_depth = gap_depth
                        # Calculate weighted center of the gap
                        if current_depth != 0:  # Avoid division by zero
                            total_weight = sum(ranges[start_index:i]**2)
                            best_gap_center = weighted_index_sum // total_weight
                        else:
                            best_gap_center = (start_index + i - 1) // 2
                    # Reset for the next potential gap
                    start_index = -1
                    current_depth = 0
                    weighted_index_sum = 0
        
        # Check in case the last values in the array form the deepest gap
        if start_index != -1:
            gap_length = len(ranges) - 180 - start_index
            gap_depth = current_depth / gap_length
            if gap_depth > max_depth:
                if current_depth != 0:
                    total_weight = sum(ranges[start_index:len(ranges) - 180]**2)
                    best_gap_center = weighted_index_sum // total_weight
                else:
                    best_gap_center = (start_index + len(ranges) - 180 - 1) // 2
        
        #self.get_logger().info(f"best_gap_center is: {best_gap_center}") 
        target_distance = ranges[int(best_gap_center)]
        #self.get_logger().info(f"Furthest distance is: {target_distance:.3f}") 
        return best_gap_center


    def lidar_callback(self, data):
        # start_time = time.time() 
        ranges = np.array(data.ranges)
        angle_increment = data.angle_increment

        proc_ranges = self.preprocess_lidar(ranges)
        self.disparity_extender(proc_ranges, angle_increment)
        self.closest_point_zero_out(proc_ranges, angle_increment)

        # Eliminate all points inside 'bubble'
        # self.update_safety_bubble(proc_ranges, angle_increment, self.bubble_radius)

        # Find max length gap
        # start_i, end_i = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best_index = self.find_best_point(proc_ranges)
        target_distance = ranges[int(best_index)]

        front_distance = proc_ranges[540]
        # front_distance = target_distance # REMOVE THIS AFTER ********************************
        # Create and publish Ackermann drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        
        angle = (best_index - len(ranges) / 2) * data.angle_increment


        #These 3 lines are for visulizing
        # target_distance = ranges[int(best_index)]
        # angle_increment = data.angle_increment  # Ensure angle_increment is correctly passed
        # self.publish_target_marker(best_index, target_distance, angle_increment)


        # self.get_logger().info(f"Best index is: {best_index}") 
        # self.get_logger().info(f"Angle is: {angle:.2f}") 
        # Adjust the speed
        
        max_speed = 4.72 #was 4.5
        if abs(angle)>0.78:
            set_speed = 0.9
            
            self.get_logger().info(f"The angle is over maximum: {angle:.3f}") 
        else:
            if front_distance < 1 :   # 0.349066=20 degree
                set_speed = 0.9       # * (front_distance/self.max_distance)
            elif front_distance < 3:
                set_speed = 1.2 * front_distance + 0.415
            else:
                set_speed = max_speed
        
                self.get_logger().info(f"The speed reaches maximum: {set_speed:.3f}")               
                self.get_logger().info(f"When the front distance is: {front_distance:.3f}")
        #alternate choice of speed control
        # if angle < 0.2:
        #     set_speed = max_speed
        # elif angle < 0.5:
        #     set_speed = 1
        # else:i
            # set_speed = 0.5


        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = set_speed  # Set your desired speed
        self.publisher.publish(drive_msg)

        # end_time = time.time()  # End time measurement
        # elapsed_time = end_time - start_time
        # self.get_logger().info(f"Execution time: {elapsed_time:.6f} seconds") 

    # def publish_target_marker(self, best_index, target_distance, angle_increment):
    #     marker = Marker()
    #     marker.header.frame_id = "laser"  # Make sure this matches your RViz fixed frame
    #     marker.header.stamp = self.get_clock().now().to_msg()

    #     marker.ns = "target_point"
    #     marker.id = 0

    #     marker.type = Marker.SPHERE
    #     marker.action = Marker.ADD

    #     # Convert index and distance to coordinates
    #     angle = (best_index - len(data.ranges) / 2) * angle_increment
    #     x = target_distance * np.cos(angle)
    #     y = target_distance * np.sin(angle)

    #     marker.pose.position.x = x
    #     marker.pose.position.y = y
    #     marker.pose.position.z = 0

    #     marker.pose.orientation.x = 0.0
    #     marker.pose.orientation.y = 0.0
    #     marker.pose.orientation.z = 0.0
    #     marker.pose.orientation.w = 1.0

    #     marker.scale.x = 0.2
    #     marker.scale.y = 0.2
    #     marker.scale.z = 0.2

    #     marker.color.a = 1.0
    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0

    #     self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
