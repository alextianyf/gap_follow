

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>

class ReactiveFollowGap : public rclcpp::Node
{
public:
	ReactiveFollowGap() : Node("reactive_node")
	{
		// Topics & Subs, Pubs
		std::string lidarscan_topic = "/scan";
		std::string drive_topic = "/drive";

		// Subscriber to LIDAR
		lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
			lidarscan_topic, 10, std::bind(&ReactiveFollowGap::plan, this, std::placeholders::_1));

		// Publisher to drive
		drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
	}

private:
	// LIDAR processing parameters
	// const int bubble_radius = 160;
	const int preprocess_conv_size = 3;
	const int best_point_conv_size = 80;
	const double max_lidar_dist = 5.0;
	const double fast_speed = 5.0;
	const double straights_speed = 5.0;
	const double corners_speed = 1.0;
	const double straights_steering_angle = 0.174;
	const double fast_steering_angle = 0.0785;
	const int safe_threshold = 5;
	const double max_steer = 0.4;

	const float disparity_extender_thresh = 0.2;
	const float car_width = 0.25; // 0.28
	const float safety_bubble_size = 0.4;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

	void plan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
	{
		std::vector<float> proc_ranges = preprocess_lidar(scan_msg->ranges); // TODO: cut of back quarters of lidar scans
		// auto closest = std::distance(proc_ranges.begin(), std::min_element(proc_ranges.begin(), proc_ranges.end()));
		disparity_extender(proc_ranges, scan_msg->angle_increment);
		// Eliminate points inside 'bubble'
		update_safety_bubble(proc_ranges, scan_msg->angle_increment); // TODO: FIX THIS CRASHING CODE

		// int min_index = std::max(0, static_cast<int>(closest) - bubble_radius);
		// int max_index = std::min(static_cast<int>(proc_ranges.size()) - 1, static_cast<int>(closest) + bubble_radius);
		// std::fill(proc_ranges.begin() + min_index, proc_ranges.begin() + max_index, 0);

		// Find max length gap
		std::pair<int, int> gap_start_and_end = find_max_gap(proc_ranges);
		int gap_start = gap_start_and_end.first;
		int gap_end = gap_start_and_end.second;

		// Find best point in the gap
		auto best = find_best_point(gap_start, gap_end, proc_ranges);

		// Publish Drive message
		double steering_angle = get_angle(best, proc_ranges.size(), scan_msg->angle_increment, scan_msg->angle_min);
		double speed;
		if (std::abs(steering_angle) > straights_steering_angle)
			speed = corners_speed;
		else if (std::abs(steering_angle) > fast_steering_angle)
			speed = straights_speed;
		else
			speed = fast_speed;

		auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
		drive_msg.drive.steering_angle = steering_angle;
		drive_msg.drive.speed = speed;
		drive_publisher_->publish(drive_msg);
	}

	std::vector<float> preprocess_lidar(const std::vector<float> &ranges)
	{

		int behind_view_offset = ranges.size() / 5;

		std::vector<float> proc_ranges(ranges.begin() + behind_view_offset, ranges.end() - behind_view_offset);
		for (size_t i = 0; i < proc_ranges.size(); ++i)
		{
			int window_start = std::max(0, static_cast<int>(i) - preprocess_conv_size / 2);
			int window_end = std::min(static_cast<int>(proc_ranges.size()) - 1, static_cast<int>(i) + preprocess_conv_size / 2);

			float window_sum = std::accumulate(proc_ranges.begin() + window_start, proc_ranges.begin() + window_end + 1, 0.0);
			proc_ranges[i] = window_sum / (window_end - window_start + 1);
			if (proc_ranges[i] > max_lidar_dist)
			{
				proc_ranges[i] = max_lidar_dist;
			}
		}
		return proc_ranges;
	}

	void update_safety_bubble(std::vector<float> &ranges, double angle_increment)
	{
		// find closest distance and index

		if (ranges.empty())
		{
			// Handle the case when ranges is empty
			return;
		}

		int closest_point = 0;
		for (size_t i = 1; i < ranges.size(); i++)
		{
			if (ranges[i] < ranges[closest_point])
			{
				closest_point = i;
				// RCLCPP_INFO(this->get_logger(),"Updating Min Index to %d", min_index);
			}
		}
		// int closest_point = static_cast<int>(std::distance(ranges.begin(), std::min_element(ranges.begin(), ranges.end())));
		float min_dist = ranges[closest_point];

		float one_index_increment = min_dist * tan(angle_increment);

		if (one_index_increment == 0.0f)
		{
			RCLCPP_INFO(this->get_logger(), "About to divide by 0");
			// Avoid division by zero if angle_increment results in zero increment
			return;
		}

		int bubble_radius = static_cast<int>((safety_bubble_size / 2.0) / one_index_increment);
		int min_index = std::max(0, closest_point - bubble_radius);
		int max_index = std::min(static_cast<int>(ranges.size()) - 1, closest_point + bubble_radius);
		std::fill(ranges.begin() + min_index, ranges.begin() + max_index, 0);

		// zero out the array within the distance
		// float one_index_increment = min_dist * sin(angle_increment);
		// float cur_distance = 0.0;
		// int cur_index = 0;
		// while ((cur_distance < (safety_bubble_size / 2.0)) && (size_t(min_index + cur_index) < ranges.size()) && (min_index - cur_index >= 0))
		// {
		// 	// RCLCPP_INFO(this->get_logger(),"While %f is less than %f, Cur_INDEX IS %d",cur_distance, safety_bubble_size / 2.0, cur_index);
		// 	ranges[min_index + cur_index] = 0.0;
		// 	ranges[min_index - cur_index] = 0.0;
		// 	// RCLCPP_INFO(this->get_logger(),"Ranges that set to zero are %f and %f", ranges[min_index + cur_index], ranges[min_index-cur_index]);
		// 	cur_index++;
		// 	cur_distance += one_index_increment;
		// }
		// // loop through half the size of safety bubble zeroing the array

		// return;
	}

	void disparity_extender(std::vector<float> &ranges, float angle_increment)
	{
		// loop through array if the difference between two points is greater than some thresh hold
		// then update the distance of the closer one into the farther range
		for (size_t i = 0; i < (ranges.size() - 1); i++)
		{
			if (abs(ranges[i] - ranges[i + 1]) > disparity_extender_thresh)
			{
				// extend the threshhold
				if (ranges[i] < ranges[i + 1])
				{
					extend_gap(ranges, angle_increment, i, 1);
					RCLCPP_INFO(this->get_logger(), "extending disparity right");
				}
				else
				{
					extend_gap(ranges, angle_increment, i + 1, -1);
					RCLCPP_INFO(this->get_logger(), "extending disparity left");
				}
			}
		}
	}

	int extend_gap(std::vector<float> &ranges, float angle_increment, int index, int direction)
	{
		// 1 = right, -1 = left. direction to extend the disparity
		float one_index_increment = ranges[index] * tan(angle_increment);
		float cur_distance = 0.0;
		int cur_index = 0;

		while (cur_distance < (car_width / 2.0) && (size_t(index + cur_index * direction) < ranges.size()) && (index - cur_index * direction >= 0))
		{
			ranges[index + cur_index * direction] = ranges[index];
			cur_index++;
			cur_distance += one_index_increment;
		}

		return (direction == 1) ? index + cur_index : index - cur_index;
	}

	std::pair<int, int> find_max_gap(const std::vector<float> &free_space_ranges)
	{
		std::vector<int> gap_starts, gap_ends;
		bool in_gap = false;
		for (size_t i = 0; i < free_space_ranges.size(); i++)
		{
			if (free_space_ranges[i] > 0 && !in_gap)
			{
				gap_starts.push_back(i);
				in_gap = true;
			}
			if (free_space_ranges[i] == 0 && in_gap)
			{
				gap_ends.push_back(i - 1);
				in_gap = false;
			}
		}
		if (gap_starts.size() != gap_ends.size())
			gap_ends.push_back(free_space_ranges.size() - 1);

		int max_gap_size = 0;
		int chosen_start = 0;
		int chosen_end = 0;
		for (size_t i = 0; i < gap_starts.size(); i++)
		{
			int gap_size = gap_ends[i] - gap_starts[i];
			if (gap_size > max_gap_size)
			{
				max_gap_size = gap_size;
				chosen_start = gap_starts[i];
				chosen_end = gap_ends[i];
			}
		}
		return {chosen_start, chosen_end};
	}

	int find_best_point(int start_i, int end_i, const std::vector<float> &ranges)
	{
		std::vector<float> averaged_max_gap(end_i - start_i);
		for (int i = start_i; i < end_i; i++)
		{
			int window_start = std::max(0, i - best_point_conv_size / 2);
			int window_end = std::min(static_cast<int>(ranges.size()), i + best_point_conv_size / 2 + 1);
			int window_size = window_end - window_start;

			float window_sum = std::accumulate(ranges.begin() + window_start, ranges.begin() + window_end, 0.0);
			averaged_max_gap[i - start_i] = window_sum / window_size;
		}

		return std::distance(averaged_max_gap.begin(), std::max_element(averaged_max_gap.begin(), averaged_max_gap.end())) + start_i;
	}

	double get_angle(int range_index, int range_len, double angle_increment, double min_angle)
	{

		// double lidar_angle = (range_index * angle_increment) + min_angle; // index * increment + min
		double lidar_angle = (range_index - range_len / 2.0) * (2 * M_PI / range_len);
		range_len = range_len;
		double steering_angle = lidar_angle;
		if (steering_angle > max_steer)
		{
			steering_angle = max_steer;
		}
		else if (steering_angle < -max_steer)
		{
			steering_angle = -max_steer;
		}

		return steering_angle;
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ReactiveFollowGap>());
	rclcpp::shutdown();
	return 0;
}
