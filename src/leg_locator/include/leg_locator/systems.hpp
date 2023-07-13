#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
// #include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <numeric>
#include <algorithm>
#include <mutex>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
// #include "leg_tracker/LegArray.h"
#include "leg_tracker/PersonArray.h"
