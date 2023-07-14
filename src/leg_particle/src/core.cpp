#include "leg_particle/core.hpp"

void core::laser_callback()
{
	laser_sub = nh_.subscribe<sensor_msgs::LaserScan>("/scan_multi", 1, &core::scan_callback, this);
}

std::vector<cv::Point2f> core::init_laser()
{
	l_tmp_point.clear();
	return l_point;
}

void core::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	mutex.lock();
	l_receiver.get_tf(msg->header.frame_id);

	int size = std::min((int)msg->ranges.size(), 1440); // TG = 2019, Sick
	float angle_min = msg->angle_min;
	float angle_max = msg->angle_max;
	float angle_increment = msg->angle_increment;
	float range_min = (float)msg->range_min;
	float range_max = (float)msg->range_max;

	// for every data within the size
	for (int i = 0; i < size; i++)
	{
		float val = msg->ranges[i]; // ranges[i]?
		if (val <= range_min || val >= range_max || !std::isfinite(val) || val == 0.0)
			continue;
		float angle = angle_min + angle_increment * (float)i; 
		float degree = angle * 180.0f / CV_PI;
		float x = cos(angle) * val;
		float y = sin(angle) * val;
		tf::Vector3 p(x, y, 0);
		tf::Vector3 reprj_p = l_receiver.R * p + l_receiver.T; 
		cv::Point2f tmp_pt;
		tmp_pt.x = 1000.0f * reprj_p.getX();
		tmp_pt.y = 1000.0f * reprj_p.getY();

		l_tmp_point.push_back(tmp_pt);
		l_tmp_point.swap(l_point);
	}
	l_tmp_point.clear();
	
	mutex.unlock();
}

void core::runloop()
{
    thread_list.push_back(std::move(std::thread([this]
    {
        ros::Rate hz(20);
        while(ros::ok())
        {


            cv::waitKey(1);
            ros::spinOnce();
            hz.sleep();
        }
    })))
};

void core::destructor()
{
    ROS_INFO("destroy thread!");
    for(int i = 0; i < thread_list.size(); i++)
    thread_list[i].join();
}