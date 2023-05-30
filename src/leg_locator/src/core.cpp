#include "leg_locator/core.hpp"

std::vector<cv::Point2f> leg_locator::initialize_scan()
{
	point_m.clear();
	return dst_v;
}

Leg_cluster leg_locator::initialize_leg()
{
	return dst_clusters;
}

void leg_locator::scan_CB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	s_receiver.get_tf(msg->header.frame_id);

	// getting laser data within the value size (where is size?)
	int size = std::min((int)msg->ranges.size(), 1440); // TG = 2019, Sick
	float angle_min = msg->angle_min;
	float angle_max = msg->angle_max;
	float angle_increment = msg->angle_increment;
	float range_min = (float)msg->range_min;
	float range_max = (float)msg->range_max;

	for (int i = 0; i < size; i++)
	{
		float val = msg->ranges[i]; // ranges[i]?
		if (val <= range_min || val >= range_max || !std::isfinite(val) || val == 0.0)
			continue;
		float angle = angle_min + angle_increment * (float)i; // getting the angle for every point
		float degree = angle * 180.0f / CV_PI;				  // change from rad to degree (not using)
		float x = cos(angle) * val;							  // coordinate x
		float y = sin(angle) * val;							  // coordinate y
		tf::Vector3 p(x, y, 0);
		tf::Vector3 reprj_p = s_receiver.R * p + s_receiver.T; // Rotation and Transformation
		cv::Point2f tmp_pt;
		tmp_pt.x = 1000.0f * reprj_p.getX();
		tmp_pt.y = 1000.0f * reprj_p.getY();

		point_m.push_back(tmp_pt);
		dst_v= point_m;
	}
	point_m.clear();
}

void leg_locator::leg_CB(const leg_tracker::LegArrayConstPtr &leg)
{
	std::cout << "WTF" << std::endl;
	Leg_cluster tmp_target;

	leg_tracker::LegArray tmp_leg;

	
	tmp_leg.header = leg->header;
	tmp_leg.legs = leg->legs;
	
	int leg_size = tmp_leg.legs.size();
	
	for(int i = 0; i < leg_size; i++)
	{
		float confidence_level = tmp_leg.legs[i].confidence;
		if(confidence_level > 0.5f)
		{
			tmp_target.target_leg.push_back(tmp_leg.legs[i]);
			tmp_target.leg_header = tmp_leg.header;
		}
	}

}

void leg_locator::laserscan_topic_parser()
{
	scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan_multi", 1, &leg_locator::scan_CB, this);
}

void leg_locator::odom_subscriber()
{
	odom_sub_ = nh.subscribe("odom", 1, &OdoManager::odom_callback, &odomPt);
}

void leg_locator::leg_subscriber()
{
	leg_sub = nh.subscribe<leg_tracker::LegArray>("detected_leg_clusters", 1, &leg_locator::leg_CB, this);
}

Cona_Odom leg_locator::laser2Odom(cv::Point2f laser_pt, OdoManager &odomPoint)
{
	double d2r = 3.141592 / 180.0f;
	double base_radian_th = odomPoint.robot.th * d2r;

	Cona_Odom tmp_target;
	
	tmp_target.x = ((double)laser_pt.x * cos(base_radian_th)) - ((double)laser_pt.y * sin(base_radian_th)) + odomPoint.robot.x;
	tmp_target.y = ((double)laser_pt.x * sin(base_radian_th)) + ((double)laser_pt.y * cos(base_radian_th)) + odomPoint.robot.y;

	return tmp_target;
}

float leg_locator::euclidean_distance(cv::Point2f first, cv::Point2f second)
{
	float output;

	output = std::sqrt(pow(first.x - second.x,2) + pow(first.y - second.y,2));

	return output;
}


void leg_locator::segmentation(std::vector<cv::Point2f> &_laser_pt, Leg_cluster &_leg_pt)
{
	Cluster l_cluster;
	Group cluster_group;
	std::vector<cv::Point2f> tmp_laser;
	std::deque<leg_tracker::Leg> tmp_leg;
	
	_laser_pt.swap(tmp_laser);
	_leg_pt.target_leg.swap(tmp_leg);
	int v_size = tmp_laser.size();

	int leg_size = tmp_leg.size();

	for(int j = 0; j < leg_size; j++)
	{
		cv::Point2f tmp_target;
		tmp_target.x = tmp_leg[j].position.x * m2mm;
		tmp_target.y = tmp_leg[j].position.y * m2mm;

		for(int k = 0; k < v_size; k++)
		{
			float distance = euclidean_distance(tmp_target, tmp_laser[k]);
			if(distance < 30.0f)
			{
				final_clusters[k].body.push_back(tmp_laser[k]);
			}
		}
	}

	grid.segGrid(final_clusters.body)
	final_clusters.clear();
}

void leg_locator::runloop()
{

	thread_list.push_back(std::move(std::thread([this]
												{
        ros::Rate hz(20);
        while(ros::ok()){
            src_laser = std::move(initialize_scan());
			src_leg = std::move(initialize_leg());
		
				segmentation(src_laser, src_leg);

            cv::waitKey(1);
            ros::spinOnce();
            hz.sleep();
        } })));
}

void leg_locator::destructor()
{
	ROS_INFO("destroy thread!");
	for (int i = 0; i < thread_list.size(); i++)
		thread_list[i].join();
}

