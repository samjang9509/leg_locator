#include "leg_locator/core.hpp"

std::vector<cv::Point2f> leg_locator::initialize_scan()
{
	point_m.clear();
	return dst_v;
}

std::vector<cv::Point2f> leg_locator::initialize_leg()
{
	return dst_points;
}

void leg_locator::scan_CB(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	s_receiver.get_tf(msg->header.frame_id);

	int size = std::min((int)msg->ranges.size(), 1440); 
	float angle_min = msg->angle_min;
	float angle_max = msg->angle_max;
	float angle_increment = msg->angle_increment;
	float range_min = (float)msg->range_min;
	float range_max = (float)msg->range_max;

	for (int i = 0; i < size; i++)
	{
		float val = msg->ranges[i];
		if (val <= range_min || val >= range_max || !std::isfinite(val) || val == 0.0)
			continue;
		float angle = angle_min + angle_increment * (float)i; 
		float degree = angle * 180.0f / CV_PI;				  
		float x = cos(angle) * val;							  
		float y = sin(angle) * val;							  
		tf::Vector3 p(x, y, 0);
		tf::Vector3 reprj_p = s_receiver.R * p + s_receiver.T;
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
	std::vector<cv::Point2f> tmp_dst;

	leg_tracker::LegArray tmp_leg;
	
	tmp_leg.header = leg->header;
	tmp_leg.legs = leg->legs;
	
	int leg_size = tmp_leg.legs.size();
	if(leg_size == 0)
	{
		ROS_INFO("No Leg");
	}
	else
	{
		tmp_dst.resize(tmp_leg.legs.size());

		for (int i = 0; i < leg_size; i++)
		{
			float confidence_level = tmp_leg.legs[i].confidence;
			if (confidence_level > 0.5f)
			{
				tmp_dst[i].x = tmp_leg.legs[i].position.x;
				tmp_dst[i].y = tmp_leg.legs[i].position.y;
			}
		}
		int size = tmp_dst.size();
		if (size == 0)
		{
			std::cout << "Searching for leg input" << std::endl;
		}
		else
		{
			dst_points.swap(tmp_dst);
		}
		tmp_dst.clear();
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

float inline leg_locator::ed_btw_points(cv::Point2f first, cv::Point2f second)
{
	return std::sqrt(pow(first.x - second.x,2) + pow(first.y - second.y,2));
}

float inline leg_locator::euclidean_distance(cv::Point2f target)
{
	return std::sqrt(pow(target.x,2) + pow(target.y,2));
}

void leg_locator::segmentation(std::vector<cv::Point2f> &_laser_pt, std::vector<cv::Point2f> &_leg_pt)
{
	std::vector<cv::Point2f> tmp_laser;
	std::vector<cv::Point2f> tmp_leg;
	
	if(_laser_pt.empty() || _leg_pt.empty())
	{
		ROS_INFO("no input");
	}
	else
	{
		l_mutex.lock();
		_laser_pt.swap(tmp_laser);
		_leg_pt.swap(tmp_leg);
		l_mutex.unlock();

		int v_size = tmp_laser.size();
		int leg_size = tmp_leg.size();

		std::cout << "v_size : " << v_size << std::endl;
		std::cout << "leg_siz : " << leg_size << std::endl;

		for (int j = 0; j < leg_size; j++)
		{
			cv::Point2f tmp_target;
			tmp_target.x = tmp_leg[j].x * m2mm;
			tmp_target.y = tmp_leg[j].y * m2mm;

			for (int k = 0; k < v_size; k++)
			{
				std::cout << tmp_target << std::endl;
				std::cout << tmp_laser[k] << std::endl;
				float distance = ed_btw_points(tmp_target, tmp_laser[k]);
				final_clusters.resize(v_size);
				if (distance < 30.0f)
				{
					final_clusters[k].body.push_back(tmp_laser[k]);
				}
			}
		}

		std::cout << "final_cluste size = " << final_clusters.size() << std::endl;
		catch_target(final_clusters);
		final_clusters.clear();
	}
}

void leg_locator::catch_target(std::vector<Cluster> leg_target)
{
	std::vector<cv::Point2f> grid;
	int cluster_num = leg_target.size();
	cv::Point2f zero(0.0f,0.0f);
	float min_distance = 0.0f;

	for(int i = 0; i < cluster_num; i++)
	{
		cv::Point2f target_mean = std::accumulate(leg_target[i].body.begin(), leg_target[i].body.end(),zero);
		
		target_mean.x = target_mean.x / cluster_num;
		target_mean.y = target_mean.y / cluster_num;

		int laser_data_num = leg_target[i].body.size();
		for(int j = 0; j < laser_data_num; j++)
		{
			grid.push_back(leg_target[i].body[j]);
			
			float distance2target = euclidean_distance(target_mean);
			if(min_distance == 0.0f)
			{
				min_distance = distance2target;
				final_target = target_mean;
			}
			else if(min_distance > distance2target)
			{
				min_distance = distance2target;
				final_target = target_mean;
			}
			else if(min_distance < distance2target)
			{
				continue;
			}
		}
	}
	Control.move2target(final_target);
	vizual.segGrid(grid);	
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

