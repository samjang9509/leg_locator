#include "leg_locator/core.hpp"

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
	leg_sub = nh.subscribe<leg_tracker::PersonArray>("people_tracked", 1, &leg_locator::leg_CB, this);
}


std::vector<cv::Point2f> leg_locator::initialize_scan()
{
	point_m.clear();
	return dst_v;
}

std::vector<std::pair<int, cv::Point2f>> leg_locator::initialize_leg()
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
		dst_v = point_m;
	}
	point_m.clear();
}

void leg_locator::leg_CB(const leg_tracker::PersonArray::ConstPtr &person)
{
	std::vector<std::pair<int, cv::Point2f>> tmp_dst;


	leg_tracker::PersonArray tmp_person;

	tmp_person.header = person->header;
	tmp_person.people = person->people;

	int person_size = tmp_person.people.size();
	if (person_size == 0)
	{
		ROS_INFO("No Deteced Person");
	}
	else
	{
		tmp_dst.resize(person_size);

		for (int i = 0; i < person_size; i++)
		{
			
				tmp_dst[i].second.x = tmp_person.people[i].pose.position.x;
				tmp_dst[i].second.y = tmp_person.people[i].pose.position.y;
				tmp_dst[i].first = (int)tmp_person.people[i].id;
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

void leg_locator::segmentation(std::vector<cv::Point2f> &_laser_pt, std::vector<std::pair<int, cv::Point2f>> &_leg_pt)
{
	std::vector<cv::Point2f> tmp_laser;
	std::vector<std::pair<int, cv::Point2f>> tmp_person;

	if (_laser_pt.empty() || _leg_pt.empty())
	{
		ROS_INFO("no input");
	}
	else
	{
		l_mutex.lock();
		_laser_pt.swap(tmp_laser);
		_leg_pt.swap(tmp_person);
		l_mutex.unlock();

		int v_size = tmp_laser.size();
		int leg_size = tmp_person.size();
		final_clusters.resize(leg_size);

		for (int j = 0; j < leg_size; j++)
		{

			cv::Point2f tmp_target;
			tmp_target.x = tmp_person[j].second.x * m2mm;
			tmp_target.y = tmp_person[j].second.y * m2mm;
			final_clusters[j].label = tmp_person[j].first;

			for (int k = 0; k < v_size; k++)
			{
				float distance = ed_btw_points(tmp_target, tmp_laser[k]);
				// std::cout << distance << std::endl;
			
				if (distance <= 300.0f)
				{
					final_clusters[j].body.push_back(tmp_laser[k]);
				}
			}
		}
		if(final_clusters.size() == 0)
		{
			std::cout << "something is wrong" << std::endl;
		}
		else
		{
			catch_target(final_clusters);
			final_clusters.clear();
		}
	}
}

void leg_locator::catch_target(std::vector<Cluster> leg_target)
{	std::pair<int,cv::Point2f> tmp_grid;
	std::vector<std::pair<int,cv::Point2f>> grid;
	int cluster_num = leg_target.size();
	cv::Point2f zero(0.0f, 0.0f);
	float min_distance = 0.0f;

	cv::Point2f target_mean;

	if(cluster_num == 0)
	{
		ROS_INFO("No incoming body");	
	}
	else
	{
		for (int i = 0; i < cluster_num; i++)
		{
			cv::Point2f target_sum = std::accumulate(leg_target[i].body.begin(), leg_target[i].body.end(), zero);
			int body_size = leg_target[i].body.size();

			target_mean.x = target_sum.x / body_size;
			target_mean.y = target_sum.y / body_size;

			int laser_data_num = leg_target[i].body.size();
			// grid.resize(laser_data_num * cluster_num);
			for (int j = 0; j < laser_data_num; j++)
			{
				tmp_grid.second = leg_target[i].body[j];
				tmp_grid.first = leg_target[i].label;

				// std::cout << "tmp_grid id : "<< tmp_grid.first << std::endl;

				grid.push_back(tmp_grid);

				// std::cout << "grid id : " << grid[j].first << std::endl;
				// printf("vector size : %d \n", (int)grid.size());
				// printf("last index : %d \n", grid.back().first);

				float distance2target = euclidean_distance(target_mean);
				if (min_distance == 0.0f)
				{
					min_distance = distance2target;
					final_target = target_mean;
				}
				else if (min_distance > distance2target)
				{
					min_distance = distance2target;
					final_target = target_mean;
				}
				else if (min_distance < distance2target)
				{
					continue;
				}
			}
		}

		Control.move2target(final_target);
		vizual.segGrid(grid);
		grid.clear();
	}

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
	return std::sqrt(pow(first.x - second.x, 2) + pow(first.y - second.y, 2));
}

float inline leg_locator::euclidean_distance(cv::Point2f target)
{
	return std::sqrt(pow(target.x, 2) + pow(target.y, 2));
}


void leg_locator::runloop()
{

	thread_list.push_back(std::move(std::thread([this]
												{
        ros::Rate hz(20);
        while(ros::ok()){
            src_laser = std::move(initialize_scan());
			src_person = std::move(initialize_leg());

				segmentation(src_laser, src_person);

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
