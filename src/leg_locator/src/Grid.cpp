#include "leg_locator/Grid.hpp"

void Grid_map::segGrid(std::vector<cv::Point2f> &_laser_pt, std::vector<std::pair<int, cv::Point2f>> &grid)
{
	seg_grid.setTo(130);
	odom_grid.setTo(130);

	if (vizualizer)
	{
		double base_radian_th = odomPt.robot.th * d2r;
		int clusterSize;
		int actual_label = 0;


		
		int point_size = grid.size();
		int laser_size = _laser_pt.size();
		int target_id;
		int check_id;
		int num_points = 0;

		float min_distance = 0.0f;

		cv::Point2f Opoint;
		cv::Point2f target;
		cv::Point2f grid_target;
		cv::Point2f tmp_grid_target(0.0f, 0.0f);
		cv::Point2f init_point(0.0f, 0.0f);
		cv::Point2f robot_tf_x(0.0f, 0.0f);
		cv::Point2f robot_tf_y(0.0f, 0.0f);
		
		cv::line(seg_grid, cv::Point(grid_robot_col, 0), cv::Point(grid_robot_col, (grid_robot_col * 2)), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		cv::line(seg_grid, cv::Point(0, (grid_robot_col)), cv::Point((grid_robot_col * 2), grid_robot_col), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		cv::circle(seg_grid, cv::Point2f(grid_robot_col, grid_robot_row), 4, cv::Scalar(0, 0, 255), -1);
		
		// Robot to pixel
		cv::Rect robot;
		robot.x = (grid_robot_col - 20);
		robot.y = (grid_robot_row - 20);
		robot.width = 40;
		robot.height = 40;
		cv::rectangle(odom_grid, robot, cv::Scalar(255,0,0), 2);
		points_vector.resize(point_size);
		// 

		// Initial point
		init_point.x = grid_robot_col - ((init_robot_coor.y - odomPt.robot.y) * mm2pixel); 
		init_point.y = grid_robot_row - ((init_robot_coor.x - odomPt.robot.x) * mm2pixel); 
		

		points_vector = grid;

		cv::line(odom_grid, cv::Point(init_point.x, 0), cv::Point(init_point.x, grid_row), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		cv::line(odom_grid, cv::Point(0, init_point.y), cv::Point(grid_col, init_point.y), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		cv::circle(odom_grid, init_point, 4, cv::Scalar(0, 0, 255), -1);
		// 

		// axis
		cv::Point2f axis_x(500.0f, 0.0f);
		cv::Point2f axis_y(0.0f, 500.0f);
		tmp_axis = laser2Odom(axis_x, odomPt);
		robot_tf_y.x = grid_robot_col - ((tmp_axis.y - odomPt.robot.y) * mm2pixel);
		robot_tf_y.y = grid_robot_row - ((tmp_axis.x - odomPt.robot.x) * mm2pixel);

		tmp_axis = laser2Odom(axis_y, odomPt);
		robot_tf_x.x = grid_robot_col - ((tmp_axis.y - odomPt.robot.y) * mm2pixel);
		robot_tf_x.y = grid_robot_row - ((tmp_axis.x - odomPt.robot.x) * mm2pixel);

		cv::line(odom_grid, cv::Point(grid_robot_col, grid_robot_row), cv::Point(robot_tf_x.x, robot_tf_x.y), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
		cv::line(odom_grid, cv::Point(grid_robot_col, grid_robot_row), cv::Point(robot_tf_y.x, robot_tf_y.y), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		// 

		for (int i = 0; i < laser_size; i++)
		{
			cv::Point2f points;
			cv::Point2f laser_Opoints;
			points.x = -(_laser_pt[i].y * mm2pixel) + grid_robot_col;
			points.y = -(_laser_pt[i].x * mm2pixel) + grid_robot_row;
			cv::circle(seg_grid, points, 2, cv::Scalar(0, 0, 0), -1);
			abs_laser_points = laser2Odom(_laser_pt[i], odomPt);
			laser_Opoints.x = grid_robot_col - ((abs_laser_points.y - odomPt.robot.y) * mm2pixel);
			laser_Opoints.y = grid_robot_row - ((abs_laser_points.x - odomPt.robot.x) * mm2pixel);
			cv::circle(odom_grid, laser_Opoints, 2, cv::Scalar(0, 0, 0), -1);
		}
		

		std::sort(points_vector.begin(), points_vector.end(), [&](std::pair<int, cv::Point2f> a, std::pair<int, cv::Point2f> b)
				  { return euclidean_distance(a.second) < euclidean_distance(b.second); });

		target_id = points_vector[0].first;

		std::sort(points_vector.begin(), points_vector.end(), [&](std::pair<int, cv::Point2f> a, std::pair<int, cv::Point2f> b)
				  { return a.first < b.first; });
		

		check_id = points_vector[0].first;

		for (int j = 0; j < point_size; j++)
		{
			if (check_id == points_vector[j].first)
			{

				tmp_grid_target.x = tmp_grid_target.x + points_vector[j].second.x;
				tmp_grid_target.y = tmp_grid_target.y + points_vector[j].second.y;
				num_points++;
			}
			else
			{
				grid_target.x = -((tmp_grid_target.y / num_points) * mm2pixel) + grid_robot_row;
				grid_target.y = -((tmp_grid_target.x / num_points) * mm2pixel) + grid_robot_col;

				num_points = 0;
				tmp_grid_target = cv::Point2f(0.0f, 0.0f);

				std::stringstream ss;
				ss << check_id;
				cv::String id = ss.str();

				if (check_id == grid_id)
				{
					cv::putText(seg_grid, id, grid_target, 1, 3, cv::Scalar(0, 0, 255), 3);
					cv::circle(seg_grid, grid_target, 30, cv::Scalar(0, 0, 255), 2);
				}
				else
				{
					cv::putText(seg_grid, id, grid_target, 1, 3, cv::Scalar(255, 0, 0), 3);
					cv::circle(seg_grid, grid_target, 30, cv::Scalar(255, 0, 0), 2);
				}
				check_id = points_vector[j].first;
			}

			if (j == (point_size - 1))
			{
				grid_target.x = -((tmp_grid_target.y / num_points) * mm2pixel) + grid_robot_row;
				grid_target.y = -((tmp_grid_target.x / num_points) * mm2pixel) + grid_robot_col;

				num_points = 0;
				tmp_grid_target = cv::Point2f(0.0f, 0.0f);

				std::stringstream ss;
				ss << check_id;
				cv::String id = ss.str();

				if (check_id == grid_id)
				{
					cv::putText(seg_grid, id, grid_target, 1, 3, cv::Scalar(0, 0, 255), 3);
					cv::circle(seg_grid, grid_target, 30, cv::Scalar(0, 0, 255), 2);
				}
				else
				{
					cv::putText(seg_grid, id, grid_target, 1, 3, cv::Scalar(255, 0, 0), 3);
					cv::circle(seg_grid, grid_target, 30, cv::Scalar(255, 0, 0), 2);
				}
				cv::Point2f abs_target_odom;
			}
		} 
		cv::imshow("leg_detector", seg_grid);
		cv::imshow("ODOM", odom_grid);
	}
	points_vector.clear();
}

void Grid_map::initGrid(std::vector<cv::Point2f> &_laser_pt)
{
	init_grid.setTo(255);
	

	if (vizualizer)
	{
		cv::line(init_grid, cv::Point(grid_robot_col, 0), cv::Point(grid_robot_col, (grid_robot_col * 2)), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		cv::line(init_grid, cv::Point(0, (grid_robot_col)), cv::Point((grid_robot_col * 2), grid_robot_col), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		cv::circle(init_grid, cv::Point2f(grid_robot_col, grid_robot_row), 4, cv::Scalar(0, 0, 255), -1);
		cv::circle(init_grid, cv::Point2f(grid_robot_col, grid_robot_row - 75.0f), 25, cv::Scalar(0, 0, 255), 2);
		
		int clusterSize;
		int actual_label = 0;

		int laser_size = _laser_pt.size();
		int target_id;
		int check_id;
		int num_points = 0;

		float min_distance = 0.0f;

		cv::Point2f target;
		cv::Point2f grid_target;
		cv::Point2f tmp_grid_target(0.0f, 0.0f);

		for (int i = 0; i < laser_size; i++)
		{
			cv::Point2f points;
			cv::Point2f Opoints;
			points.x = -(_laser_pt[i].y * mm2pixel) + grid_robot_col;
			points.y = -(_laser_pt[i].x * mm2pixel) + grid_robot_row;
			cv::circle(init_grid, points, 2, cv::Scalar(0, 0, 0), -1);		
		}		
		cv::imshow("initial_map", init_grid);
	}
	init_robot_coor = odomPt.robot;
}

float inline Grid_map::euclidean_distance(cv::Point2f check_distance)
{
	return std::sqrt(pow((check_distance.x), 2) + pow((check_distance.y), 2));
}

cv::Point2f Grid_map::pt2Grid(float x_co, float y_co)
{
	float grid_x = grid_robot_row - y_co * mm2pixel;
	float grid_y = grid_robot_col - x_co * mm2pixel;

	cv::Point2f output(grid_x, grid_y);
	return output;
	cv::Point2f pt2Grid(float x_co, float y_co);
}

Odom Grid_map::laser2Odom(cv::Point2f laser_pt, OdoManager &odomPoint)
{
	double base_radian_th = odomPoint.robot.th * d2r;

	Odom output;

	output.x = ((double)laser_pt.x * cos(base_radian_th)) - ((double)laser_pt.y * sin(base_radian_th)) + odomPoint.robot.x;
	output.y = ((double)laser_pt.x * sin(base_radian_th)) + ((double)laser_pt.y * cos(base_radian_th)) + odomPoint.robot.y;

	return output;
}