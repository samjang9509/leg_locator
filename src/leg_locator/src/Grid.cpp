#include "leg_locator/Grid.hpp"

void Grid_map::segGrid(std::vector<cv::Point2f> &_laser_pt, std::vector<std::pair<int, cv::Point2f>> &grid)
{
	seg_grid.setTo(255);
	odom_grid.setTo(255);

	if (vizualizer)
	{
		int clusterSize;
		int actual_label = 0;

		cv::line(seg_grid, cv::Point(grid_robot_col, 0), cv::Point(grid_robot_col, (grid_robot_col * 2)), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		cv::line(seg_grid, cv::Point(0, (grid_robot_col)), cv::Point((grid_robot_col * 2), grid_robot_col), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		cv::circle(seg_grid, cv::Point2f(grid_robot_col, grid_robot_row), 4, cv::Scalar(0, 0, 255), -1);

		cv::line(odom_grid, cv::Point(grid_robot_col, 0), cv::Point(grid_robot_col, (grid_robot_col * 2)), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		cv::line(odom_grid, cv::Point(0, (grid_robot_col)), cv::Point((grid_robot_col * 2), grid_robot_col), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
		cv::circle(odom_grid, cv::Point2f(grid_robot_col, grid_robot_row), 4, cv::Scalar(0, 0, 255), -1);
		
		int point_size = grid.size();
		int laser_size = _laser_pt.size();
		int target_id;
		int check_id;
		int num_points = 0;

		float min_distance = 0.0f;

		cv::Point2f target;
		cv::Point2f grid_target;
		cv::Point2f tmp_grid_target(0.0f, 0.0f);

		points_vector.resize(point_size);

		points_vector = grid;
		for (int i = 0; i < laser_size; i++)
		{
			cv::Point2f points;
			cv::Point2f Opoints;
			points.x = -(_laser_pt[i].y * mm2pixel) + grid_robot_col;
			points.y = -(_laser_pt[i].x * mm2pixel) + grid_robot_row;
			cv::circle(seg_grid, points, 2, cv::Scalar(0, 0, 0), -1);
			tmp_target = laser2Odom(_laser_pt[i], odomPt);
			abs_laser_points = odomPt.absol2grid(tmp_target.x, tmp_target.y, tmp_target.th, (double)grid_robot_col, (double)grid_robot_row, (double)mm2pixel);
			Opoints.x = abs_laser_points.x;
			Opoints.y = abs_laser_points.y;
			cv::circle(odom_grid, Opoints, 2, cv::Scalar(0, 0, 0), -1);
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
				abs_laser_points = odomPt.absol2grid(abs_target.x, abs_target.y, abs_target.th, (double)grid_robot_col, (double)grid_robot_row, (double)mm2pixel);
				abs_target_odom.x = abs_laser_points.x;
				abs_target_odom.y = abs_laser_points.y;

				std::stringstream oo;
				oo << grid_id;
				cv::String o_id = oo.str();

				cv::circle(odom_grid, abs_target_odom, 2, cv::Scalar(0, 0, 255), -1);
				cv::circle(odom_grid, abs_target_odom, 30, cv::Scalar(0, 0, 255), 2);
				cv::putText(odom_grid, o_id, abs_target_odom, 1, 3, cv::Scalar(0, 0, 255), 3);
			}
		}
		cv::imshow("leg_detector", seg_grid);
		cv::imshow("ODOM", odom_grid);
	}
	points_vector.clear();
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
	double d2r = 3.141592 / 180.0f;
	double base_radian_th = odomPoint.robot.th * d2r;

	Odom output;

	output.x = ((double)laser_pt.x * cos(base_radian_th)) - ((double)laser_pt.y * sin(base_radian_th)) + odomPoint.robot.x;
	output.y = ((double)laser_pt.x * sin(base_radian_th)) + ((double)laser_pt.y * cos(base_radian_th)) + odomPoint.robot.y;

	return output;
}