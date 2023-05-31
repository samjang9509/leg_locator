#include "leg_locator/Grid.hpp"

void Grid_map::segGrid(std::vector<cv::Point2f> &grid)
{
	seg_grid.setTo(255);

	int clusterSize;
	int actual_label = 0;

	cv::line(seg_grid, cv::Point(grid_robot_col, 0), cv::Point(grid_robot_col, (grid_robot_col * 2)), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
	cv::line(seg_grid, cv::Point(0, (grid_robot_col)), cv::Point((grid_robot_col * 2), grid_robot_col), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
	cv::circle(seg_grid, cv::Point2f(grid_robot_col, grid_robot_row), 4, cv::Scalar(0, 0, 255), -1);
	if (vizualizer == true)
	{

		int point_size = grid.size();
		float min_distance = 0.0f;
		cv::Point2f target;
		
		for(int i = 0; i < point_size; i++)
		{
			cv::Point2f points;
			points.x = (grid[i].x * mm2pixel) + grid_robot_col;
			points.y = (grid[i].y * mm2pixel) + grid_robot_row;
			cv::circle(seg_grid, points, 2, cv::Scalar(0,0,0), -1);
			float distance = euclidean_distance(points, robot);
			if(min_distance == 0.0f)
			{
				min_distance = distance;
				target = points;
			}
			else if(min_distance > distance)
			{
				min_distance = distance;
				target = points;
			}
			else
			{
				continue;
			}
		}
		cv::circle(seg_grid, target, 15, cv::Scalar(0,0,255), 2);
		
		cv::imshow("leg_detector", seg_grid);
	}
}

float Grid_map::euclidean_distance(cv::Point2f check_distance, cv::Point2f origin)
{
	float output;

	output = std::sqrt(pow((check_distance.x - origin.x),2) + pow((check_distance.y - origin.y),2));

	return output;
}
cv::Point2f Grid_map::pt2Grid(float x_co, float y_co)
{
	float grid_x = grid_robot_row - y_co * mm2pixel;
	float grid_y = grid_robot_col - x_co * mm2pixel;

	cv::Point2f coordinate(grid_x, grid_y);
	return coordinate;
}