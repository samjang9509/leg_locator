#include "leg_locator/Grid.hpp"

void Grid_map::segGrid(std::vector<cv::Point2f> &grid)
{
	cv::Point2f select_pt(0.0f, 0.0f);
	cv::Point2f begin(0.0f, 0.0f);
	cv::Point2f end(0.0f, 0.0f);
	cv::Point2f point_cloud;
	seg_grid.setTo(255);

	int groupSize = grid.grouping.size();

	int clusterSize;
	int actual_label = 0;

	cv::line(seg_grid, cv::Point(grid_robot_col, 0), cv::Point(grid_robot_col, (grid_robot_col * 2)), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
	cv::line(seg_grid, cv::Point(0, (grid_robot_col)), cv::Point((grid_robot_col * 2), grid_robot_col), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
	cv::circle(seg_grid, cv::Point2f(grid_robot_col, grid_robot_row), 4, cv::Scalar(0, 0, 255), -1);
	if (vizualizer == true)
	{
		int cluster_size = final_clusters.size();
		cv::Point2f zero(0.0f,0.0f);
		for(int i = 0; i < cluster_size; i++)
		{
			cv::Point2f total = std::accumulate(final_clusters[i].body.begin(), draw_leg.final_clusters[i].body.end(), zero);
			cv::circle(seg_grid, cv::Point2f(total.x/cluster_size,total.y/cluster_size), 40, cv::Scalar(255,0,255), 2);
		}
	
		cv::imshow("leg_locator", seg_grid);
	}
	draw_leg.final_clusters.clear();
}

cv::Point2f Grid_map::pt2Grid(float x_co, float y_co)
{
	float grid_x = grid_robot_row - y_co * mm2pixel;
	float grid_y = grid_robot_col - x_co * mm2pixel;

	cv::Point2f coordinate(grid_x, grid_y);
	return coordinate;
}