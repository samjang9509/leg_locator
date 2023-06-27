#include "leg_locator/systems.hpp"
#include "leg_locator/receiver.hpp"
#include "leg_locator/OdomManager.hpp"


#define RAD2DEG(x) ((x)*180. / M_PI)
#define DEG2RAD(x) ((x)*M_PI / 180.)
class Grid_map
{
private:
	ros::NodeHandle nh_;
	cv::Point2f grid_mid;
	std::vector<std::pair<int,cv::Point2f>> points_vector;
	double d2r = 3.141592 / 180.0f;

public:
	std::string this_name;
	cv::Mat odom_grid;
	cv::Mat seg_grid;
	cv::Mat init_grid;

    OdoManager odomPt;

	Odom tmp_target;
	Odom abs_laser_points;
	Odom abs_target;
	Odom init_robot_coor;

	int grid_id;

	float grid_robot_col = 500.0f;
	float grid_robot_row = 500.0f;
	float mm2pixel = 100.0f / 1500.0f;

	int grid_row = 1000;
	int grid_col = 1000;

	bool vizualizer;

	Odom laser2Odom(cv::Point2f laser_pt, OdoManager &odomPoint);

    void segGrid(std::vector<cv::Point2f> &_laser_pt, std::vector<std::pair<int,cv::Point2f>>  &grid);
	void initGrid(std::vector<cv::Point2f> &_laser_pt);
	float inline euclidean_distance(cv::Point2f check_distance);

    cv::Point2f pt2Grid(float x_co, float y_co);
public:
	Grid_map() : this_name("Grid_map"), vizualizer(true), grid_mid(grid_robot_row, grid_robot_col)
	{
		odom_grid = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0, 0, 0));
		seg_grid = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0, 0, 0));
		init_grid = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0, 0, 0));
	}
	~Grid_map()
	{
	}
};