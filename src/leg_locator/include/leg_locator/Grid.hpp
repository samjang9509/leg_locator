#include "leg_locator/systems.hpp"
#include "leg_locator/receiver.hpp"
#include "leg_locator/OdomManager.hpp"


#define RAD2DEG(x) ((x)*180. / M_PI)
#define DEG2RAD(x) ((x)*M_PI / 180.)

class Grid_map
{
private:
	ros::NodeHandle nh_;
public:
	std::string this_name;
	cv::Mat Grid;
	cv::Mat occup;
	cv::Mat seg_grid;

	OdoManager odomGrp;

	Cona_Odom tmp_target;
	Cona_Odom abs_target;

	int grid_robot_col = 500;
	int grid_robot_row = 500;
	float mm2pixel = 100.0f / 1000.0f;

	int grid_row = 1000;
	int grid_col = 1000;

	bool vizualizer;

    void m_segGrid(std::vector<cv::Point2f> &grid);

    cv::Point2f pt2Grid(float x_co, float y_co);
public:
	Grid_map() : this_name("Grid_map"), vizualizer(true)
	{
		Grid = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0, 0, 0));
		occup = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0, 0, 0));
		seg_grid = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0, 0, 0));
	}
	~Grid_map()
	{
	}
};