// #ifndef Odometry
// #define Odometry

#include "leg_locator/systems.hpp"

class Odom
{
public:
	//각도를 -180도 ~ 180도 사이로 만들어줌
	void angleRearrange(void)
	{
		while (1)
		{
			if (th < -180.0)
				th += 360.0;
			else if (th > 180.0)
				th -= 360.0;
			else
				break;
		}
	}

public:
	double x, y, th, weight;

	Odom(void)
		: x(0), y(0), th(0), weight(0){};
	Odom(double x_, double y_, double th_)
		: x(x_), y(y_), th(th_){};
	Odom addMotion(Odom motion)
	{
		double d2r = 3.141592 / 180.0;
		double base_radian_th = this->th * d2r;

		Odom output;
		output.x = this->x + motion.x * cos(base_radian_th) - motion.y * sin(base_radian_th);
		output.y = this->y + motion.x * sin(base_radian_th) + motion.y * cos(base_radian_th);
		output.th = this->th + motion.th;
		output.angleRearrange();
		return output;
	}
	Odom getMotion(Odom target)
	{
		
		Odom motion(target.x - x, target.y - y, target.th - th);
		motion.angleRearrange(); 

		double d2r = 3.141592 / 180.0;
		double base_radian_th = -th * d2r;

		Odom output;
		output.x = motion.x * cos(base_radian_th) - motion.y * sin(base_radian_th);
		output.y = motion.x * sin(base_radian_th) + motion.y * cos(base_radian_th);
		output.th = motion.th;
		return output;
	}
};

class OdoManager
{
private:
public:
	OdoManager() : flag_init(false)
	{
	}
	ros::Subscriber sub_odom;

	std::mutex robot_mtx;
	Odom robot;
	bool flag_init;

	double quaternion2yawdegree(double x, double y, double z, double w)
	{
		tf::Quaternion q(x, y, z, w);

		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		double angle = yaw / 3.141592 * 180.0;
		return angle;
	}

	Odom grid2absol(double col, double row, double th, double col_center, double row_center, double mm2grid)
	{
		double dx = (row_center - row) / mm2grid;
		double dy = (col_center - col) / mm2grid;

		robot_mtx.lock();
		Odom output = robot.addMotion(Odom(dx, dy, th));
		robot_mtx.unlock();

		return output;
	}
	Odom absol2grid(double x, double y, double th, double col_center, double row_center, double mm2grid)
	{
		robot_mtx.lock();
		Odom motion = robot.getMotion(Odom(x, y, th));
		robot_mtx.unlock();

		cv::Point2d output(
			col_center - motion.y * mm2grid,
			row_center - motion.x * mm2grid);

		return Odom(output.x, output.y, th);
	}
	nav_msgs::Odometry cona2ros(Odom input)
	{
		nav_msgs::Odometry output;
		output.pose.pose.position.x = input.x / 1000.0;
		output.pose.pose.position.y = input.y / 1000.0;

		double radian = input.th / 180.0 * 3.141592;
		output.pose.pose.orientation = tf::createQuaternionMsgFromYaw(radian);

		return output;
	}
	Odom ros2cona(nav_msgs::Odometry input)
	{
		Odom output;
		output.x = input.pose.pose.position.x * 1000.0;
		output.y = input.pose.pose.position.y * 1000.0;
		output.th = quaternion2yawdegree(
			input.pose.pose.orientation.x,
			input.pose.pose.orientation.y,
			input.pose.pose.orientation.z,
			input.pose.pose.orientation.w);

		return output;
	}

	void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
	{
		robot_mtx.lock();
		robot = ros2cona(*msg);
		// std::cout << "Catching bug (" << robot.x << ", " << robot.y << ")" << std::endl;
		if (!flag_init)
		{
			ROS_INFO("Odometry is coming! -> [OdoManager]");
			this->flag_init = true;
		}
		robot_mtx.unlock();
	}
};

// #endif