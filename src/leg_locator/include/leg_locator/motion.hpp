#include "leg_locator/systems.hpp"
#include "leg_locator/receiver.hpp"
#include "leg_locator/OdomManager.hpp"

#define mm2m(x) (x/1000)

class motion_control
{
private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;

    float ed_meter(cv::Point2f d_target);
    double velocity(float distance);

    double max_lin_vel;
    double min_lin_vel;
    double max_ang_vel;
    double min_ang_vel;
    float safe_distance;
    
public:

    void init_Publisher();
    void move2target(cv::Point2f p_target);
    // void param_callback();

    motion_control() : max_lin_vel(0.7), min_lin_vel(0.1),
    safe_distance(0.0008f), min_ang_vel(0.05), max_ang_vel(0.2)
    {
        // param_callback();
        this->init_Publisher();
        
    }
    ~motion_control()
    {

    }
};