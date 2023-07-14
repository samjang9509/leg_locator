#include "leg_particle/system.hpp"
#include "leg_particle/receiver.hpp"

class core
{
    private:
    
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub;
    std::mutex mutex;
    std::deque<std::thread> thread_list;
    receiver l_receiver;

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void runloop();
    void destructor();
    public:
    std::vector<cv::Point2f> l_tmp_point;
    std::vector<cv::Point2f> l_point;
    std::vector<cv::Point2f> init_laser();
    void laser_callback();


    core()
    {
        this->runloop();
    }
    ~core()
    {

    }
};