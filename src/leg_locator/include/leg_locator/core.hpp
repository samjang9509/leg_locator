#include "leg_locator/Grid.hpp"
#include "leg_locator/motion.hpp"

class Cluster
{
public:
	int label;
	std::vector<cv::Point2f> body;

	Cluster() : label(0)
	{
	}
	~Cluster()
	{
	}
};

class Group
{
public:
	std::vector<Cluster> grouping;
	int groupSize;

	Group() : groupSize(0)
	{
	}
	~Group()
	{
	}
};

class Leg_cluster
{
    public:
    std::deque<leg_tracker::Leg> target_leg;
    std_msgs::Header leg_header;

    Leg_cluster()
    {
    }
    ~Leg_cluster()
    {
    }
};

class leg_locator
{
public:
    std::string this_name;
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub_;
    ros::Subscriber leg_sub;

    Cona_Odom odomCo;
    OdoManager odomPt;
    receiver s_receiver;
    Grid_map vizual;
    motion_control Control;

    std::mutex l_mutex;

    std::vector<cv::Point2f> point_c_tmp;
    std::vector<cv::Point2f> src_laser;
    std::vector<cv::Point2f> dst_v;

    std::vector<ros::Subscriber> scan_subscribers;
    std::vector<std::thread> thread_list;
    std::vector<cv::Point2f> point_m;

    std::vector<cv::Point2f> src_leg;
    Leg_cluster dst_clusters;

    std::vector<cv::Point2f> dst_points;

    std::vector<Cluster> final_clusters;

private:
   
    Cona_Odom tmp_target;
    Cona_Odom abs_target;

    cv::Point2f final_target;

	float m2mm = 1000.0f;

    Cona_Odom laser2Odom(cv::Point2f laser_pt, OdoManager &odomPoint);
    std::vector<cv::Point2f> initialize_scan();
    std::vector<cv::Point2f> initialize_leg();
    float inline ed_btw_points(cv::Point2f first, cv::Point2f second);
    float inline euclidean_distance(cv::Point2f target);
    

    void segmentation(std::vector<cv::Point2f> &_laser_pt, std::vector<cv::Point2f> &_leg_pt);
    void catch_target(std::vector<Cluster> leg_target);

    void scan_CB(const sensor_msgs::LaserScan::ConstPtr &msg);
    void leg_CB(const leg_tracker::LegArrayConstPtr &leg);

    void laserscan_topic_parser();
    void runloop();
    void destructor();
    void odom_subscriber();
    void leg_subscriber();

public:
    leg_locator() : this_name("leg_locator")
    {
        this->odom_subscriber();
        this->leg_subscriber();
        this->laserscan_topic_parser();
        this->runloop();
    }
    ~leg_locator()
    {
        destructor();
    }
};