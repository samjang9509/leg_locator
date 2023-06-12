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

class Leg_cluster
{
    public:
    // std::deque<leg_tracker::Leg> target_leg;
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
    ros::Subscriber odom_sub_;
    Odom odomCo;
    OdoManager odomPt;
    receiver s_receiver;
    Grid_map vizual;
    motion_control Control;

    std::mutex l_mutex;

    std::vector<cv::Point2f> point_c_tmp;
    std::vector<cv::Point2f> src_laser;
    std::vector<cv::Point2f> dst_v;

    std::vector<std::thread> thread_list;
    std::vector<cv::Point2f> point_m;

    std::vector<std::pair<int, cv::Point2f>> src_person;
    std::vector<std::pair<int, cv::Point2f>> dst_points;

    std::vector<Cluster> final_clusters;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
    message_filters::Subscriber<leg_tracker::PersonArray> person_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, leg_tracker::PersonArray> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;

private:
   
    Odom tmp_target;
    Odom abs_target;

    cv::Point2f final_target;

	float m2mm = 1000.0f;


    Odom laser2Odom(cv::Point2f laser_pt, OdoManager &odomPoint);
    std::vector<cv::Point2f> initialize_scan();
    std::vector<std::pair<int, cv::Point2f>> initialize_leg();
    float inline ed_btw_points(cv::Point2f first, cv::Point2f second);
    float inline euclidean_distance(cv::Point2f target);

    // int initializer(std::vector<std::pair<int, cv::Point2f>> &_leg_pt);
    bool initialized;
    int target_id;

    void segmentation(std::vector<cv::Point2f> &_laser_pt, std::vector<std::pair<int, cv::Point2f>> &_leg_pt);
    void catch_target(std::vector<cv::Point2f> &_laser_pt, std::vector<Cluster> leg_target);

    void scan_CB(const sensor_msgs::LaserScan::ConstPtr &msg);
    void leg_CB(const leg_tracker::PersonArray::ConstPtr &person);
    void sync_callback(const sensor_msgs::LaserScan::ConstPtr &msg, const leg_tracker::PersonArray::ConstPtr &person);

    void laserscan_topic_parser();
    void leg_subscriber();
    void runloop();
    void destructor();
    void odom_subscriber();
    void init_subscriber();

public:
    leg_locator() : this_name("leg_locator"), initialized(false), target_id(0),
    laser_sub(nh, "/scan_multi", 1), person_sub(nh, "/people_tracked", 1), sync(MySyncPolicy(10), laser_sub, person_sub)

    {
        this->odom_subscriber();
        // this->leg_subscriber();
        // this->laserscan_topic_parser();
        this->init_subscriber();
        this->runloop();
    }
    ~leg_locator()
    {
        destructor();
    }
};
	// OdoManager odomGrp;

	// Odom tmp_target;
	// Odom abs_target;