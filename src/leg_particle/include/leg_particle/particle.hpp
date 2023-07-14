#include "leg_particle/system.hpp"

class Target
{
public:
    std::vector<cv::Point2f> target;
    float distance;
    float c_distance;
    int label;

    Target()
    {
    }
    ~Target()
    {
    }
};
class Particles
{
public:
    int id;
    double x;
    double y;
    double theta;
    double weight;

    Particles()
    {
    }
    ~Particles()
    {
    }
};

class Grouping
{
public:
    std::vector<Target> T_group;

    Grouping()
    {
    }
    ~Grouping()
    {
    }
};

class Particle
{
public:
    float mm2pixel = 100.0f / 1000.0f;

    int grid_row = 1000;
    int grid_col = 1000;

    ros::NodeHandle nh_param;

    float max_weight;

    std::vector<Particles> parts;
    std::vector<float> weights;
    Grouping clusterGroup;
    Particles p;

    // bool caught)target

    cv::Point2f actual_target_coordinate;

    int particle_num = 1000;

    cv::Point2f Target(std::vector<cv::Point2f> _laser_pt);
    cv::Point2f pt2Coor(float x_co, float y_co);

    float gaussian_weight(double p_coordinate, float target_coor);

    void initiate();
    void motion(std::vector<Particles> &smp_particle);
    void addWeight(std::vector<Particles> &smp_particle);
    void resampling(std::vector<Particles> &smp_particle);
    cv::Point2f mean_point(std::vector<Particles> &smp_particle);

    bool caught_target;
    Particle() : max_weight(0.0), caught_target(false)
    {
        // param_callback();
        initiate();
    }
    ~Particle()
    {
    }
};