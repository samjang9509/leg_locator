#include "leg_locator/motion.hpp"

void motion_control::init_Publisher()
{
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

float motion_control::ed_meter(cv::Point2f d_target)
{
    float output;

    output = std::sqrt(pow(d_target.x,2) + pow(d_target.y,2));

    return output;
}

void motion_control::move2target(cv::Point2f p_target)
{
    geometry_msgs::Twist vel_pub;

    cv::Point2f target;

    target.x = mm2m(p_target.x);
    target.y = mm2m(p_target.y);

    double angle = atan2(target.y, target.x);
    float distance = ed_meter(target);
    try
    {
        if (distance <= safe_distance)
        {
            vel_pub.linear.x = 0.0;
            vel_pub.linear.y = 0.0;
            if(abs(angle) > 0.1)
            {
                vel_pub.angular.z = angle / 2.0;
            }
            else
            {
                vel_pub.angular.z = 0.0;
            }
        }
        else
        {
            if (angle < 0 && target.x < 0)
            {
                vel_pub.linear.x = std::min(-(double)target.x / 1.5, -min_lin_vel);
                vel_pub.linear.y = std::min(-(double)target.y / 1.5, -min_lin_vel);
                vel_pub.angular.z = std::min(angle/1.3, -min_ang_vel);
            }
            else if(angle < 0 && target.x > 0)
            {
                vel_pub.linear.x = std::max((double)target.x / 1.5, min_lin_vel);
                vel_pub.linear.y = std::min(-(double)target.y / 1.5, -min_lin_vel);
                vel_pub.angular.z = std::min(angle/1.3, -min_ang_vel);
            }
            else if(angle > 0 && target.x < 0)
            {
                vel_pub.linear.x = std::min(-(double)target.x / 1.5, -min_lin_vel);
                vel_pub.linear.y = std::max((double)target.y / 1.5, min_lin_vel);
                vel_pub.angular.z = std::max(angle/1.3, min_ang_vel);
            }
            else if(angle > 0 && target.x > 0)
            {
                vel_pub.linear.x = std::max((double)target.x / 1.5, min_lin_vel);
                vel_pub.linear.y = std::max((double)target.y / 1.5, min_lin_vel);
                vel_pub.angular.z = std::max(angle/1.3, min_ang_vel);
            }
        }
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }catch (const std::runtime_error &e)
    {
      std::cerr << e.what() << '\n';
    }catch (const cv::Exception &e)
    {
      std::cerr << e.what() << '\n';
    }catch (const std::bad_alloc &e)
    {
      std::cerr << e.what() << '\n';
    }
    catch(...)
    {
        std::cout << "Other Exception : Motion" << std::endl;
    }

    cmd_pub.publish(vel_pub);
}