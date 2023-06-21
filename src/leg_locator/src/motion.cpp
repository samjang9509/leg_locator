#include "leg_locator/motion.hpp"

void motion_control::init_Publisher()
{
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/leg_tracker/cmd_vel", 10);
}

float motion_control::ed_meter(cv::Point2f d_target)
{
    float output;

    output = std::sqrt(pow(d_target.x,2) + pow(d_target.y,2));

    return output;
}

double motion_control::velocity(float distance)
{
    double output;

    double tmp_x;

    tmp_x = (pow((double)distance - 0.3, 2)) / 5;

    output = std::exp(tmp_x) / (5 * std::sqrt(2 * M_PI));

    return output;
}


void motion_control::move2target(cv::Point2f p_target) 
{
    geometry_msgs::Twist vel_pub;

    cv::Point2f target;
    cv::Point2f odom_target;

    // std::cout << "Motion final_target coordinate : (" << final_target.x << ", " << final_target.y << ", " << final_target.th << ")" << std::endl;
    // std::cout << "Motion Robot coordinate : (" << odomPt.robot.x << ", " << odomPt.robot.y << ", " << odomPt.robot.th << ")" << std::endl;

    odom_target.x = final_target.x - odomPt.robot.x;
    odom_target.y = final_target.y - odomPt.robot.y;
    

    // std::cout << "Motion tmp_target coordinate : (" << tmp_target.x << ", " << tmp_target.y << ", " << tmp_target.th << ")" << std::endl;

    target.x = p_target.x;
    target.y = p_target.y;

    // std::cout << "target : " << target << std::endl;
    double angle = atan2(target.y, target.x);
    float distance = ed_meter(odom_target);
    try
    {
        if (distance < safe_distance)
        {
            ROS_INFO("Too Close");
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
        else if(!target_track)
        {
            ROS_ERROR("Motion has not been activated");
            vel_pub.linear.x = 0.0;
            vel_pub.linear.y = 0.0;
            vel_pub.angular.z = 0.0;
        }
        else
        {
            // std::cout << angle << std::endl;

            if (angle < 0 && target.x < 0)
            {
                vel_pub.linear.x = std::max(-velocity(target.x), -min_lin_vel);
                vel_pub.linear.y = std::max(-velocity(target.y), -min_lin_vel);
                vel_pub.angular.z = std::min(-angle/1.3, min_ang_vel);
            }
            else if(angle < 0 && target.x > 0)
            {
                vel_pub.linear.x = std::min(velocity(target.x), min_lin_vel);
                vel_pub.linear.y = std::max(-velocity(target.y), -min_lin_vel);
                vel_pub.angular.z = std::min(-angle/1.3, min_ang_vel);
            }
            else if(angle > 0 && target.x < 0)
            {
                vel_pub.linear.x = std::max(-velocity(target.x), -min_lin_vel);
                vel_pub.linear.y = std::min(velocity(target.y), min_lin_vel);
                vel_pub.angular.z = std::max(-angle/1.3, -min_ang_vel);
            }
            else if(angle > 0 && target.x > 0)
            {
                vel_pub.linear.x = std::min(velocity(target.x), min_lin_vel);
                vel_pub.linear.y = std::min(velocity(target.y), min_lin_vel);
                vel_pub.angular.z = std::max(-angle/1.3, -min_ang_vel);
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