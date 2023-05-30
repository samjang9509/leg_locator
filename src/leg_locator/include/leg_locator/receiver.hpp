#include "leg_locator/systems.hpp"

class receiver
{
public:
	struct extrinsic
	{
		tf::Matrix3x3 R;
		tf::Vector3 T;
	};
    bool get_tf_flag;
    tf::Matrix3x3 R;
    tf::Vector3 T;
    tf::TransformListener listener;

    std::string parent_frame, child_frame;

    receiver() :
        get_tf_flag(false)
    {
		parent_frame = "base_link";
        child_frame = "laser1";
    }
    ~receiver()
    {}

public:
    bool get_tf(std::string child_frame_)
    {
		if(parent_frame.empty() || child_frame_.empty())
		{
			ROS_ERROR("There is no frame name");
			return 0;
		}

		try
		{
			tf::StampedTransform tf_msg;
			ros::Time now = ros::Time::now();
			listener.lookupTransform(parent_frame, child_frame_, ros::Time(0), tf_msg);

			R = tf::Matrix3x3(tf_msg.getRotation());
			T = tf::Vector3(
				tf_msg.getOrigin().x(),
				tf_msg.getOrigin().y(),
				tf_msg.getOrigin().z());

            tf::Vector3 v1 = R[0]; 
            tf::Vector3 v2 = R[1]; 
            tf::Vector3 v3 = R[2]; 

            // std::cout<<"R "<<v1[0]<<','<<v1[1]<<','<<v1[2]<<std::endl
            //                         <<v2[0]<<','<<v2[1]<<','<<v2[2]<<std::endl
            //                         <<v3[0]<<','<<v3[1]<<','<<v3[2]<<std::endl;
		}
		catch(...)//...?
		{
			ROS_ERROR("Fail to get tf");
			return 0;
		}
		
		// ROS_INFO("success to get tf");
		return 1;


    }	
};