#ifndef __GRIPPERCONTROL_H__
#define __GRIPPERCONTROL_H__


#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <robotiq_s_model_control/s_model_msg_client.h>
#include <robotiq_s_model_control/s_model_api.h>


class grippercontrol{

    public: 
        
        robotiq_s_model_control::SModelAPI riq_gripper;

    protected:
        double riq_angle;
        double angleDeg;
        void setRobotiqAngle(double angle);
        void initRobotiq();
        void gripperCallback(const std_msgs::Float64::ConstPtr& msg);

};
#endif