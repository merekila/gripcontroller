  /*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ali Sacakli */

//#include <grippercontrol/grippercontrol.h>

#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <iimoveit/robot_interface.h>
#include <robotiq_s_model_control/s_model_msg_client.h>
#include <robotiq_s_model_control/s_model_api.h>


//double omega_angle;
//grippercontrol greifer;
double omega_angle;
double riq_angle;
ros::NodeHandle n;
boost::shared_ptr<robotiq_s_model_control::SModelMsgClient> sModelMsgClient(new robotiq_s_model_control::SModelMsgClient(*n));
robotiq_s_model_control::SModelAPI riq_gripper(sModelMsgClient);

// void grippercontrol::setRobotiqAngle(double angle){
//     angleDeg = -(90/28)*angle + 90;
//     riq_gripper.setRawPosition(angleDeg);
//     riq_gripper.write();
// }

// void grippercontrol::initRobotiq() {
//     ROS_INFO("Initializing riq_gripper...");
//     /*riq_gripper.setInitialization(INIT_ACTIVATION);
//     riq_gripper.setGraspingMode(GRASP_PINCH);
//     riq_gripper.setActionMode(ACTION_GO); */
//     riq_gripper.setRawVelocity(255);
//     riq_gripper.setRawForce(1);
//     riq_gripper.setRawPosition(0);
//     riq_gripper.write();
// }

void gripperCallback(const std_msgs::Float64& msg){

    omega_angle = msg.data;
    //greifer.setRobotiqAngle(omega_angle);
    riq_angle = -(9/28)*omega_angle +90;
    //riq_gripper.setRobotiqAngle(riq_angle);
    riq_gripper.setRawPosition(riq_angle);
    riq_gripper.write();
}


int main(int argc, char ** argv){
    ros::init(argc, argv, "grippercontrol");
    //ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("GripperAngle", 1, gripperCallback);
    
    ros::spin();
}
