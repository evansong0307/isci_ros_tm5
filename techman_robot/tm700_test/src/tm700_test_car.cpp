/*********************************************************************
 * tm700_test_node.cpp
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 *
 * Author: Howard Chen, ISCI, NCTU
 */

#include <ros/ros.h>
#include <ros/console.h>
#include "tm_kinematics/tm_kin.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <mysql/mysql.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>


#include "tm_msgs/SetIO.h"
//#include "tm_msgs/SetIORequest.h"
//#include "tm_msgs/SetIOResponse.h"

using namespace tm_kinematics;
using namespace std;
void finish_with_error(MYSQL *con)
{
    fprintf(stderr, "%s\n", mysql_error(con));
    mysql_close(con);
    exit(1);
}
bool try_move_to_named_target(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan,
                              const std::string& target_name,
                              unsigned int max_try_times = 1
                             )
{
    if (!ros::ok()) return false;
    bool success = false;

    for (unsigned int i = 0; i < max_try_times; i++)
    {

        group.setNamedTarget(target_name);

        if (group.move())
        {
            success = true;
            break;
        }
        else
        {
            if (!ros::ok()) break;
            sleep(1);
        }
    }
    return success;
}

bool try_move_to_joint_target(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan,
                              const std::vector<double>& joint_target,
                              unsigned int max_try_times = 1
                             )
{
    if (!ros::ok()) return false;
    bool success = false;

    for (unsigned int i = 0; i < max_try_times; i++)
    {

        group.setJointValueTarget(joint_target);

        if (group.move())
        {
            success = true;
            break;
        }
        else
        {
            if (!ros::ok()) break;
            sleep(1);
        }
    }
}
bool Cartesian(moveit::planning_interface::MoveGroup& group,moveit::planning_interface::MoveGroup::Plan& plan,const std::vector<double>& pos_target,unsigned int max_try_times = 1)
{
        if (!ros::ok()) return false;
        bool success = false;
    for (unsigned int i = 0; i < max_try_times; i++)
        {
         group.setPositionTarget(pos_target[0],pos_target[1],pos_target[2]);
         if(group.asyncMove())
         {
            success=true;
            break;
         }
         else
         {
             if(!ros::ok()) break;
             sleep(1);
         }
        }
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "tm700_test");
    ros::NodeHandle node_handle;
    ros::ServiceClient set_io_client = node_handle.serviceClient<tm_msgs::SetIO>("tm_driver/set_io");
    tm_msgs::SetIO io_srv;
    io_srv.request.fun = 2;//tm_msgs::SetIORequest::FUN_SET_EE_DIGITAL_OUT;
    io_srv.request.ch = 0;
    io_srv.request.value = 0.0;
    double *x;
    double *y;
    double *z;
    x=new double [2000];
    y=new double [2000];
    z=new double [2000];
    char save[100];
    // start a background "spinner", so our node can process ROS messages
    //  - this lets us know when the move is completed
    ros::AsyncSpinner spinner(1);
    spinner.start();
    MYSQL *con = mysql_init(NULL);
    if (con == NULL)
      {
      fprintf(stderr, "%s\n", mysql_error(con));
      exit(1);
      }
      if (mysql_real_connect(con, "140.113.149.61", "test", "0120","nova", 0, NULL, 0) == NULL)
      {
      finish_with_error(con);
      }
      sprintf(save,"SELECT * FROM %s","DATA621");
      string a=save;
      if (mysql_query(con, a.c_str()))
    {
    finish_with_error(con);
    }
    MYSQL_RES *result = mysql_store_result(con);
      if (result == NULL)
      {
          finish_with_error(con);
      }
      int num_fields = mysql_num_fields(result);
      MYSQL_ROW row;
      int record=0;
      while ((row = mysql_fetch_row(result)))
  {
      /*for(int i = 0; i < num_fields; i++)
      {
          printf("%s ", row[i] ? row[i] : "NULL");
      }*/
      x[record]=(atof(row[0])/1000.0)+0.0;//0
      y[record]=(atof(row[1])/1000.0)+0.15;
      z[record]=(atof(row[2])/1000.0)+0.608;//0.458
      if(z[record]<0.314)
      	z[record]=0.314;
      printf("x=%f  y=%f  z=%f\n",x[record],y[record],z[record]);
      record++;
  }
  mysql_free_result(result);
    mysql_close(con);
    sleep(1);

    // Setup
    // ^^^^^
    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name
    // of the group you would like to control and plan for.
    moveit::planning_interface::MoveGroup group("manipulator");
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // (Optional) Create a publisher for visualizing plans in Rviz.
    //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //moveit_msgs::DisplayTrajectory display_trajectory;

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    moveit::planning_interface::MoveGroup::Plan my_plan;

    set_io_client.call(io_srv);

    group.setPlanningTime(10.0);//30.0

    try_move_to_named_target(group, my_plan, "home", 100);
    double *T1;
    T1= new double[16];
    std::vector<double> joint_target_1;
    std::vector<double> joint_target_2;
    std::vector<double> Car[7];
    T1[0]=0;
    T1[1]=1;
    T1[2]=0;
    T1[3]=0.2401;
    T1[4]=1;
    T1[5]=0;
    T1[6]=0;
    T1[7]=-0.2137;
    T1[8]=0;
    T1[9]=0;
    T1[10]=-1;
    T1[11]=0.36;
    T1[12]=0;
    T1[13]=0;
    T1[14]=0;
    T1[15]=1;
//    double joint_target[6]={0.5236,0.2618,1.8326,-0.5236,1.5708,0.5236};
    joint_target_1.assign(6, 0.0f);
   // joint_target_1[0] = 0.0174533 * ( 30.0);
  //  joint_target_1[1] = 0.0174533 * ( 15.0);
//    joint_target_1[2] = 0.0174533 * (105.0);
    //joint_target_1[3] = 0.0174533 * (-30.0);
  //  joint_target_1[4] = 0.0174533 * ( 90.0);
//    joint_target_1[5] = 0.0174533 * ( 30.0);
  //  forward(joint_target,T);
    /*for(int i=0;i<16;i++)
    ROS_INFO("T%d=%f",i,T1[i]);*/
    double p[10];
   inverse(T1,p);
  /*  for(int i=0;i<7;i++)
        ROS_INFO("p[%d]=%f",i,p[i]);*/
    for(int i=0;i<6;i++)
        joint_target_1[i]=p[i];
    joint_target_2.assign(6, 0.0f);
    joint_target_2[0] = 0.0174533 * (-30.0);
    joint_target_2[1] = 0.0174533 * (-15.0);
    joint_target_2[2] = 0.0174533 * ( 90.0);
    joint_target_2[3] = 0.0174533 * (-75.0);
    joint_target_2[4] = 0.0174533 * (120.0);


   // int step = 0;
//Move to First Point
    ROS_INFO("move...to First Point");
    try_move_to_joint_target(group, my_plan, joint_target_1, 100);
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
    int k=0;

    double lastrecordx,lastrecordy,lastrecordz;
    lastrecordx=x[k];
    lastrecordy=y[k];
    lastrecordz=z[k];
    while (ros::ok())
    {
      if(x[k]==-1)
      {
       ROS_INFO("here");
        ros::shutdown();
      }
      if((abs(x[k]-lastrecordx)>0.01)||(abs(y[k]-lastrecordy))>0.01||(abs(z[k]-lastrecordz)>0.01))
      {
        lastrecordx=x[k];
        lastrecordy=y[k];
        lastrecordz=z[k];
        T1[3]=x[k];
        T1[7]=y[k];
        T1[11]=z[k];
        inverse(T1,p);
    for(int i=0;i<6;i++)
        joint_target_1[i]=p[i];
  //  ROS_INFO("move...to next Point");
    try_move_to_joint_target(group, my_plan, joint_target_1, 100);
      }
       /* switch (step)
        {
        case 0: //move to ready1
            ROS_INFO("move...to ready1");
            //Cartesian(group,my_plan,Car[0],100);
            try_move_to_named_target(group, my_plan, "ready1", 100);
            ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
            break;
        case 1: //move to ready2
            ROS_INFO("move...to ready2");
           // Cartesian(group,my_plan,Car[1],100);
            try_move_to_named_target(group, my_plan, "ready2", 100);
            break;
        case 2: //move to ready3
            ROS_INFO("move...to ready3");
            //Cartesian(group,my_plan,Car[2],100);
            try_move_to_named_target(group, my_plan, "ready3", 100);
            break;
        case 3: //move to ready4
            ROS_INFO("move...to ready4");
//            Cartesian(group,my_plan,Car[3],100);
            try_move_to_named_target(group, my_plan, "ready4", 100);
            break;

        case 4: //move 1
            ROS_INFO("move...to joint_target_1");
            try_move_to_joint_target(group, my_plan, joint_target_1, 100);
            //Cartesian(group,my_plan,Car[4],100);
            break;

        case 5: //move 2...
            ROS_INFO("move...to joint_target_2");
            try_move_to_joint_target(group, my_plan, joint_target_2, 100);
           // Cartesian(group,my_plan,Car[5],100);
            break;
        case 6://to assign position
            ROS_INFO("move to position target");
           // Cartesian(group,my_plan,Car[6],100);
            break;
        }
        step = (step + 1) % 7;*/
        k++;
        if(k==record)
        break;
    }
    delete T1;
    delete x;
    delete y;
    delete z;
    return 0;
}
