// COMMENTAIRE en 2spi
// la simulation sous gazebo marche pas
// je simule donc avec ritz avec la commande plus bas
// jai creer un service pour bouger les joints
//
// FONCTIONNEMENT
//
// 1. lancer ritz dans un premier terminal :
// roslaunch niryo_one_bringup desktop_rviz_simulation.launch
//
// 2. lancer le run.launch dans un deuxieme terminal
// roslaunch projet_niryo_clement run.launch
//
// 3. lancer mon_service afin de bouger les joints du robot
// rosservice call /mon_service "{joint1: 3.0, joint2: 0.0, joint3: 0.0,
// joint4: 2.0, joint5: 5.0, joint6: 1.0}"

#include "projet_niryo_clement/TrajectoireForwardKinematic.h"
#include <csignal>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define FREQUENCY 1

using namespace ros;
using namespace std_msgs;
using namespace trajectory_msgs;

// Prototypes des fonctions
void sigintHandler(int sig);

// Variables globales
Publisher log_pub;
Publisher trajectoire_pub;
Subscriber trajectoire_sub;

JointTrajectory join_trajectory;
JointTrajectoryPoint joint_trajectory_point;

String msg;
float joint1, joint2, joint3, joint4, joint5, joint6;
double frequence = FREQUENCY;

// Fonctions
void forward_kinematic(float joint1, float joint2, float joint3, float joint4,
                       float joint5, float joint6) {
  join_trajectory = JointTrajectory();

  join_trajectory.header.stamp = ros::Time::now();
  join_trajectory.joint_names.push_back("joint_1");
  join_trajectory.joint_names.push_back("joint_2");
  join_trajectory.joint_names.push_back("joint_3");
  join_trajectory.joint_names.push_back("joint_4");
  join_trajectory.joint_names.push_back("joint_5");
  join_trajectory.joint_names.push_back("joint_6");

  joint_trajectory_point = JointTrajectoryPoint();
  joint_trajectory_point.time_from_start.sec = 1;

  joint_trajectory_point.positions.push_back(joint1);
  joint_trajectory_point.positions.push_back(joint2);
  joint_trajectory_point.positions.push_back(joint3);
  joint_trajectory_point.positions.push_back(joint4);
  joint_trajectory_point.positions.push_back(joint5);
  joint_trajectory_point.positions.push_back(joint6);
  join_trajectory.points.push_back(joint_trajectory_point);
}

// Callbacks

bool ServiceCallback(
    projet_niryo_clement::TrajectoireForwardKinematic::Request &req,
    projet_niryo_clement::TrajectoireForwardKinematic::Response &res) {
  joint1 = req.joint1;
  joint2 = req.joint2;
  joint3 = req.joint3;
  joint4 = req.joint4;
  joint5 = req.joint5;
  joint6 = req.joint6;

  ROS_INFO(" La destination a ete change ! ");
  return true;
}
void trajectoireCallback(JointTrajectory join_trajectory) {
  for (int i = 0; i < 6; i++) {
    ROS_INFO("Joint %d , position : %f", i + 1,
             joint_trajectory_point.positions[i]);
  }
}

void build_log_msg(char *text) { ROS_INFO("Publisher : %s", msg.data.c_str()); }

int main(int argc, char **argv) {

  signal(SIGINT, sigintHandler);
  ros::init(argc, argv, "controleur", ros::init_options::NoSigintHandler);

  NodeHandle nh;

  // publishers
  trajectoire_pub =
      nh.advertise<JointTrajectory>("/niryo_one_follow_joint_trajectory_\
controller/command",
                                    1000);
  trajectoire_sub = nh.subscribe("/niryo_one_follow_joint_trajectory_\
controller/command",
                                 1000, trajectoireCallback);

  // services
  ros::ServiceServer service =
      nh.advertiseService("mon_service", ServiceCallback);

  Rate loop_rate(frequence);

  joint1 = 0.2;
  joint2 = 0.5;
  joint3 = 0.0;
  joint4 = 0.1;
  joint5 = 0.4;
  joint6 = 0.0;

  // quand j'aurais import le bail des vectors : faire un vectore avec tous les
  // joints (0.2, 0.5, 0.0, 0.1, 0.4, 0.0)

  while (ok()) {
    forward_kinematic(joint1, joint2, joint3, joint4, joint5, joint6);
    trajectoire_pub.publish(join_trajectory);
    spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

// FIN DU PROGRAMME  FIN DU PROGRAMME  FIN DU PROGRAMME  FIN DU PROGRAMME

/**
 *
 * SIGINT handler.
 * Exit code gracefully.
 *
 * @param sig - catched signal.
 *
 */

void sigintHandler(int sig) {
  // Log quit
  ROS_INFO("Exiting program gracefully ...");

  // MESSAGE A PUBLIER le MESSAGE

  // Kill all open subscriptions, publications, service calls, and service
  // servers
  shutdown();
}