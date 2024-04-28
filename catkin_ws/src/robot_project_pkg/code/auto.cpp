#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_and_goal_publisher");
    ros::NodeHandle nh;

    // Publisher per la posizione iniziale
    ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    // Publisher per il goal di navigazione
    ros::Publisher goal_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);

    // Attendi un po' per garantire che i publisher si registrino correttamente
    ros::Duration(2.0).sleep();

    // Crea e pubblica il messaggio di initial pose
    geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
    initial_pose_msg.header.stamp = ros::Time::now();
    initial_pose_msg.header.frame_id = "map";  // Frame di riferimento della posizione iniziale
    initial_pose_msg.pose.pose.position.x = (0+106.913*0.02)/2;  // Posizione X del centro del mondo reale
    initial_pose_msg.pose.pose.position.y = (13+49.3527 *0.02)/2;  // Posizione Y del centro del mondo reale
    initial_pose_msg.pose.pose.position.z = 0;  // Nessuna componente Z, solo posizione 2D
    initial_pose_msg.pose.pose.orientation.w = 1.0;  // Orientamento (in questo caso, nessuna rotazione)

    // Pubblica il messaggio di initial pose
    initial_pose_pub.publish(initial_pose_msg);
    ROS_INFO("New initial pose published.");

    // Crea e pubblica il messaggio di goal
    move_base_msgs::MoveBaseActionGoal goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "map";  // Frame di riferimento del goal
    goal_msg.goal.target_pose.header.stamp = ros::Time::now();
    goal_msg.goal.target_pose.header.frame_id = "map";  // Frame di riferimento del goal
    goal_msg.goal.target_pose.pose.position.x = 10.0;  // Coordinata X del goal
    goal_msg.goal.target_pose.pose.position.y = 20.0;   // Coordinata Y del goal
    goal_msg.goal.target_pose.pose.position.z = 0.0;    // Nessuna componente Z, solo posizione 2D
    goal_msg.goal.target_pose.pose.orientation.w = 1.0;  // Orientamento (in questo caso, nessuna rotazione)

    // Pubblica il messaggio del goal
    goal_pub.publish(goal_msg);
    ROS_INFO("New goal published.");

    ros::spin();
    return 0;
}