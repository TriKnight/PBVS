#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//the circle constant tau=2*pi
const double tau=2*M_PI;

int main(int argc, char** argv)
{
    ros::init(argc,argv, "move_group_tutorial");
    ros::NodeHandle nh; 
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Start the code
    //Create the planning group 
    static const std::string PLANNING_GROUP ="manipulator";
    static const std::string base_link_inertia;
    //Setup the planning interface to control the planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    // We will use the :planning_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface ;    
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //Set the base_frame
    move_group.setPoseReferenceFrame(base_link_inertia);
    //Moving the end effector to the Pose goal
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w =1;
    target_pose1.orientation.x =0;
    target_pose1.orientation.y =0;
    target_pose1.orientation.z =0;
    target_pose1.position.x =0.2;
    target_pose1.position.y =0.5;
    target_pose1.position.z =0.5;
    // Movegroup to move to the point
    move_group.setPoseTarget(target_pose1); 
    move_group.move();

}