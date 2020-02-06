#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mbzirc_husky/sprayAction.h>
#include <mbzirc_husky/fireExploreAction.h>

int main (int argc, char **argv) {
    ros::init(argc, argv, "fireStateMachine");

    actionlib::SimpleActionClient<mbzirc_husky::fireExploreAction> exploreAC("fireExploreServer", true);
    actionlib::SimpleActionClient<mbzirc_husky::sprayAction> sprayAC("sprayServer", true);

    ROS_INFO("Waiting for action server to start.");
    sprayAC.waitForServer();
    exploreAC.waitForServer();
    ROS_INFO("Action server started, sending goal."); 

    while(ros::ok())
    {
        //permanently explore until a goal is found
        mbzirc_husky::fireExploreGoal exploreGoal;
        exploreGoal.explore = 1;
        actionlib::SimpleClientGoalState exploreState = exploreAC.sendGoalAndWait(exploreGoal, ros::Duration(0,0), ros::Duration(0,0));
        
        if(exploreState != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Explore server didn't complete. Trying again.");
            //TODO, clearly something fucked up, add some recovery behaviour here
            continue;
        }

        //Fire found, switch to spray AC, which only has t seconds to extinguish fire, then 5 seconds to recover before exploration
        ROS_INFO("Fire found, switching to spray server");
        ros::Duration totalMaxDuration = ros::Duration(180, 0);
        ros::Duration recoveryTime = ros::Duration(5, 0);
        int sprayTime = 15;
        
        mbzirc_husky::sprayGoal sprayGoal;
        sprayGoal.duration = sprayTime;
        actionlib::SimpleClientGoalState sprayState = sprayAC.sendGoalAndWait(sprayGoal, totalMaxDuration, recoveryTime);
        
        if(sprayState == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Fire extinguished");
        }
        else if(sprayState != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Spray failed, going back to explore");
            //TODO, recovery behavious. Perhaps try again from alternative angle, and/or mark area as difficult but can return eventually
        }
    }
    return 0;
}
