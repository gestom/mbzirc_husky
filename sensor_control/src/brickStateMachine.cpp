#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mbzirc_husky/brickExploreAction.h>
#include <mbzirc_husky/brickPickupAction.h>
#include <mbzirc_husky/brickStackAction.h>
#include <std_srvs/Trigger.h>

typedef enum{
    FINDINGBRICKS,
    PICKINGUP,
    FINDINGSTACKSITE,
    STACKING
}EState;

EState state = FINDINGBRICKS;
//EState state = PICKINGUP;

ros::ServiceClient armHomeClient;

int pickupFailures = 0;
int stackingFailures = 0;

int main (int argc, char **argv) {

    //wait for uav's to take off
    //usleep(15000000);

    ros::init(argc, argv, "brickStateMachine");
    ros::NodeHandle n;

    armHomeClient = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/home_arm"); 
    std_srvs::Trigger srv;
    if(!armHomeClient.call(srv))
        ROS_ERROR("Error resetting arm position");

    actionlib::SimpleActionClient<mbzirc_husky::brickExploreAction> exploreAC("brickExploreServer", true);
    actionlib::SimpleActionClient<mbzirc_husky::brickPickupAction> pickupAC("brickPickupServer", true);
    actionlib::SimpleActionClient<mbzirc_husky::brickStackAction> stackAC("brickStackServer", true);

    ROS_INFO("Waiting for action servers to start.");
    ROS_INFO("Waiting for explore...");
    exploreAC.waitForServer();
    ROS_INFO("Waiting for pickup...");
    pickupAC.waitForServer();
    ROS_INFO("Waiting for stack...");
    //stackAC.waitForServer();
    ROS_INFO("Action servers started, sending goal."); 

    while(ros::ok())
    {
        if(state == FINDINGBRICKS)
        {
            //permanently explore until a goal is found
            mbzirc_husky::brickExploreGoal exploreGoal;
            exploreGoal.goal = 1;//1 = goto brick stack, 2 = goto build site
            actionlib::SimpleClientGoalState exploreState = exploreAC.sendGoalAndWait(exploreGoal, ros::Duration(0,0), ros::Duration(0,0));

            if(exploreState != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Explore server didn't complete. Trying again.");
                armHomeClient.call(srv);
                //TODO, clearly something fucked up, add some recovery behaviour here
            }
            else
            {
  	            ROS_INFO("Bricks found and approached, switching to pickup state");
                state = PICKINGUP;
            }
        }
        else if(state == PICKINGUP)
        {
            //bricks found and approached, switch to brick pickup, which only has t seconds to run, then t seconds to recover before going back to exploration
            ros::Duration totalMaxDuration = ros::Duration(500, 0);
            ros::Duration recoveryTime = ros::Duration(10, 0);

            mbzirc_husky::brickPickupGoal pickupGoal;
            actionlib::SimpleClientGoalState pickupState = pickupAC.sendGoalAndWait(pickupGoal, totalMaxDuration, recoveryTime);

            if(pickupState != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                pickupFailures++;
                int maxAttempts = 3;
                if(pickupFailures <= maxAttempts)
                {
                    ROS_INFO("Pickup attempt %i/%i failed, trying again", pickupFailures, maxAttempts);
                    state = FINDINGBRICKS;
                }
                else
                {
                    ROS_INFO("Pickup attempt %i/%i failed, going to build", pickupFailures, maxAttempts);
                    state = FINDINGSTACKSITE;
                    pickupFailures = 0;
                }
                ROS_INFO("Resetting arm");
                armHomeClient.call(srv);
            }
            else
            {
                state = FINDINGSTACKSITE;
                pickupFailures = 0;
                ROS_INFO("Bricks pickup up successfully, moving to stack area and rearranging");
            }
        }
        else if(state == FINDINGSTACKSITE)
        {
            //send message to move to brick stack area, and tell us when finished
            mbzirc_husky::brickExploreGoal exploreGoal;
            exploreGoal.goal = 2;
            actionlib::SimpleClientGoalState exploreState = exploreAC.sendGoalAndWait(exploreGoal, ros::Duration(0,0), ros::Duration(0,0));

            if(exploreState != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Failed to find stack area.");
                armHomeClient.call(srv);
                //TODO add recovery behaviour
            }
            else
            {
                ROS_INFO("Finished finding stack site, stacking bricks");
                state = STACKING;
            }
        }
        else if(state == STACKING)
        {
            ros::Duration totalMaxDuration = ros::Duration(600, 0);
            ros::Duration recoveryTime = ros::Duration(10, 0);

            mbzirc_husky::brickStackGoal stackGoal;
            actionlib::SimpleClientGoalState stackState = stackAC.sendGoalAndWait(stackGoal, totalMaxDuration, recoveryTime);
            
            if(stackState != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                stackingFailures++;
                int maxAttempts = 3;
                if(stackingFailures <= maxAttempts)
                {
                    ROS_INFO("Stacking attempt %i/%i failed, trying again", stackingFailures, maxAttempts);
                }
                else
                {
                    ROS_INFO("Stacking attempt %i/%i failed, going to loop", stackingFailures, maxAttempts);
                    state = FINDINGSTACKSITE;
                    stackingFailures = 0;
                }
                ROS_INFO("Resetting arm");
                armHomeClient.call(srv);
            }
            else
            {
                ROS_INFO("Bricks built up successfully, moving to brick area and picking up");
                state = FINDINGBRICKS;
                stackingFailures = 0;
            }
        }
        usleep(2000000);
    }
    return 0;
}
