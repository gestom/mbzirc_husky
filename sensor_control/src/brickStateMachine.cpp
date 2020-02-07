#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mbzirc_husky/brickExploreAction.h>
#include <mbzirc_husky/brickPickupAction.h>
#include <mbzirc_husky/brickRearrangeAction.h>
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
bool currentlyRearranging = false;

ros::ServiceClient armHomeClient;

int main (int argc, char **argv) {
    ros::init(argc, argv, "brickStateMachine");

    ros::NodeHandle n;
    armHomeClient = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/home_arm"); 
    std_srvs::Trigger srv;
    if(!armHomeClient.call(srv))
        ROS_ERROR("Error resetting arm position");

    actionlib::SimpleActionClient<mbzirc_husky::brickExploreAction> exploreAC("brickExploreServer", true);
    actionlib::SimpleActionClient<mbzirc_husky::brickPickupAction> pickupAC("brickPickupServer", true);
    actionlib::SimpleActionClient<mbzirc_husky::brickRearrangeAction> rearrangeAC("brickRearrangeServer", true);
    actionlib::SimpleActionClient<mbzirc_husky::brickStackAction> stackAC("brickStackServer", true);

    ROS_INFO("Waiting for action servers to start.");
    ROS_INFO("Waiting for explore...");
    exploreAC.waitForServer();
    ROS_INFO("Waiting for pickup...");
    pickupAC.waitForServer();
    ROS_INFO("Waiting for rearrange...");
    rearrangeAC.waitForServer();
    ROS_INFO("Waiting for stack...");
    stackAC.waitForServer();
    ROS_INFO("Action servers started, sending goal."); 

    while(ros::ok())
    {
        if(state == FINDINGBRICKS)
        {
            //permanently explore until a goal is found
            mbzirc_husky::brickExploreGoal exploreGoal;
            exploreGoal.goal = 1;//find brick stack
            actionlib::SimpleClientGoalState exploreState = exploreAC.sendGoalAndWait(exploreGoal, ros::Duration(0,0), ros::Duration(0,0));

            if(exploreState != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Explore server didn't complete. Trying again.");
                armHomeClient.call(srv);
                //TODO, clearly something fucked up, add some recovery behaviour here
                continue;
            }
            state = PICKINGUP;
            ROS_INFO("Bricks found and approached, switching to pickup");
        }
        else if(state == PICKINGUP)
        {
            //bricks found and approached, switch to brick pickup, which only has t seconds to run, then t seconds to recover before going back to exploration
            ros::Duration totalMaxDuration = ros::Duration(300, 0);
            ros::Duration recoveryTime = ros::Duration(15, 0);

            mbzirc_husky::brickPickupGoal pickupGoal;
            pickupGoal.bearing = 0;
            actionlib::SimpleClientGoalState pickupState = pickupAC.sendGoalAndWait(pickupGoal, totalMaxDuration, recoveryTime);

            if(pickupState != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Pickup failed, going back to explore");
                armHomeClient.call(srv);
                //TODO maybe try same again before going back to explore
                //TODO, recovery behavious. Perhaps try again from alternative angle, and/or mark area as difficult but can return eventually
                state = FINDINGBRICKS;
                continue;
            }
            state = FINDINGSTACKSITE;
            ROS_INFO("Brick(s) pickup up successfully, moving to stack area and rearranging");
        }
        else if(state == FINDINGSTACKSITE)
        {
            if(!currentlyRearranging)
            {
                //send message to begin rearranging bricks on board
                mbzirc_husky::brickRearrangeGoal rearrangeGoal;
                rearrangeAC.sendGoal(rearrangeGoal);
                currentlyRearranging = true;
            }

            //send message to move to brick stack area, and tell us when finished
            mbzirc_husky::brickExploreGoal exploreGoal;
            exploreGoal.goal = 2;
            actionlib::SimpleClientGoalState exploreState = exploreAC.sendGoalAndWait(exploreGoal, ros::Duration(0,0), ros::Duration(0,0));

            if(exploreState != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Failed to find stack area.");
                armHomeClient.call(srv);
                //TODO add recovery behaviour
                continue;
            }

            ROS_INFO("Found brick stack area, checking state of rearranger");
            rearrangeAC.waitForResult(ros::Duration(0.0));
            currentlyRearranging = false;
            state = STACKING;
            ROS_INFO("Rearranger finished, stacking bricks");
        }
        else if(state == STACKING)
        {
            mbzirc_husky::brickStackGoal stackGoal;
            actionlib::SimpleClientGoalState stackState = stackAC.sendGoalAndWait(stackGoal, ros::Duration(0,0), ros::Duration(0,0));
            
            if(stackState != actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Stack server didn't complete. Trying again.");
                armHomeClient.call(srv);
                //TODO, clearly something fucked up, add some recovery behaviour here
                //TODO if fail multiple times, go back to explore anyway
                continue;
            }
            state = FINDINGBRICKS;
            ROS_INFO("Bricks stacked, finding more bricks");
        }
        usleep(100);
    }
    return 0;
}
