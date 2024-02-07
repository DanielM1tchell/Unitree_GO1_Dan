#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dan_walk");

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;

    ros::Rate loop_rate(500);

    long motiontime = 0;

    unitree_legged_msgs::HighCmd high_cmd_ros;

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);

    while (ros::ok())
    {

        motiontime += 2;

        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        high_cmd_ros.mode = 0;
        high_cmd_ros.gaitType = 0;
        high_cmd_ros.speedLevel = 0;
        high_cmd_ros.footRaiseHeight = 0;
        high_cmd_ros.bodyHeight = 0;
        high_cmd_ros.euler[0] = 0;
        high_cmd_ros.euler[1] = 0;
        high_cmd_ros.euler[2] = 0;
        high_cmd_ros.velocity[0] = 0.0f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.yawSpeed = 0.0f;
        high_cmd_ros.reserve = 0;

     
        
        if (motiontime > 0 && motiontime < 1000)
        {
            high_cmd_ros.mode = 6;				//Stand Up 
        }
        
        if (motiontime > 1000 && motiontime < 2000)
        {
            high_cmd_ros.mode = 1;				//Force Stand
        }
        
        if (motiontime > 2000 && motiontime < 12000)		//			WALK FORWARD
        {
            high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = 0.2f; // -1  ~ +1
            high_cmd_ros.yawSpeed = 0.0;			//rotate Theta left (2 is approx 180 degrees)
            // printf("walk\n");
        }
        
        if (motiontime > 12000 && motiontime < 13000)
        {
            high_cmd_ros.mode = 1;				//Force Stand Idle
        }
        
        
        if (motiontime > 13000)
        {
            high_cmd_ros.mode = 1;
        }

        pub.publish(high_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
