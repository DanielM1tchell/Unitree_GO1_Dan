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

     
        if (motiontime > 0 && motiontime < 2000)
        {
            high_cmd_ros.mode = 5;				//Stand down
        }
        if (motiontime > 2000 && motiontime < 3000)
        {
            high_cmd_ros.mode = 6;				//Stand Up 
        }
        if (motiontime > 4000 && motiontime < 5000)
        {
            high_cmd_ros.mode = 0;				//Stand Idle
        }
        if (motiontime > 5000 && motiontime < 6000)
        {
            high_cmd_ros.mode = 1;				//Force Stand
        }
        
        if (motiontime > 6000 && motiontime < 8000)		//			WALK FORWARD
        {
            high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = 0.4f; // -1  ~ +1
            high_cmd_ros.yawSpeed = 0;			//rotate Theta left (2 is approx 180 degrees)
            // printf("walk\n");
        }
        
        if (motiontime > 8000 && motiontime < 10000)
        {
            high_cmd_ros.mode = 1;				//Force Stand Idle
        }
        
        
        if (motiontime > 10000 && motiontime < 12000)		//			SETUP FOR 45 Degrees left
        {
            high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = 0.4f; // -1  ~ +1
            high_cmd_ros.yawSpeed = 0.5;			//Speed of rotation whilst walking (left) 0.5 = 45 degrees approx
            high_cmd_ros.footRaiseHeight = 0.1;
            // printf("walk\n");
        }	
        
        if (motiontime > 12000 && motiontime < 14000)
        {
            high_cmd_ros.mode = 1;				//Force Stand Idle
        }
        
        if (motiontime > 14000 && motiontime < 16000)	//			SETUP FOR 45 Degrees Right
        {
            high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = 0.4f; // -1  ~ +1
            high_cmd_ros.yawSpeed = -0.5;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
            high_cmd_ros.footRaiseHeight = 0.1;
            // printf("walk\n");
        }	
        
        if (motiontime > 18000)
        {
            high_cmd_ros.mode = 1;
        }

        pub.publish(high_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
