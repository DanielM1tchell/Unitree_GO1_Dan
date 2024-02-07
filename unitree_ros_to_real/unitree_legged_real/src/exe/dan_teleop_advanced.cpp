#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <termios.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <thread>
#include <chrono>

using namespace UNITREE_LEGGED_SDK;

char getch() 
{
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
 }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_input_node");

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;

    ros::Rate loop_rate(500);

    long motiontime = 0;
    long motiontimedelete= 0;
    long count = 0;
    

    unitree_legged_msgs::HighCmd high_cmd_ros;

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);
     	
     	
   while (ros::ok())
    {

        motiontime += 2;
        motiontimedelete += 2;

        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        high_cmd_ros.mode = 0;			//0=idle stand, 1=force stand, 2=walk, 3=walk to specified location, 4= path mode walking, 5=stand down at current position, 6=stand up at current position, 7= damping at current position, 8=recovery stand up at current position, 9= back flip, 10=90 degree rotation with jump, 11= upwards praying, 12/13=dancing
        high_cmd_ros.gaitType = 0;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle)
        high_cmd_ros.speedLevel = 0;			//ONLY IF MODE 3 (I haven't tested this)
        high_cmd_ros.footRaiseHeight = 0;		//Foot height in metres when moved up
        high_cmd_ros.bodyHeight = 0;			//Body height in metres
        high_cmd_ros.euler[0] = 0;			
        high_cmd_ros.euler[1] = 0;
        high_cmd_ros.euler[2] = 0;
        high_cmd_ros.velocity[0] = 0.0f;		//+ve= forward -ve= backwards
        high_cmd_ros.velocity[1] = 0.0f;		//+ve left strafe -ve right strafe 
        high_cmd_ros.yawSpeed = 0.0f;			// Yaw rotation +ve = left, -ve = right
        high_cmd_ros.reserve = 0;       
     	
     	char mode = 0;
     	char ch = 0;
     	mode = getch ();
     	ch = getch ();
     	
     	
    	printf("Motiontime: %ld\n", motiontime);
     	printf("count++: %ld\n",count++);
     	printf("ch: %d\n\n", ch); 
     	printf("mode: %d\n\n", mode);    
     	
     	//Note to self can I do switch (ch || mode)
     
     switch (mode)
     {
     	case 'z':
	printf("Basic Controller Activated");
	mode = 'z';
	
	case 'x':
	printf("Crawler Mode Activated");
	mode = 'x';
	
	case 'c':
	printf("Stair Controller Activated");
	mode = 'c';	

	}
     	 	       
while (mode == 'z' || mode == 'x' || mode == 'c')
{


	if (mode == 'z')
	{
		switch (ch)
		{	
     			case 'q':
     		
     				printf("already quit!\n");
     				return 0;
     		
     		
     			case 'w':			//Walk forward
     				high_cmd_ros.mode = 2;				
     				high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
        			high_cmd_ros.velocity[0] = 0.6f; 		// -1  ~ +1
     				high_cmd_ros.yawSpeed = 0;			//rotate Theta left (2 is approx 180 degrees)
          			printf("trotting forward\n");   		
          			break;
     		
     			case 'a':			//turning left
            			high_cmd_ros.mode = 2;				
            			high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            			high_cmd_ros.velocity[0] = 0.6f; 			// -1  ~ +1
            			high_cmd_ros.yawSpeed = 0.5;			//Speed of rotation whilst walking (left) 0.5 = 45 degrees approx
            			high_cmd_ros.footRaiseHeight = 0.1;
            			printf("trotting left\n");
            			break;
        
      			case 'd':			//turning right
            			high_cmd_ros.mode = 2;				
           			high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            			high_cmd_ros.velocity[0] = 0.6f; 			// -1  ~ +1
             			high_cmd_ros.yawSpeed = -0.5;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
      	 			high_cmd_ros.footRaiseHeight = 0.1;
      	 			printf("trotting right\n");
            			break;
        	
        	
       		case 's':			// reverse
           			high_cmd_ros.mode = 2;				
            			high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            			high_cmd_ros.velocity[0] = -0.4f; 		// -1  ~ +1
            			high_cmd_ros.yawSpeed = 0;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
               		printf("Reverse trotting\n");
            			break;
      
      			case 'e':
      				high_cmd_ros.mode = 1; //Force Stand			
               		printf("Force Stand\n");
      				break;
            	 
       	}
       }	


	else if (mode == 'x')		//CRAWLING MODE
	{	
		switch (ch)
		{	
     			case 'q':
     		
     				printf("already quit!\n");
     				return 0;
     		
     		
     			case 'w':			//Walk forward
     				high_cmd_ros.mode = 2;				
     				high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
        			high_cmd_ros.velocity[0] = 0.6f; 		// -1  ~ +1
        			high_cmd_ros.bodyHeight = 0.5;		// body height 0.5
     				high_cmd_ros.yawSpeed = 0;			//rotate Theta left (2 is approx 180 degrees)
          			printf("trotting forward\n");   		
          			break;
     		
     			case 'a':			//turning left
            			high_cmd_ros.mode = 2;				
            			high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            			high_cmd_ros.velocity[0] = 0.6f; 			// -1  ~ +1
            			high_cmd_ros.bodyHeight = 0.5;		// body height 0.5
            			high_cmd_ros.yawSpeed = 0.5;			//Speed of rotation whilst walking (left) 0.5 = 45 degrees approx
            			high_cmd_ros.footRaiseHeight = 0.1;
            			printf("trotting left\n");
            			break;
        
      			case 'd':			//turning right
            			high_cmd_ros.mode = 2;				
           			high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            			high_cmd_ros.velocity[0] = 0.6f; 			// -1  ~ +1
            			high_cmd_ros.bodyHeight = 0.5;		// body height 0.5
             			high_cmd_ros.yawSpeed = -0.5;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
      	 			high_cmd_ros.footRaiseHeight = 0.1;
      	 			printf("trotting right\n");
            			break;
        	
        	
       		case 's':			// reverse
           			high_cmd_ros.mode = 2;				
            			high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            			high_cmd_ros.velocity[0] = -0.4f; 		// -1  ~ +1
            			high_cmd_ros.bodyHeight = 0.5;		// body height 0.5
            			high_cmd_ros.yawSpeed = 0;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
               		printf("Reverse trotting\n");
            			break;
      
      			case 'e':
      				high_cmd_ros.mode = 1; //Force Stand			
               		printf("Force Stand\n");
      				break;
            	 
       	}
       }		


	else (mode == 'c');
	{
		switch (ch)		//STAIR MODE
		{	
     			case 'q':
     		
     				printf("already quit!\n");
     				return 0;
     		
     		
     			case 'w':			//Walk forward
     				high_cmd_ros.mode = 3;				
     				high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
        			high_cmd_ros.velocity[0] = 0.6f; 		// -1  ~ +1
     				high_cmd_ros.yawSpeed = 0;			//rotate Theta left (2 is approx 180 degrees)
          			printf("trotting forward\n");   		
          			break;
     		
     			case 'a':			//turning left
            			high_cmd_ros.mode = 3;				
            			high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            			high_cmd_ros.velocity[0] = 0.6f; 			// -1  ~ +1
            			high_cmd_ros.yawSpeed = 0.5;			//Speed of rotation whilst walking (left) 0.5 = 45 degrees approx
            			high_cmd_ros.footRaiseHeight = 0.1;
            			printf("trotting left\n");
            			break;
        
      			case 'd':			//turning right
            			high_cmd_ros.mode = 3;				
           			high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            			high_cmd_ros.velocity[0] = 0.6f; 			// -1  ~ +1
             			high_cmd_ros.yawSpeed = -0.5;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
      	 			high_cmd_ros.footRaiseHeight = 0.1;
      	 			printf("trotting right\n");
            			break;
        	
        	
       		case 's':			// reverse
           			high_cmd_ros.mode = 3;				
            			high_cmd_ros.gaitType = 2;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            			high_cmd_ros.velocity[0] = -0.4f; 		// -1  ~ +1
            			high_cmd_ros.yawSpeed = 0;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
               		printf("Reverse trotting\n");
            			break;
      
      			case 'e':
      				high_cmd_ros.mode = 1; //Force Stand			
               		printf("Force Stand\n");
      				break;
            	 
       	}
       }		
}

       pub.publish(high_cmd_ros);
       

       ros::spinOnce();
       loop_rate.sleep();
        
    }

    return 0;
    
}
