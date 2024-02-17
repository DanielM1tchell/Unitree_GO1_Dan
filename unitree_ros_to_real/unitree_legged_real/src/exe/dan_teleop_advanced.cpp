//TITLE: Teleoperation code to operate the Unitree GO1
//Instructions: Used WASD for forwards, reverse, left/right (on the spot) z and x are left/right whilst moving forward (curved)
//Instructions 2: Use keyboard commands 1,2 or 3 to change the modes from walking, crawling and stair mode respectively
//AUTHOR 1: Daniel Mitchell University of Glasgow and California Institute of Technology
//AUTHOR 2: Yukin Ann California Institute of Technology
//AUTHOR 3: John Lathrop California Institute of Technology

//Citation for publication: TBC

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


//create a function for 3 types of controls- Walk- Crawl- Stairs
void walk_mode(char mode, char ch, unitree_legged_msgs::HighCmd & high_cmd_ros)
{	
	switch(ch)
		{
		case 'q':
     		printf("already quit!\n");
     		return;
     		
    	case 'w':			//Walk forward
			
     		high_cmd_ros.mode = 2;		
     		high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
        	high_cmd_ros.velocity[0] = 0.8f; 		// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
     		high_cmd_ros.yawSpeed = 0;			//rotate Theta left (2 is approx 180 degrees)
			high_cmd_ros.footRaiseHeight = 0;
          	printf("forward_walk\n");   		
          	break;
     		
    	case 'a':			//turning left on the spot
            high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = 0.0f; 			// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
            high_cmd_ros.yawSpeed = 0.8;			//Speed of rotation whilst walking (left) 0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0;
            printf("left_walk\n");
            break;
        
		case 'd':			//turning right on the spot
            high_cmd_ros.mode = 2;				
           	high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 	
            high_cmd_ros.velocity[0] = 0.0f; 			// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
            high_cmd_ros.yawSpeed = -0.8;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0;
      	 	printf("right_walk\n");
            break;
        	
    	case 's':			// reverse
           	high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 	
            high_cmd_ros.velocity[0] = -0.4f; 		// -1  ~ +1}
			high_cmd_ros.bodyHeight = 0.0;
            high_cmd_ros.yawSpeed = 0;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0;
            printf("Reverse_walk\n");
            break;

		case 'z':			//turning left  curve
            high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = 0.4f; 			// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
            high_cmd_ros.yawSpeed = 0.8;			//Speed of rotation whilst walking (left) 0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0;
            printf("left curve_walk\n");
            break;
        
		case 'x':			//turning right curve
            high_cmd_ros.mode = 2;				
           	high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 	
            high_cmd_ros.velocity[0] = 0.4f; 			// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
            high_cmd_ros.yawSpeed = -0.8;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0;
      	 	printf("right curve_walk\n");
            break;
      
    	case 'e':
      		high_cmd_ros.mode = 1; //Force Stand			
            printf("Force Stand- walk mode\n");
      		break;
      					
    	default:
       		printf("Stopped\n");
		}
}

void crawl_mode(char mode, char ch, unitree_legged_msgs::HighCmd & high_cmd_ros)
{
	switch(ch)
		{
    	case 'q':
    		printf("already quit!\n");
    		return;
     		 		
    	case 'w':			//Walk forward
   			high_cmd_ros.mode = 2;				
    		high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
    		high_cmd_ros.velocity[0] = 0.6f; 		// -1  ~ +1
    		high_cmd_ros.bodyHeight = -0.3;		// body height 0.5
    		high_cmd_ros.yawSpeed = 0;			//rotate Theta left (2 is approx 180 degrees)
			high_cmd_ros.footRaiseHeight = 0;
    		printf("forward crawl\n");   		
    		break;
     		
    	case 'a':			//turning left
    		high_cmd_ros.mode = 2;				
    		high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
    		high_cmd_ros.velocity[0] = 0.0f; 			// -1  ~ +1
    		high_cmd_ros.bodyHeight = -0.3;		// body height 0.5
    		high_cmd_ros.yawSpeed = 0.8;			//Speed of rotation whilst walking (left) 0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0;
    		printf("left crawl\n");
    		break;
        
    	case 'd':			//turning right
    		high_cmd_ros.mode = 2;				
           	high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = 0.0f; 			// -1  ~ +1
            high_cmd_ros.bodyHeight = -0.3;		// body height 0.5
            high_cmd_ros.yawSpeed = -0.8;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0;
      	 	printf("right crawl\n");
            break;
		
		case 'z':			//turning left  curve
            high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = 0.4f; 			// -1  ~ +1
			high_cmd_ros.bodyHeight = -0.3;
            high_cmd_ros.yawSpeed = 0.8;			//Speed of rotation whilst walking (left) 0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0;
            printf(" left curve crawl\n");
            break;
        
		case 'x':			//turning right curve
            high_cmd_ros.mode = 2;				
           	high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 	
            high_cmd_ros.velocity[0] = 0.4f; 			// -1  ~ +1
			high_cmd_ros.bodyHeight = -0.3;
            high_cmd_ros.yawSpeed = -0.8;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0;
      	 	printf("right curve walk\n");
            break;
        	
        	
    	case 's':			// reverse
        	high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 1;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = -0.3f; 		// -1  ~ +1
            high_cmd_ros.bodyHeight = -0.3;		// body height 0.5
            high_cmd_ros.yawSpeed = 0;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0;
            printf("Reverse crawl\n");
            break;
      
    	case 'e':
      		high_cmd_ros.mode = 1; //Force Stand			
            printf("Force Stand- crawl mode\n");
      		break;
      					
    	default:
       		printf("Stop!\n");            	 
    	}
	
}

void stair_mode(char mode, char ch, unitree_legged_msgs::HighCmd & high_cmd_ros)
{
	switch(ch)
		{
    	case 'q':
    		printf("already quit!\n");
    		return;
     		 		
    	case 'w':			//Walk forward
   			high_cmd_ros.mode = 2;				
    		high_cmd_ros.gaitType = 3;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
    		high_cmd_ros.velocity[0] = 0.6f; 		// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
    		high_cmd_ros.yawSpeed = 0;			//rotate Theta left (2 is approx 180 degrees)
			high_cmd_ros.footRaiseHeight = 0.8;
    		printf("forward_stair walk\n");   		
    		break;
     		
    	case 'a':			//turning left
    		high_cmd_ros.mode = 2;				
    		high_cmd_ros.gaitType = 3;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
    		high_cmd_ros.velocity[0] = 0.0f; 			// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
    		high_cmd_ros.yawSpeed = 0.8;			//Speed of rotation whilst walking (left) 0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0.8;
    		printf(" left stair walk\n");
    		break;
        
    	case 'd':			//turning right
    		high_cmd_ros.mode = 2;				
           	high_cmd_ros.gaitType = 3;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = 0.0f; 			// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
            high_cmd_ros.yawSpeed = -0.8;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0.8;
      	 	printf("right stair walk\n");
            break;

		case 'z':			//turning left  curve
            high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 3;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = 0.4f; 			// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
            high_cmd_ros.yawSpeed = 0.8;			//Speed of rotation whilst walking (left) 0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0.8;
            printf("left curve stair walk\n");
            break;
        
		case 'x':			//turning right curve
            high_cmd_ros.mode = 2;				
           	high_cmd_ros.gaitType = 3;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 	
            high_cmd_ros.velocity[0] = 0.4f; 			// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
            high_cmd_ros.yawSpeed = -0.8;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0.8;
      	 	printf("right curve stair walk\n");
            break;
        	
    	case 's':			// reverse
        	high_cmd_ros.mode = 2;				
            high_cmd_ros.gaitType = 3;			//walk at specified velocity (0= idle, 1=trot, 2=trot running, 3=climb stair, 4=trot obstacle) 
            high_cmd_ros.velocity[0] = -0.2f; 		// -1  ~ +1
			high_cmd_ros.bodyHeight = 0.0;
            high_cmd_ros.yawSpeed = 0;			//Speed of rotation whilst walking (right) -0.5 = 45 degrees approx
			high_cmd_ros.footRaiseHeight = 0.8;
            printf("Reverse stair walk\n");
            break;
      
    	case 'e':
      		high_cmd_ros.mode = 1; //Force Stand			
            printf("Force Stand- stair mode\n");
      		break;
      					
    	default:
       		printf("Stop!\n");            	 
		}
	
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
    
	char ch = 'e';
    char mode = '1';

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

   while (ros::ok())
	{

        motiontime += 2;
        motiontimedelete += 2;

     	
     	//char ch = getch();
		
		//char mode = getch();
		
		char key = getch();			//This is the issue I think... Dan

		if(key == 'w' || key == 'a' || key == 's' || key == 'd' || key == 'e' || key == 'z' || key == 'x' || key == 'q'){
			ch = key;
		}

		else if (key == '1' || key == '2' || key == '3')
		{
			mode = key;
			std::cout << "switched mode to " << mode << std::endl;
		}
		std::cout << "mode is " << mode << std::endl;

		if (key == 'q') { return 0; }
     	
    	printf("%ld\n", motiontime);
     	printf("%ld\n",count++);
     	printf("ch = %c\n\n", ch);     //chat GPT %d changed to %c for char

	 		
     		       
     	switch (mode)
     	{
		case '1':
			printf("BASIC CONTROLLER ACTIVATED");
			walk_mode(mode, ch, high_cmd_ros);
			break;											//CHAT GPT recommendation
				
				
			//if(mode== '2'){
			//	printf("Jumped out mode 1");
			//	break;
			//}	
					
			//else (mode== '3');{
			//	printf("Jumper out mode 1");
			//	break;
			//}

				
				
				
     	case '2':
     		printf("CRAWLER CONTROLLER ACTIVATED ");
     		crawl_mode(mode, ch, high_cmd_ros);
			break;
     			
     		//if(mode== '1' ){
			//	printf("Jumped out mode 2");
		    //	break;
		    //}
			    
		    //else(mode== '3' );{
			//	printf("Jumped out mode 2");
		    //	break;
		    //}
			    
			    
     	
     	case '3':
     		printf("STAIRS CONTROLLER ACTIVATED ");
     		stair_mode(mode, ch, high_cmd_ros);
			break;
     		
		    //if(mode== '1'){
			//	printf("Jumped out mode 3");
		    //	break;
		    //}
			    
			//else (mode== '2');{
			//	printf("Jumped out mode");
			//	break;
			//}

		default:
        printf("Invalid mode\n");
			    
				
    	}
	
    	pub.publish(high_cmd_ros);
       
    	ros::spinOnce();
    	loop_rate.sleep();
        
    }
	return 0;
}