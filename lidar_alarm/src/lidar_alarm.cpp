// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <math.h> //for cos
#include <std_srvs/Trigger.h>


const double MIN_SAFE_DISTANCE = 1.5; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
bool alarm_prev = false;

double robot_radius = 0.25; //taken from the simulator, and added a bit (+0.05) to give a little more room
double switch_angle_pos = .253; //positive angle to swap between 1m checks and straight-line checks
double switch_angle_neg = -.253; //negative angle to swap between 1m checks and straight-line checks 
double side_angle_pos = 1.57; //positive angle straight to the side
double side_angle_neg = -1.57; //negative angle straight to the side
int current_index = 0; //keeps track of current index
double current_angle = 0.0; //keeps track of current angle

ros::ServiceClient eStopClient;
ros::ServiceClient eStopClearClient;

// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);   
    }

    current_index = 0;
    current_angle = angle_min_;
    laser_alarm_ = false;
    while(current_angle < angle_max_){ //loop until all ranges have been checked
        if(current_angle < switch_angle_pos && current_angle > switch_angle_neg){ 
            //range in which we can just check if the range is less than 1m
            if(laser_scan.ranges[current_index] < MIN_SAFE_DISTANCE){
                ROS_WARN("DANGER IN FRONT");
                laser_alarm_=true;
                break; //leave the loop
            }
        }
        else if((current_angle < switch_angle_neg && current_angle > side_angle_neg) ||
                (current_angle > switch_angle_pos && current_angle < side_angle_pos)){
            //angles in which we want to do some trig to make sure its safe
            double safe_range = robot_radius/ sin(current_angle);
            if(current_angle < 0){
                safe_range = -safe_range;
            }
            if(laser_scan.ranges[current_index] < safe_range){
                ROS_WARN("DANGER ON SIDES");
                ROS_INFO("current_angle =  %f, current_index = %d, safe_range = %f, actual range = %f",current_angle, current_index, safe_range, laser_scan.ranges[current_index] );
                laser_alarm_=true;
                break; //leave the loop
            }
        }
        //get here by either not being in needed angle ranges or if range is ok
        current_index++;
        current_angle += angle_increment_;
    }
    float ping_dist_in_front_ = laser_scan.ranges[ping_index_];
    std_srvs::Trigger srv;
    if(alarm_prev != laser_alarm_){
        if(laser_alarm_){
            eStopClient.call(srv);
        }
        else{
            eStopClearClient.call(srv);
        }
    }
   
   alarm_prev = laser_alarm_;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh;
    eStopClient = nh.serviceClient<std_srvs::Trigger>("estop_service");
    eStopClearClient = nh.serviceClient<std_srvs::Trigger>("clear_estop_service");
    ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

