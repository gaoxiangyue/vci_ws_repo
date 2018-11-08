#include "ros/ros.h"
#include "std_msgs/String.h"
#include "darknet_ros_msgs/TrackingBoxes.h"
#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

#define WRITE_WAIT_INFINITE     (unsigned long)(-1)

int msgId=0;
canHandle hnd;
canStatus stat;
int channel=1;

static void check(char* id, canStatus stat)
{
  if (stat != canOK) {
    char buf[50];
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  }
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
//ROS_INFO("I heard: [%d]", msg->tracking_boxes[0].class_no);
}

//void canCallback(const darknet_ros_msgs::TrackingBoxes::ConstPtr& msg)
void canCallback(const darknet_ros_msgs::TrackingBoxes::ConstPtr& msg)
{
//printf("haha");

 int a;
 int dis[4] = {0};
 int dis1[4] = {0};
 int dis2[4] = {0};
 int dis3[4] = {0};
//int dis[4] = {0};
//const_cast<int>dis[0] = msg->tracking_boxes[0].dis_x;
 // const_cast<int>dis[1] = msg->tracking_boxes[0].dis_y;
  //const_cast<int>dis[2] = msg->tracking_boxes[0].speed_x;
  //const_cast<int>dis[3] = msg->tracking_boxes[0].speed_y;
  dis[0] = (int)(msg->tracking_boxes[0].dis_x*100);
  dis[1] = (int)(msg->tracking_boxes[0].dis_y*100);
  dis[2] = (int)(msg->tracking_boxes[0].speed_x*100);
  dis[3] = (int)(msg->tracking_boxes[0].speed_y*100);
/* 
 dis1[0] = (int)(msg->tracking_boxes[0].dis_x*100);
  dis1[1] = (int)(msg->tracking_boxes[0].dis_y*100);
  dis1[2] = (msg->tracking_boxes[0].speed_x);
  dis1[3] = (msg->tracking_boxes[0].speed_y);
  dis2[0] = (int)(msg->tracking_boxes[0].dis_x*100);
  dis2[1] = (int)(msg->tracking_boxes[0].dis_y*100);
  dis2[2] = (msg->tracking_boxes[0].speed_x);
  dis2[3] = (msg->tracking_boxes[0].speed_y);
  dis3[0] = (int)(msg->tracking_boxes[0].dis_x*100);
  dis3[1] = (int)(msg->tracking_boxes[0].dis_y*100);
  dis3[2] = (msg->tracking_boxes[0].speed_x);
  dis3[3] = (msg->tracking_boxes[0].speed_y);  
*/
//(int)(dis[0]*100);
  //(int)(dis[1]*100);
  //(int)(dis[2]*100);
  //(int)(dis[3]*100);
  //disInt[0] = 1;
  //disInt[1] = 2;
  //disInt[2] = 3;
  //disInt[3] 
 // std_msgs/Header a ;
printf("dis[0]=%d\n",dis[0]);
printf("dis[1]=%d\n",dis[1]);
printf("dis[2]=%d\n",dis[2]);
printf("dis[3]=%d\n",dis[3]);
a= msg->tracking_boxes[0].class_no;
//printf("a=%d",a);
//stat = canWriteWait(hnd, msgId++, dis+2, 2*sizeof(float), canMSG_EXT,WRITE_WAIT_INFINITE);
stat = canWriteWait(hnd, 0x100, dis, 2*sizeof(int), canMSG_EXT,WRITE_WAIT_INFINITE);
stat = canWriteWait(hnd, 0x101, dis+2, 2*sizeof(int), canMSG_EXT,WRITE_WAIT_INFINITE);
//stat = canWriteWait(hnd, 0x102, dis1, 2*sizeof(int), canMSG_EXT,WRITE_WAIT_INFINITE);
//stat = canWriteWait(hnd, 0x103, dis1+2, 2*sizeof(float), canMSG_EXT,WRITE_WAIT_INFINITE);
//stat = canWriteWait(hnd, 0x104, dis2, 2*sizeof(float), canMSG_EXT,WRITE_WAIT_INFINITE);
//stat = canWriteWait(hnd, 0x105, dis2+2, 2*sizeof(float), canMSG_EXT,WRITE_WAIT_INFINITE);
//stat = canWriteWait(hnd, 0x106, dis3, 2*sizeof(float), canMSG_EXT,WRITE_WAIT_INFINITE);
//stat = canWriteWait(hnd, 0x107, dis3+2, 2*sizeof(float), canMSG_EXT,WRITE_WAIT_INFINITE);
//stat = canWriteWait(hnd, 0x100, disInt, 2*sizeof(int), canMSG_EXT,WRITE_WAIT_INFINITE);
//d= msg->tracking_boxes[0].speed_x;
//e= msg->tracking_boxes[0].speed_y; 
  
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  //printf("1111\n");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
canInitializeLibrary();
  //printf("2222\n");
hnd = canOpenChannel(channel, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
  //printf("3333\n");
  if (hnd < 0) {
    printf("canOpenChannel %d Error", channel);
  stat = canUnloadLibrary();
  check("canUnloadLibrary", stat);
    //check("", hnd);
    return 0;
  }
stat = canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);
if (stat != canOK) {
    printf("canSetBusParams Error");

  stat = canClose(hnd);
  check("canClose", stat);
  stat = canUnloadLibrary();
  check("canUnloadLibrary", stat);
return 0;
//goto ErrorExit;
  }
  stat = canBusOn(hnd);
  check("canBusOn", stat);
  if (stat != canOK) {    
    printf("canBusOn Error");
  stat = canClose(hnd);
  check("canClose", stat);
  stat = canUnloadLibrary();
  check("canUnloadLibrary", stat);
return 0;
  }
  //printf("4444\n");
  ros::Subscriber subcan = n.subscribe("/darknet_ros/vision_pub", 1000, canCallback);

   // printf("6666\n");

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  stat = canBusOff(hnd);
  check("canBusOff", stat);
  stat = canClose(hnd);
  check("canClose", stat);
  stat = canUnloadLibrary();
  check("canUnloadLibrary", stat);
  return 0;
}
