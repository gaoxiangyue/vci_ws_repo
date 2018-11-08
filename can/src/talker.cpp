#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


#include <canlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include "darknet_ros_msgs/TrackingBoxes.h"

#define ALARM_INTERVAL_IN_S     (1)
#define WRITE_WAIT_INFINITE     (unsigned long)(-1)

static unsigned int msgCounter = 0;
static int willExit = 0;

static void check(char* id, canStatus stat)
{
  if (stat != canOK) {
    //char buf[50];
    //buf[0] = '\0';
    //canGetErrorText(stat, buf, sizeof(buf));
    //printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  }
}

static void sighand(int sig)
{
  static unsigned int last;

  switch (sig) {
  case SIGINT:
    willExit = 1;
    break;
  case SIGALRM:
    if (msgCounter - last) {
      printf("msg/s = %d, total=%u\n",
             (msgCounter - last) / ALARM_INTERVAL_IN_S, msgCounter);
    }
    last = msgCounter;
    alarm(ALARM_INTERVAL_IN_S);
    break;
  }
}

static void printUsageAndExit(char *prgName)
{
  printf("Usage: '%s <channel>'\n", prgName);
  exit(1);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
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
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

////////////////////
  canHandle hnd;
  canStatus stat;
  char canmsg[8] = "";
  int channel;

  if (argc != 2) {
    printUsageAndExit(argv[0]);
  }

  {
    char *endPtr = NULL;
    errno = 0;
    channel = strtol(argv[1], &endPtr, 10);
    if ( (errno != 0) || ((channel == 0) && (endPtr == argv[1])) ) {
      printUsageAndExit(argv[0]);
    }
  }

  printf("Writing messages on channel %d\n", channel);

  /* Use sighand as our signal handler */
  signal(SIGALRM, sighand);
  signal(SIGINT, sighand);
  alarm(ALARM_INTERVAL_IN_S);

  /* Allow signals to interrupt syscalls */
  siginterrupt(SIGINT, 1);

  canInitializeLibrary();

  /* Open channel, set parameters and go on bus */
  hnd = canOpenChannel(channel, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
  if (hnd < 0) {
    printf("canOpenChannel %d", channel);
    //check("", hnd);
    return -1;
  }
  stat = canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);
  check("canSetBusParams", stat);
  if (stat != canOK) {
    //goto ErrorExit;
  }
  stat = canBusOn(hnd);
  check("canBusOn", stat);
  if (stat != canOK) {
    //goto ErrorExit;
  }


/////////////////////
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    //std_msgs::String Rmsg;

    //std::stringstream ss;
    //ss << "hello world " << count;
    //Rmsg.data = ss.str();

    //ROS_INFO("%s", Rmsg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //chatter_pub.publish(Rmsg);
///////////////////////////
    if ((stat == canOK) && !willExit) {
      long id = channel + 100;
//void canCallback(const darknet_ros_msgs::TrackingBoxes::ConstPtr& msg)
{
 int a;
 float b;
 float c;
 float d;
 float e;
}
 // std_msgs/Header a ;
//a= msg->tracking_boxes[0].class_no;
//b= msg->tracking_boxes[1].dis_x;
//c= msg->tracking_boxes[2].dis_y;
//d= msg->tracking_boxes[3].speed_x;
//e= msg->tracking_boxes[4].speed_y;  
//}
{
//canmsg[0] = tracking_boxes[0];
//canmsg[0] = (char)10.2;
//canmsg[1] = 65;
//canmsg[2] = 45;
//canmsg[3] = 64;
//canmsg[4] = 01;
//canmsg[5] = 00;
//canmsg[6] = 01;
//canmsg[7] = 02;
;
}
       stat = canWriteWait(hnd, id, canmsg, sizeof(canmsg) / sizeof(canmsg[0]), canMSG_EXT, WRITE_WAIT_INFINITE);
     //stat = canWriteWait(hnd, id, &a, sizeof(msg), canMSG_EXT, WRITE_WAIT_INFINITE);
      if (errno == 0) {
      //check("\ncanWriteWait", stat);
      }
      else {
      //perror("\ncanWriteWait error");
      }
      if (stat == canOK) {
        msgCounter++;
      }
    }

////////////////////////////
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
