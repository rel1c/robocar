#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <cstdlib>
#include "ros/ros.h"
#include "autodriver/ControlState.h"
#include "autodriver/teleop.h"

struct termios new_term, old_term;

/* restore terminal to previous settings: input echoing and blocking stdin read */
void on_exit(int sig) {
  tcsetattr(STDIN_FILENO, TCSANOW, &old_term);  
  ros::shutdown();
}

/* set message to default, at rest control state */
void init_state(autodriver::ControlState &msg) {
  msg.motor_pct = 0.0;
  msg.heading = 90;
  msg.reverse = 0;
}


int main(int argc, char *argv[]) {
  struct sigaction act;
  sigemptyset(&act.sa_mask);
  act.sa_handler = on_exit;
  act.sa_flags = 0;

  // on interrupt handler
  sigaction(SIGINT, &act, NULL);

  // eliminate terminal input buffering for non-blocking stdin read 
  tcgetattr(STDIN_FILENO, &old_term);
  new_term = old_term;
  new_term.c_lflag &= ~ICANON & ~ECHO;        
  tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

  ros::init(argc, argv, "driver_teleop");
  ros::NodeHandle nh;
  ros::Publisher control_stream = nh.advertise<autodriver::ControlState>("control_data", 10);

  autodriver::ControlState msg;
  init_state(msg);
  int speed = 0;

  std::cout << "Use arrow keys for navigation, press q to quit. Reverse is activated by continuing to "
    "decrement speed when stopped.\n";

  while (ros::ok()) {
    char input = getchar();

    switch (input) {
      case KEY_UP: 
        if (speed == 0) {
          // stopped or reverse to forward
          msg.reverse = 0;
          control_stream.publish(msg);
          ++speed;
        }
        else if (speed < 0 || speed < NUM_SPD_INTRVL) ++speed;
        msg.motor_pct = SET_MOTOR_PCT(speed);
        control_stream.publish(msg);
        break;
      
      case KEY_DOWN:
        if (speed == 0) {
          // stopped or forward to reverse
          msg.reverse = 1;
          control_stream.publish(msg);
          --speed;
        }
        else if (speed > 0 || speed > -NUM_SPD_INTRVL) --speed;
        msg.motor_pct = SET_MOTOR_PCT(speed);
        control_stream.publish(msg);
        break;

      case KEY_LEFT:
        if (msg.heading - TURN_ANGLE >= STEER_MIN) msg.heading -= TURN_ANGLE;
        else msg.heading = STEER_MIN;
        control_stream.publish(msg);
        break;

      case KEY_RIGHT:
        if (msg.heading + TURN_ANGLE <= STEER_MAX) msg.heading += TURN_ANGLE;
        else msg.heading = STEER_MAX;
        control_stream.publish(msg);
        break;

      case KEY_QUIT:
        init_state(msg);
        control_stream.publish(msg);
        on_exit(SIGINT);
    }
  }
  return 0;
}
