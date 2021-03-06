#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include "drive_toy_car/MsgDriverCmd.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

#define KEYCODE_q 0x71
#define KEYCODE_Q 0x51    // 设置车辆的移动方向：前景或者后退
#define KEYCODE_SP 0x20   // 停止、启动车辆

#define STOP_THROTTLE 5.1  //如果油门开度低于这个值，玩具车将停止运动
#define MAX_THROTTLE  15.0  //允许的最大油门开度，防止车辆速度过快
#define START_THROTTLE 9.0 //这辆玩具车从静止状态启动的时候，必须给出大于这个油门开度的值，才能够启动
#define MAX_STEER 100.0      //允许的最大的方向转角
#define DUR_TIME 0.2        // 启动后油门的持续时间，之后，油门将自动降为停车油门值STOP_THROTTLE


class TeleopToyCar
{
public:
  TeleopToyCar();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double throttle_, steer_, t_step, s_step;
  bool forward_moving,on_moving;
  ros::Publisher car_cmd_pub_;
  
};

TeleopToyCar::TeleopToyCar():
  throttle_(0),
  steer_(0),
  t_step(0.5),
  s_step(6),
  forward_moving(true),
  on_moving(false)
{

  car_cmd_pub_ = nh_.advertise<drive_toy_car::MsgDriverCmd>("/car_cmd", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_toycar");
  puts("forward direction, stoped");
  TeleopToyCar teleop_toycar;

  signal(SIGINT,quit);

  teleop_toycar.keyLoop();
  
  return(0);
}


void TeleopToyCar::keyLoop()
{
  char c;
  bool dirty=false;
  drive_toy_car::MsgDriverCmd drive_cmd;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to drive the toy car.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

  

    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        steer_ -= s_step;
        steer_ = std::max(steer_,-MAX_STEER);
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        steer_ += s_step;
        steer_ = std::min(steer_,MAX_STEER);
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        if(!on_moving)
          break;
        
        if (throttle_ <= - STOP_THROTTLE){  // 油门值为负，且大于停车油门值
          throttle_ -= t_step;                
          throttle_ = std::max(throttle_,-MAX_THROTTLE);  //限制反向油门最小值，防止车辆停车
        }
        else if(throttle_ >= STOP_THROTTLE){ //油门值为正，且大于停车油门值
          throttle_ += t_step; 
          throttle_ = std::min(throttle_,MAX_THROTTLE); //限制正向油门最大值，防止车辆超速  
        }   
        else
          throttle_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        if (throttle_ >= STOP_THROTTLE){ //油门值为正，且大于停车油门值
          throttle_ -= t_step;
          throttle_ =std::max(throttle_,STOP_THROTTLE);  //限制正向油门的最小值，防止车辆停车
        }
        else if(throttle_ <= -STOP_THROTTLE){ //油门值为负，且大于停车油门值
          throttle_ += t_step;
          throttle_ = std::min(throttle_,-STOP_THROTTLE); //限制反向油门的最大值，防止车辆超速
        }
        else
          throttle_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_SP:
        ROS_DEBUG("SPACE BAR");
        if(on_moving){
          throttle_ = 0.0;  //立即停车
          puts("Car Stoped");
          dirty = true;
          on_moving=false;
        }
        else{
                //从静止状态启动车辆的时候，必须给一个大的油门绝对值，否则车辆不能启动.启动后，自动将油门降为最低，这样做是担心人为控制来不及处理
          if(forward_moving){
            throttle_ = START_THROTTLE;
            drive_cmd.throttle_pos = throttle_;
            car_cmd_pub_.publish(drive_cmd);   
            sleep(DUR_TIME);

            throttle_ = STOP_THROTTLE;
            drive_cmd.throttle_pos = throttle_;
            car_cmd_pub_.publish(drive_cmd); 

          }
          else{
            throttle_ =-START_THROTTLE;
            drive_cmd.throttle_pos = throttle_;
            car_cmd_pub_.publish(drive_cmd);   
            sleep(DUR_TIME);

            throttle_ = -STOP_THROTTLE;     
            drive_cmd.throttle_pos = throttle_;
            car_cmd_pub_.publish(drive_cmd);                  
          }  
          puts("Car Started");
          on_moving=true;
        }
        break;
      case KEYCODE_q:
      case KEYCODE_Q:
        if(on_moving){
          puts("Can not change direction while car is moving!!!");
          break;
        }
        forward_moving = ! forward_moving;
        if(forward_moving)
          puts("Forward Moving");
        else
          puts("Backword Moving!");
        break;

    }
   
    drive_cmd.throttle_pos = throttle_;
    drive_cmd.steer_pos = steer_;

    if(dirty ==true)
    {
      car_cmd_pub_.publish(drive_cmd);    
      dirty=false;
    }
  }


  return;
}



