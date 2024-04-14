/**
 * @file dr_timer.cpp
 * @author vcb (www.deeprobotics.cn)
 * @brief 
 * @version 0.1
 * @date 2023-03-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "dr_timer.h"

using namespace std;
using namespace lite3;

void DRTimer::TimeInit(int ms){
  if(ms >= 1000){
    cout << "Error Too long period" << endl;
    exit(1);
  }
  struct itimerspec time_intv; 
  tfd_ = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
  if( tfd_ == -1) {
    printf("create timer fd fail \r\n");
  }
  time_intv.it_value.tv_sec = 0; //设定2s超时
  time_intv.it_value.tv_nsec = 1000*1000*ms;
  time_intv.it_interval.tv_sec = time_intv.it_value.tv_sec;   //每隔2s超时
  time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

  printf("timer start ...\n");
  timerfd_settime( tfd_, 0, &time_intv, NULL);  //启动定时器

  efd_ = epoll_create1(0); //创建epoll实例
  if (efd_ == -1) {
      printf("create epoll fail \r\n");
      close( tfd_);
  }

  evptr_ = (struct epoll_event *)calloc(1, sizeof(struct epoll_event));
  if (evptr_ == NULL) {
      printf("epoll event calloc fail \r\n");
      close( tfd_);
      close(efd_);
  }

  ev_.data.fd =  tfd_; 
  ev_.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
  epoll_ctl(efd_, EPOLL_CTL_ADD,  tfd_, &ev_); //添加到epoll监听队列中
}



bool DRTimer::TimerInterrupt(){
  int ret = 0;
  uint64_t value = 0;
  fds_ = epoll_wait(efd_, evptr_, 1, -1);    //阻塞监听，直到有事件发生
    if(evptr_[0].events & EPOLLIN){   
    ret = read(evptr_->data.fd, &value, sizeof(uint64_t));
      if (ret == -1) {
        printf("read return 1 -1, errno :%d \r\n", errno);
        return true;
      }            
    }
  return false;
}

double DRTimer::GetIntervalTime(double start_time){
  timespec system_time;

  clock_gettime(1,&system_time);  
  return system_time.tv_sec + system_time.tv_nsec/1e9 - start_time;
}

double DRTimer::GetCurrentTime(){
  timespec system_time;

  clock_gettime(1,&system_time);
  return system_time.tv_sec + system_time.tv_nsec/1e9;
}
