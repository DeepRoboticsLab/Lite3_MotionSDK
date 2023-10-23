# 绝影Lite3运动控制SDK

[English](https://github.com/DeepRoboticsLab/Lite3_MotionSDK#readme)

&nbsp;
## 1 SDK更新记录

### V1.0（2023-03-31） 
首次发布。 
### V1.1（2023-05-16）
**[新增]** ***received.h***中添加`RegisterCallBack`回调函数：目前仅支持在机器人数据每次更新时调用回调函数，对应的函数参数值，即`std::function<void(int)> CallBack_`中的`int`值为`0x0906`。  
**[修改]** ***main.cpp***中打印的数据，由原先的关节力矩，改为陀螺仪角加速度。


&nbsp;
## 2 SDK下载及解压

- 下载 **Lite3_MotionSDK**，并解压。


&nbsp;
## 3 远程连接

开发者可通过SSH远程连接到运动主机。

- 将开发主机连接到机器人WiFi。

- 在开发主机上打开SSH连接软件，输入`ssh firefly@192.168.1.120`，密码为 `firefly`，即可远程连接运动主机。

- 输入以下命令以打开网络配置文件：
	```Bash
	cd /home/firefly/jy_exe/conf/
	vim network.toml
	```
- 配置文件 ***network.toml*** 内容如下：
	```toml
	ip = '192.168.1.102'
	target_port = 43897
	local_port = 43893
	~
	```
	
- 修改配置文件第一行中的IP地址，使得 **MotionSDK** 能够接收到机器狗数据:
	- 如果 **MotionSDK** 在机器人运动主机内运行，IP设置为运动主机IP：`192.168.1.120`；  
	- 如果 **MotionSDK** 在开发者自己的开发主机中运行，设置为开发主机的静态IP：`192.168.1.XXX`。
	
- 重启运动程序使配置生效：
	```bash
	cd /home/firefly/jy_exe
	sudo ./stop.sh
	sudo ./restart.sh
	```


&nbsp;
## 4 编译开发

- 编译开发时，开发者可进入解压得到的文件夹，在***CMakeLists.txt*** 的同级目录下新建 ***build*** 文件夹；

	```bash
	cd xxxxxxxx     # cd <path to where you want to create build directory>
	mkdir build
	```
	
	> 注意：开发者可在任何地方创建 ***build*** 文件夹，但在编译时，`cmake` 指令须指向 ***CMakeLists.txt*** 所在的路径。

- 打开 ***build*** 文件夹并编译；

   - 如果主机是x86架构，在终端中输入：

		```bash
		cd build
		cmake .. -DBUILD_PLATFORM=x86     # cmake <path to where the CMakeLists.txt is>
		make -j
		```
		
	- 如果主机是ARM架构，在终端中输入：

		```bash
		cd build
		cmake .. -DBUILD_PLATFORM=arm     # cmake <path to where the CMakeLists.txt is>
		make -j
		```
	
- 编译结束后，会在 ***build*** 目录下生成一个名为 ***Lite_motion*** 的可执行文件，此即为我们代码编译出来的结果；

- 在终端中继续输入以下命令行以运行程序：

	```bash
	./Lite_motion
	```


&nbsp;
## 5 示例代码

本节对 ***main.cpp*** 进行说明。  

定时器，用于设置算法周期，获得当前时间：

```cpp
DRTimer set_timer;
set_timer.TimeInit(int);                              		  ///< Timer initialization, input: cycle; unit: ms
set_timer.GetCurrentTime();                           		  ///< Obtain time for algorithm
set_timer.TimerInterrupt()			      		  ///< Timer interrupt flag
set_timer.GetIntervalTime(double);                    		  ///< Get the current time
```

SDK在绑定机器人的IP和端口后，获取控制权， 发送关节控制指令：

```cpp
Sender* send_cmd = new Sender("192.168.1.120",43893); 		  ///< Create a sender thread
send_cmd->RobotStateInit();                           		  ///< Reset all joints to zero and gain control right
send_cmd->SetSend(RobotCmd); 			     		  ///< Send joint control command
send_cmd->ControlGet(int);                            		  ///< Return the control right
```

SDK接收机器人下发的关节数据：

```cpp
Receiver* robot_data_recv = new Receiver();           		  ///< Create a thread for receiving and parsing
robot_data_recv->GetState(); 			      		  ///< Receive data from 12 joints
robot_data_recv->RegisterCallBack(CallBack);			    ///< Registering Callbacks
```

SDK接收到的关节数据将保存在`robot_data`中：

```cpp
RobotData *robot_data = &robot_data_recv->GetState(); 		  ///< Saving joint data to the robot_data
///< Left front leg：fl_leg[3], the sequence is FL_HipX, FL_HipY, FL_Knee
///< Right front leg：fr_leg[3], the sequence is FR_HipX, FR_HipY, FR_Knee
///< Left hind leg：hl_leg[3], the sequence is HL_HipX, HL_HipY, HL_Knee
///< Right hind leg：hr_leg[3], the sequence is HR_HipX, HR_HipY, HR_Knee
///< All joints：leg_force[12]/joint_data[12], the sequence is FL_HipX, FL_HipY, FL_Knee, FR_HipX, FR_HipY, FR_Knee, HL_HipX, HL_HipY, HL_Knee, HR_HipX, HR_HipY, HR_Knee
	
robot_data->contact_force.fl_leg[]				  ///< Contact force on left front foot in X-axis, Y-axis and Z-axis
robot_data->contact_force.fr_leg[]				  ///< Contact force on right front foot in X-axis, Y-axis and Z-axis
robot_data->contact_force.hl_leg[]				  ///< Contact force on left hind foot in X-axis, Y-axis and Z-axis
robot_data->contact_force.hr_leg[]				  ///< Contact force on right hind foot in X-axis, Y-axis and Z-axis
robot_data->contact_force.leg_force[]			          ///< Contact force on all feet
	
robot_data->tick						  ///< Cycle of operation
	
robot_data->imu							  ///< IMU data	
robot_data->imu.acc_x						  ///< Acceleration on X-axis
robot_data->imu.acc_y						  ///< Acceleration on Y-axis
robot_data->imu.acc_z						  ///< Acceleration on Z-axis
robot_data->imu.angle_pitch					  ///< Pitch angle
robot_data->imu.angle_roll					  ///< Roll angle
robot_data->imu.angle_yaw					  ///< Yaw angle
robot_data->imu.angular_velocity_pitch			  	  ///< Pitch angular velocity
robot_data->imu.angular_velocity_roll			  	  ///< Roll angular velocity
robot_data->imu.angular_velocity_yaw		   	 	  ///< Yaw angular velocity
robot_data->imu.buffer_byte					  ///< Buffer data
robot_data->imu.buffer_float					  ///< Buffer data
robot_data->imu.timestamp					  ///< Time when the data is obtained

robot_data->joint_data						  ///< Motor status
robot_data->joint_data.fl_leg[].position		  	  ///< Motor position of left front leg
robot_data->joint_data.fl_leg[].temperature	  		  ///< Motor temperature of left front leg
robot_data->joint_data.fl_leg[].torque		 	  ///< Motor torque of left front leg 
robot_data->joint_data.fl_leg[].velocity		 	  ///< Motor velocity of left front leg
robot_data->joint_data.joint_data              		  ///< All joint data
```

机器人关节控制指令：

```cpp
RobotCmd robot_joint_cmd;  					  ///< Target data of each joint
///< Left front leg：fl_leg[3], the sequence is FL_HipX, FL_HipY, FL_Knee
///< Right front leg：fr_leg[3], the sequence is FR_HipX, FR_HipY, FR_Knee
///< Left hind leg：hl_leg[3], the sequence is HL_HipX, HL_HipY, HL_Knee
///< Right hind leg：hr_leg[3], the sequence is HR_HipX, HR_HipY, HR_Knee
///< All joints：leg_force[12]/joint_data[12], the sequence is FL_HipX, FL_HipY, FL_Knee, FR_HipX, FR_HipY, FR_Knee, HL_HipX, HL_HipY, HL_Knee, HR_HipX, HR_HipY, HR_Knee

robot_joint_cmd.fl_leg[]->kd;					  ///< Kd of left front leg
robot_joint_cmd.fl_leg[]->kp;					  ///< Kp of left front leg
robot_joint_cmd.fl_leg[]->position;				  ///< Position of left front leg
robot_joint_cmd.fl_leg[]->torque;				  ///< Torue of left front leg
robot_joint_cmd.fl_leg[]->velocity;				  ///< Velocity of left front leg
```

机器人站立的简单demo：  
1.将机器人腿收起来，为站立做准备；  
2.记录下当前时间与关节数据；  
3.机器人起立。

```cpp
MotionExample robot_set_up_demo;                      		  ///< Demo for testing

/// @brief Spend 1 sec drawing the robot's legs in and preparing to stand
/// @param cmd Send control command
/// @param time Current timestamp
/// @param data_state Real-time status data of robot
robot_set_up_demo.PreStandUp(robot_joint_cmd,now_time,*robot_data);	

/// @brief Only the current time and angle are recorded
/// @param data Current joint data
/// @param time Current timestamp
robot_set_up_demo.GetInitData(robot_data->motor_state,now_time);	

/// @brief Spend 1.5 secs standing up
/// @param cmd Send control command
/// @param time Current timestamp
/// @param data_state Real-time status data of robot
robot_set_up_demo.StandUp(robot_joint_cmd,now_time,*robot_data);
```


&nbsp;
## 6 常见问题与注意事项
### 问题一

**问：** 在自己的开发主机运行MotionSDK时，如何判断SDK是否与机器狗正常通讯？

**答：**

SDK采用UDP与机器狗进行通讯。

针对数据上报，可以在SDK里打印关节数据或陀螺仪数据等信息，以此判断是否收到机器狗上报的SDK数据。或者观察SDK运行时，是否打印connection refused，以此判断是否收到机器狗上报的SDK数据。

针对指令下发，如果SDK运行后，机器狗做出回零动作，则证明SDK能成功下发指令到机器狗本体。

### 问题二

**问：** 如果SDK没收到机器狗上报数据，如何解决？

**答：**

首先检查开发主机是否与机器狗主机能处于同一网段下，如果是在机器狗上运行SDK，此步骤可跳过。

开发者先连接机器狗的WiFi网络，然后在自己的开发主机上ping 192.168.1.120，ping通之后ssh连接到机器狗运动主机内，在运动主机内ping 192.168.1.xxx，xxx为开发者开发主机的静态ip。

如果上述步骤失败，可能需要开发者手动设置自己开发主机的ip地址。

如果开发者的开发环境为虚拟机，建议把虚拟机网络连接方式改为桥接并手动设置虚拟机ip地址后重启虚拟机。

其次检查是否按照教程里的"远程连接"部分正确设置机器狗上的配置文件。

如果仍收不到机器狗上报数据，可在机器狗运动主机上运行`sudo tcpdump -x port 43897`，等待2分钟，观察机器狗是否有原始数据上报。如果没有，输入top命令查看机器狗本体控制程序进程jy_exe是否正常运行，若jy_exe没有正常运行，参照以下指令重启运动程序：

```bash
 cd /home/firefly/jy_exe
 sudo ./stop.sh
 sudo ./restart.sh
```



### 问题三

**问：** 下发控制指令不生效，机器人没反应？

**答：**

在Lite3_MotionSDK的main.cpp中，倒数几行有一行被注释的指令下发代码：

```c++
//send_cmd->SendCmd(robot_joint_cmd); 
```

此为SDK下发指令的调用，为了确保SDK的安全使用，这行**下发指令默认是注释掉的，机器狗默认只会回零不会起立。**

取消注释前，请开发者务必确认上述问题一、二，确保SDK与机器狗正常通讯，同时确保自己的下发控制指令正确，否则机器狗执行控制指令时可能会产生危险！



### 问题四

**问：** 控制指令是如何生效的？

**答：**

提供goal_pos,goal_vel,kp,kd,t_ff，共5个关节控制参数接口，控制接口全为低速端，也就是**关节端**，最终关节目标力为
$$
T=kp*(pos_{goal} - pos_{real})+kd*(vel_{goal} - vel_{real})+t_{ff}
$$

驱动器端会将最终的关节目标力转化成期望电流，并以20KHz的频率进行闭环控制。

**使用举例：**

当做纯位控即位置控制时，电机的输出轴将会稳定在一个固定的位置。例如，如果我们希望电机输出端固定在3.14弧度的位置，下发数据格式示例：
$$
pos_{goal}=3.14, vel_{goal}=0, kp=30, kd=0, t_{ff} = 0；
$$
当做速度控制时，下发数据格式示例：
$$
pos_{goal}=0, vel_{goal}=5, kp=0, kd=1, t_{ff} = 0；
$$
当做阻尼控制时，下发数据格式示例
$$
pos_{goal}=0, vel_{goal}=0, kp=0, kd=1, t_{ff} = 0；
$$
当做力矩控制时，下发数据格式示例
$$
pos_{goal}=0, vel_{goal}=0, kp=0, kd=0, t_{ff} = 3；
$$
当做零力矩控制时，下发数据格式示例
$$
pos_{goal}=0, vel_{goal}=0, kp=0, kd=0, t_{ff} = 0；
$$
当做混合控制时，下发数据格式示例：
$$
pos_{goal}=3.14, vel_{goal}=0, kp=30, kd=1, t_{ff} = 1；
$$



### 问题五

**问：** SDK的控制逻辑是怎样的？

**答：**

当SDK有指令下发时，底层控制器会优先执行SDK的控制指令，并把指令分发给机器狗12个关节。当SDK没有指令下发时，经过1s的超时判断后，底层控制器会拿回控制权，进入阻尼保护模式一段时间后，清空关节指令。控制流程图可以参考下图：

![image-20231023110612378](C:\Users\ysc16\AppData\Roaming\Typora\typora-user-images\image-20231023110612378.png)



### 其他注意事项

1. Lite3运动主机是ARM架构的，如果开发者想在运动主机上运行自己的程序，需要注意。
2. WiFi通讯受网络环境干扰产生的通讯延迟波动，可能对控制频率在500Hz以上的控制器有一定的影响。




