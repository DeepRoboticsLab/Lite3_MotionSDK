# 绝影Lite3运动控制SDK

[English](https://github.com/DeepRoboticsLab/Lite3_MotionSDK#readme)

### SDK下载及解压

- 下载 **Lite3_MotionSDK**，并解压。

### 远程连接

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

### 编译开发

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

### 示例代码

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
   ```

SDK接收到的关节数据将保存在`robot_data`中：

   ```cpp
	RobotData *robot_data = &robot_data_recv->GetState(); 		  ///< Saving joint data to the robot_data
	///< Left front leg：fl_leg[3], the sequence is FL_HipX, FL_HipY, FL_Knee
	///< Right front leg：fr_leg[3], the sequence is FR_HipX, FR_HipY, FR_Knee
	///< Left hind leg：hl_leg[3], the sequence is HL_HipX, HL_HipY, HL_Knee
	///< Right hind leg：hr_leg[3], the sequence is HR_HipX, HR_HipY, HR_Knee
	///< All joints：leg_force[12]/joint_data[12], the sequence is FL_HipX, FL_HipY, FL_Knee, FR_HipX, FR_HipY, FR_Knee, HL_HipX, HL_HipY, HL_Knee, HR_HipX, HR_HipY, HR_Knee
	
	robot_data->contact_force->fl_leg[]				  ///< Contact force on left front foot in X-axis, Y-axis and Z-axis
	robot_data->contact_force->fr_leg[]				  ///< Contact force on right front foot in X-axis, Y-axis and Z-axis
	robot_data->contact_force->hl_leg[]				  ///< Contact force on left hind foot in X-axis, Y-axis and Z-axis
	robot_data->contact_force->hr_leg[]				  ///< Contact force on right hind foot in X-axis, Y-axis and Z-axis
	robot_data->contact_force->leg_force[]			          ///< Contact force on all feet
	
	robot_data->tick						  ///< Cycle of operation
	
	robot_data->imu							  ///< IMU data	
	robot_data->imu->acc_x						  ///< Acceleration on X-axis
	robot_data->imu->acc_y						  ///< Acceleration on Y-axis
	robot_data->imu->acc_z						  ///< Acceleration on Z-axis
	robot_data->imu->angle_pitch					  ///< Pitch angle
	robot_data->imu->angle_roll					  ///< Roll angle
	robot_data->imu->angle_yaw					  ///< Yaw angle
	robot_data->imu->angular_velocit_ptich			  	  ///< Pitch angular velocity
	robot_data->imu->angular_velocit_roll			  	  ///< Roll angular velocity
	robot_data->imu->angular_velocit_yaw		   	 	  ///< Yaw angular velocity
	robot_data->imu->buffer_byte					  ///< Buffer data
	robot_data->imu->buffer_float					  ///< Buffer data
	robot_data->imu->timestamp					  ///< Time when the data is obtained

	robot_data->joint_data						  ///< Motor status
	robot_data->joint_data->fl_leg[]->position		  	  ///< Motor position of left front leg
	robot_data->joint_data->fl_leg[]->temperature	  		  ///< Motor temperature of left front leg
	robot_data->joint_data->fl_leg[]->torque		 	  ///< Motor torque of left front leg 
	robot_data->joint_data->fl_leg[]->velocity		 	  ///< Motor velocity of left front leg
	robot_data->joint_data->joint_data[]              		  ///< All joint data
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

机器人站立的简单demo：1.将机器人腿收起来，为站立做准备；2.记录下当前时间与关节数据；3.机器人起立。

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




