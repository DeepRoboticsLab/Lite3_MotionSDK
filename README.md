# Jueying Lite3 Motion SDK

[简体中文](https://github.com/DeepRoboticsLab/Lite3_MotionSDK/blob/main/README_ZH.md)

### SDK Download and Unzip

- Download ***Lite3_MotionSDK*** and unzip.

### Remote Connection

Users can connect to the motion host remotely via SSH.

- Connect the development host to the robot's WiFi.

- Open the SSH connection software on the development host and enter `ssh firefly@192.168.1.120`, with the password `firefly`, to connect to the motion host remotely.

- Enter the following code to open the network config file:
	```Bash
	cd /home/firefly/jy_exe/conf/
	vim network.toml
	```
	
- The config file ***network.toml*** reads as follows:
	```toml
	ip = '192.168.1.102'
	target_port = 43897
	local_port = 43893
	~
	```
	
- Modify the IP address in the first line of the config file so that **MotionSDK** can receive motion data from the robot.
	- If **MotionSDK** runs on the motion host of robot, please reset the IP address to `192.168.1.120`;
	- If **MotionSDK** runs on your development host, please reset it to the static IP address of your development host:  `192.168.1.xxx`.
	
- Restart the motion program for these changes to take effect:

	```bash
	cd /home/firefly/jy_exe
	sudo ./stop.sh
	sudo ./restart.sh
	```

### Compile and Develop

- Users can navigate to the directory that contains ***CMakeLists.txt*** and create a ***build*** directory.

	```bash
	cd xxxxxxxx     # cd <path to where you want to create build directory>
	mkdir build
	```
	
	> Caution: Users can create ***build*** directory in any location, and make sure that when compiling, the path provided to `cmake` is the path to where ***CMakeLists.txt*** is.
	
- Navigate to the ***build*** directory and then compile.

	- Compile for x86 hosts:

		```bash
		cd build
		cmake .. -DBUILD_PLATFORM=x86     # cmake <path to where the CMakeLists.txt is>
		make -j
		```
		
	- Compile for ARM hosts:

		```bash
		cd build
		cmake .. -DBUILD_PLATFORM=arm     # cmake <path to where the CMakeLists.txt is>
		make -j
		```
	
- After compilation, an executable file named ***Lite_motion*** is generated in the ***build*** directory.

- Enter the following codes in Terminal to run the program:

	```bash
	./Lite_motion
	```

### Example Code

This section explains ***main.cpp***.

Timer, used to set the algorithm period and obtain the current time:

	```cpp
	DRTimer set_timer;
 	set_timer.TimeInit(int);                              		  ///< Timer initialization, input: cycle; unit: ms
 	set_timer.GetCurrentTime();                           		  ///< Obtain time for algorithm
 	set_timer.TimerInterrupt()			      		  ///< Timer interrupt flag
 	set_timer.GetIntervalTime(double);                    		  ///< Get the current time
   ```

After binding the IP and port of the robot, SDK will acquire control right and can send the joint control commands:

   ```cpp
	Sender* send_cmd = new Sender("192.168.1.120",43893); 		  ///< Create a sender thread
	send_cmd->RobotStateInit();                           		  ///< Reset all joints to zero and gain control right
	send_cmd->SetSend(RobotCmd); 			     		  ///< Send joint control command
	send_cmd->ControlGet(int);                            		  ///< Return the control right
   ```

SDK receives the joint data from the robot:

   ```cpp
	Receiver* robot_data_recv = new Receiver();           		  ///< Create a thread for receiving and parsing
	robot_data_recv->GetState(); 			      		  ///< Receive data from 12 joints 
   ```

The data SDK received will be saved into `robot_data`:

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

 Robot joint control command:

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

A simple demo that can make the robot stand:

1. Draw the robot's legs in and prepare to stand;
2. Record the current time and joint data;
3. The robot stands up.

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




