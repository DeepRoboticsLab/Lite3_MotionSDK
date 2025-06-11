import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime

class CSVDataPlotter:
    def __init__(self, csv_file):
        """
        Initialize the plotter with a CSV file.
        
        Args:
            csv_file (str): Path to the CSV file containing robot data
        """
        self.df = pd.read_csv(csv_file)
        # Convert timestamp to datetime
        self.df['timestamp'] = pd.to_datetime(self.df['timestamp'])
        # Create a time index in seconds from the start
        self.df['time'] = (self.df['timestamp'] - self.df['timestamp'].iloc[0]).dt.total_seconds()
        
    def plot_all(self):
        """Plot all data categories."""
        # Plot IMU data
        self.plot_imu_data()
        
        # Plot joint data
        self.plot_joint_data()
        
        # Plot contact forces
        self.plot_contact_forces()
        
        # Plot actions
        self.plot_actions()
        
        plt.show()

    def plot_imu_data(self):
        """Plot IMU data (angles, angular velocities, and accelerations)."""
        plt.figure(figsize=(15, 10))
        
        # Angles
        plt.subplot(3, 1, 1)
        plt.plot(self.df['time'], self.df['imu_angle_roll'], label='Roll')
        plt.plot(self.df['time'], self.df['imu_angle_pitch'], label='Pitch')
        plt.plot(self.df['time'], self.df['imu_angle_yaw'], label='Yaw')
        plt.title('IMU Angles')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (degrees)')
        plt.legend()
        plt.grid(True)
        
        # Angular velocities
        plt.subplot(3, 1, 2)
        plt.plot(self.df['time'], self.df['imu_angular_velocity_roll'], label='Roll Velocity')
        plt.plot(self.df['time'], self.df['imu_angular_velocity_pitch'], label='Pitch Velocity')
        plt.plot(self.df['time'], self.df['imu_angular_velocity_yaw'], label='Yaw Velocity')
        plt.title('IMU Angular Velocities')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (deg/s)')
        plt.legend()
        plt.grid(True)
        
        # Accelerations
        plt.subplot(3, 1, 3)
        plt.plot(self.df['time'], self.df['imu_acc_x'], label='X')
        plt.plot(self.df['time'], self.df['imu_acc_y'], label='Y')
        plt.plot(self.df['time'], self.df['imu_acc_z'], label='Z')
        plt.title('IMU Accelerations')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()

    def plot_joint_data(self):
        """Plot joint positions, velocities, and torques for all legs."""
        legs = ['fl', 'fr', 'hl', 'hr']
        leg_names = {'fl': 'Front Left', 'fr': 'Front Right', 
                    'hl': 'Hind Left', 'hr': 'Hind Right'}
        
        # Position
        plt.figure(figsize=(15, 10))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            for j in range(1, 4):
                plt.plot(self.df['time'], self.df[f'{leg}_joint{j}_pos'], label=f'Joint {j}')
            plt.title(f'{leg_names[leg]} Joint Positions')
            plt.xlabel('Time (s)')
            plt.ylabel('Position (rad)')
            plt.legend()
            plt.grid(True)
        plt.tight_layout()
        
        # Velocity
        plt.figure(figsize=(15, 10))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            for j in range(1, 4):
                plt.plot(self.df['time'], self.df[f'{leg}_joint{j}_vel'], label=f'Joint {j}')
            plt.title(f'{leg_names[leg]} Joint Velocities')
            plt.xlabel('Time (s)')
            plt.ylabel('Velocity (rad/s)')
            plt.legend()
            plt.grid(True)
        plt.tight_layout()
        
        # Torque
        plt.figure(figsize=(15, 10))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            for j in range(1, 4):
                plt.plot(self.df['time'], self.df[f'{leg}_joint{j}_torque'], label=f'Joint {j}')
            plt.title(f'{leg_names[leg]} Joint Torques')
            plt.xlabel('Time (s)')
            plt.ylabel('Torque (Nm)')
            plt.legend()
            plt.grid(True)
        plt.tight_layout()

    def plot_contact_forces(self):
        """Plot contact forces for all legs."""
        legs = ['fl', 'fr', 'hl', 'hr']
        leg_names = {'fl': 'Front Left', 'fr': 'Front Right', 
                    'hl': 'Hind Left', 'hr': 'Hind Right'}
        
        plt.figure(figsize=(15, 10))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            plt.plot(self.df['time'], self.df[f'{leg}_force_x'], label='X')
            plt.plot(self.df['time'], self.df[f'{leg}_force_y'], label='Y')
            plt.plot(self.df['time'], self.df[f'{leg}_force_z'], label='Z')
            plt.title(f'{leg_names[leg]} Contact Forces')
            plt.xlabel('Time (s)')
            plt.ylabel('Force (N)')
            plt.legend()
            plt.grid(True)
        plt.tight_layout()

    def plot_actions(self):
        """Plot the robot's actions."""
        action_header = [
            # Actions
            "fl_hip_action", "fl_thigh_action", "fl_calf_action",
            "fr_hip_action", "fr_thigh_action", "fr_calf_action",
            "hl_hip_action", "hl_thigh_action", "hl_calf_action",
            "hr_hip_action", "hr_thigh_action", "hr_calf_action"
        ]
        plt.figure(figsize=(15, 10))
        for i in range(12):
            plt.plot(self.df['time'], self.df[action_header[i]], label=action_header[i])
        plt.title('Robot Actions')
        plt.xlabel('Time (s)')
        plt.ylabel('Action Value')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

    def plot_all_in_one(self):
        """Plot all data categories in a single figure."""
        # First figure with IMU data
        plt.figure(figsize=(20, 8))
        plt.plot(self.df['time'], self.df['imu_angle_roll'], label='Roll')
        plt.plot(self.df['time'], self.df['imu_angle_pitch'], label='Pitch')
        plt.plot(self.df['time'], self.df['imu_angle_yaw'], label='Yaw')
        plt.title('IMU Angles', pad=20)
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (degrees)')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()
        
        legs = ['fl', 'fr', 'hl', 'hr']
        leg_names = {'fl': 'Front Left', 'fr': 'Front Right', 
                    'hl': 'Hind Left', 'hr': 'Hind Right'}
        
        # Joint positions figure
        plt.figure(figsize=(20, 12))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            for j in range(1, 4):
                plt.plot(self.df['time'], self.df[f'{leg}_joint{j}_pos'], 
                        label=f'Joint {j}')
            plt.title(f'{leg_names[leg]} Joint Positions', pad=20)
            plt.xlabel('Time (s)')
            plt.ylabel('Position (rad)')
            plt.legend()
            plt.grid(True)
        plt.tight_layout()
        plt.show()
        
        # Joint velocities figure
        plt.figure(figsize=(20, 12))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            for j in range(1, 4):
                plt.plot(self.df['time'], self.df[f'{leg}_joint{j}_vel'], 
                        label=f'Joint {j}')
            plt.title(f'{leg_names[leg]} Joint Velocities', pad=20)
            plt.xlabel('Time (s)')
            plt.ylabel('Velocity (rad/s)')
            plt.legend()
            plt.grid(True)
        plt.tight_layout()
        plt.show()
        
        # Actions figure
        plt.figure(figsize=(20, 12))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            actions = [f'{leg}_hip_action', f'{leg}_thigh_action', f'{leg}_calf_action']
            for action in actions:
                plt.plot(self.df['time'], self.df[action], label=action)
            plt.title(f'{leg_names[leg]} Actions', pad=20)
            plt.xlabel('Time (s)')
            plt.ylabel('Action Value')
            plt.legend()
            plt.grid(True)
            plt.ylim(-1.1, 1.1)
        plt.tight_layout()
        plt.show()  

        # Command figure: linear and angular velocities
        plt.figure(figsize=(20, 8))

        # Subplot 1: Linear velocity commands
        plt.subplot(2, 1, 1)
        for i, label in enumerate(["command_lin_vel_x", "command_lin_vel_y", "command_lin_vel_z"]):
            if label in self.df.columns:
                plt.plot(self.df['time'], self.df[label], label=label)
        plt.title('Command Linear Velocities', pad=20)
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity Command')
        plt.legend()
        plt.grid(True)

        # Subplot 2: Angular velocity command
        plt.subplot(2, 1, 2)
        if "command_ang_vel" in self.df.columns:
            plt.plot(self.df['time'], self.df["command_ang_vel"], label="command_ang_vel")
        plt.title('Command Angular Velocity', pad=20)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity Command')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.show()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Plot robot data from CSV file')
    parser.add_argument('csv_file', type=str, help='Path to the CSV file containing robot data')
    args = parser.parse_args()
    
    plotter = CSVDataPlotter(args.csv_file)
    plotter.plot_all_in_one()

if __name__ == "__main__":
    main()