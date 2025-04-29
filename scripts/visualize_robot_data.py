import numpy as np
import matplotlib.pyplot as plt
import re
from collections import defaultdict

class RobotDataPlotter:
    def __init__(self):
        self.data = defaultdict(list)
        self.ticks = []
        
    def parse_number(self, line):
        """Extract the first number from a line."""
        matches = re.findall(r'[-+]?\d*\.\d+|\d+', line)
        return float(matches[0]) if matches else None

    def parse_file(self, filename):
        current_section = None
        current_leg = None
        current_joint = None
        
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                
                # Skip empty lines
                if not line:
                    continue
                    
                # Track tick
                if line.startswith('Tick:'):
                    self.ticks.append(int(line.split(':')[1]))
                    continue
                
                # Track main sections
                if line == 'IMU Data:':
                    current_section = 'imu'
                    continue
                elif line == 'Joint Data:':
                    current_section = 'joint'
                    continue
                elif line == 'Contact Force Data:':
                    current_section = 'force'
                    continue
                elif line == 'Robot Command:':
                    current_section = 'command'
                    continue
                
                # Track leg sections
                if any(leg in line for leg in ['Front Left Leg:', 'Front Right Leg:', 'Hind Left Leg:', 'Hind Right Leg:']):
                    if 'Front Left' in line:
                        current_leg = 'fl'
                    elif 'Front Right' in line:
                        current_leg = 'fr'
                    elif 'Hind Left' in line:
                        current_leg = 'hl'
                    elif 'Hind Right' in line:
                        current_leg = 'hr'
                    continue
                
                # Parse IMU data
                if current_section == 'imu':
                    if 'Roll Angle:' in line:
                        self.data['imu_roll'].append(self.parse_number(line))
                    elif 'Pitch Angle:' in line:
                        self.data['imu_pitch'].append(self.parse_number(line))
                    elif 'Yaw Angle:' in line:
                        self.data['imu_yaw'].append(self.parse_number(line))
                    elif 'Roll Angular Velocity:' in line:
                        self.data['imu_roll_vel'].append(self.parse_number(line))
                    elif 'Pitch Angular Velocity:' in line:
                        self.data['imu_pitch_vel'].append(self.parse_number(line))
                    elif 'Yaw Angular Velocity:' in line:
                        self.data['imu_yaw_vel'].append(self.parse_number(line))
                    elif 'Acceleration X:' in line:
                        self.data['imu_acc_x'].append(self.parse_number(line))
                    elif 'Acceleration Y:' in line:
                        self.data['imu_acc_y'].append(self.parse_number(line))
                    elif 'Acceleration Z:' in line:
                        self.data['imu_acc_z'].append(self.parse_number(line))
                
                # Parse joint data
                elif current_section == 'joint' and current_leg:
                    if 'Joint 1:' in line:
                        current_joint = 1
                    elif 'Joint 2:' in line:
                        current_joint = 2
                    elif 'Joint 3:' in line:
                        current_joint = 3
                    elif current_joint:
                        if 'Position:' in line:
                            self.data[f'{current_leg}_j{current_joint}_pos'].append(self.parse_number(line))
                        elif 'Velocity:' in line:
                            self.data[f'{current_leg}_j{current_joint}_vel'].append(self.parse_number(line))
                        elif 'Torque:' in line:
                            self.data[f'{current_leg}_j{current_joint}_torque'].append(self.parse_number(line))
                        elif 'Temperature:' in line:
                            self.data[f'{current_leg}_j{current_joint}_temp'].append(self.parse_number(line))
                
                # Parse contact forces
                elif current_section == 'force' and current_leg:
                    if 'Forces (x, y, z):' in line:
                        forces = re.findall(r'[-+]?\d*\.\d+|\d+', line)
                        if len(forces) >= 3:
                            self.data[f'{current_leg}_force_x'].append(float(forces[0]))
                            self.data[f'{current_leg}_force_y'].append(float(forces[1]))
                            self.data[f'{current_leg}_force_z'].append(float(forces[2]))

    def plot_all(self):
        # Convert ticks to numpy array for plotting
        ticks = np.array(self.ticks)
        
        # Plot IMU data
        self.plot_imu_data(ticks)
        
        # Plot joint data for each leg
        self.plot_joint_data(ticks)
        
        # Plot contact forces
        # self.plot_contact_forces(ticks)
        
        plt.show()

    def plot_imu_data(self, ticks):
        # IMU angles
        plt.figure(figsize=(15, 10))
        
        # Angles
        plt.subplot(3, 1, 1)
        plt.plot(ticks, self.data['imu_roll'], label='Roll')
        plt.plot(ticks, self.data['imu_pitch'], label='Pitch')
        plt.plot(ticks, self.data['imu_yaw'], label='Yaw')
        plt.title('IMU Angles')
        plt.xlabel('Tick')
        plt.ylabel('Angle (degrees)')
        plt.legend()
        plt.grid(True)
        
        # Angular velocities
        plt.subplot(3, 1, 2)
        plt.plot(ticks, self.data['imu_roll_vel'], label='Roll Velocity')
        plt.plot(ticks, self.data['imu_pitch_vel'], label='Pitch Velocity')
        plt.plot(ticks, self.data['imu_yaw_vel'], label='Yaw Velocity')
        plt.title('IMU Angular Velocities')
        plt.xlabel('Tick')
        plt.ylabel('Angular Velocity (deg/s)')
        plt.legend()
        plt.grid(True)
        
        # Accelerations
        plt.subplot(3, 1, 3)
        plt.plot(ticks, self.data['imu_acc_x'], label='X')
        plt.plot(ticks, self.data['imu_acc_y'], label='Y')
        plt.plot(ticks, self.data['imu_acc_z'], label='Z')
        plt.title('IMU Accelerations')
        plt.xlabel('Tick')
        plt.ylabel('Acceleration (m/s²)')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()

    def plot_joint_data(self, ticks):
        legs = ['fl', 'fr', 'hl', 'hr']
        leg_names = {'fl': 'Front Left', 'fr': 'Front Right', 
                    'hl': 'Hind Left', 'hr': 'Hind Right'}
        
        # Position
        plt.figure(figsize=(15, 10))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            for j in range(1, 4):
                plt.plot(ticks, self.data[f'{leg}_j{j}_pos'], label=f'Joint {j}')
            plt.title(f'{leg_names[leg]} Joint Positions')
            plt.xlabel('Tick')
            plt.ylabel('Position (degrees)')
            plt.legend()
            plt.grid(True)
        plt.tight_layout()
        
        # Velocity
        plt.figure(figsize=(15, 10))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            for j in range(1, 4):
                plt.plot(ticks, self.data[f'{leg}_j{j}_vel'], label=f'Joint {j}')
            plt.title(f'{leg_names[leg]} Joint Velocities')
            plt.xlabel('Tick')
            plt.ylabel('Velocity (deg/s)')
            plt.legend()
            plt.grid(True)
        plt.tight_layout()
        
        # Torque
        plt.figure(figsize=(15, 10))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            for j in range(1, 4):
                plt.plot(ticks, self.data[f'{leg}_j{j}_torque'], label=f'Joint {j}')
            plt.title(f'{leg_names[leg]} Joint Torques')
            plt.xlabel('Tick')
            plt.ylabel('Torque (Nm)')
            plt.legend()
            plt.grid(True)
        plt.tight_layout()

    def plot_contact_forces(self, ticks):
        legs = ['fl', 'fr', 'hl', 'hr']
        leg_names = {'fl': 'Front Left', 'fr': 'Front Right', 
                    'hl': 'Hind Left', 'hr': 'Hind Right'}
        
        plt.figure(figsize=(15, 10))
        for i, leg in enumerate(legs):
            plt.subplot(2, 2, i+1)
            plt.plot(ticks, self.data[f'{leg}_force_x'], label='X')
            plt.plot(ticks, self.data[f'{leg}_force_y'], label='Y')
            plt.plot(ticks, self.data[f'{leg}_force_z'], label='Z')
            plt.title(f'{leg_names[leg]} Contact Forces')
            plt.xlabel('Tick')
            plt.ylabel('Force')
            plt.legend()
            plt.grid(True)
        plt.tight_layout()

def main():
    plotter = RobotDataPlotter()
    plotter.parse_file('robot_data.txt')
    plotter.plot_all()

if __name__ == "__main__":
    main()