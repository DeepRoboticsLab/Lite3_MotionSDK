import re
import matplotlib.pyplot as plt

# Data containers
ticks = []
positions = {'fl': [[], [], []], 'fr': [[], [], []], 'hl': [[], [], []], 'hr': [[], [], []]}
velocities = {'fl': [[], [], []], 'fr': [[], [], []], 'hl': [[], [], []], 'hr': [[], [], []]}

# Helper to map leg names
leg_map = {
    'Front Left Leg': 'fl',
    'Front Right Leg': 'fr',
    'Hind Left Leg': 'hl',
    'Hind Right Leg': 'hr'
}

with open('/home/superfhwl/repo/yaoxiang/dog_sim2real_0418/Lite3_MotionSDK/build/robot_data.txt', 'r') as f:
    lines = f.readlines()

i = 0
while i < len(lines):
    line = lines[i].strip()
    # Tick number
    if re.match(r'^\d+$', line):
        ticks.append(int(line))
        i += 1
        continue
    # Leg section
    for leg_name, leg_key in leg_map.items():
        if line.startswith(leg_name):
            for joint in range(3):
                # Position
                pos_line = lines[i+2].strip()
                pos_val = float(re.findall(r'[-+]?\d*\.\d+|\d+', pos_line)[0])
                positions[leg_key][joint].append(pos_val)
                # Velocity
                vel_line = lines[i+3].strip()
                vel_val = float(re.findall(r'[-+]?\d*\.\d+|\d+', vel_line)[0])
                velocities[leg_key][joint].append(vel_val)
                i += 6  # Skip to next joint
            break
    i += 1

# Plotting
legs = ['fl', 'fr', 'hl', 'hr']
leg_titles = ['Front Left', 'Front Right', 'Hind Left', 'Hind Right']

plt.figure(figsize=(16, 10))
for idx, leg in enumerate(legs):
    plt.subplot(2, 2, idx+1)
    for j in range(3):
        plt.plot(ticks, positions[leg][j], label=f'Joint {j+1} Pos')
    plt.title(f'{leg_titles[idx]} Leg Joint Positions')
    plt.xlabel('Tick')
    plt.ylabel('Position (degrees)')
    plt.legend()
    plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(16, 10))
for idx, leg in enumerate(legs):
    plt.subplot(2, 2, idx+1)
    for j in range(3):
        plt.plot(ticks, velocities[leg][j], label=f'Joint {j+1} Vel')
    plt.title(f'{leg_titles[idx]} Leg Joint Velocities')
    plt.xlabel('Tick')
    plt.ylabel('Velocity (degrees/s)')
    plt.legend()
    plt.grid(True)
plt.tight_layout()
plt.show()