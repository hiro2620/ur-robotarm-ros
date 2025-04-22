import subprocess

#!/usr/bin/env python3


def list_controllers():
    try:
        # Execute the ROS 2 command to list controllers
        result = subprocess.run(
            ['ros2', 'control', 'list_controllers'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Check if the command was successful
        if result.returncode == 0:
            print("Available controllers:")
            print(result.stdout)
        else:
            print("Error listing controllers:")
            print(result.stderr)
    except FileNotFoundError:
        print("Error: 'ros2' command not found. Make sure ROS 2 is installed and sourced.")

if __name__ == "__main__":
    list_controllers()