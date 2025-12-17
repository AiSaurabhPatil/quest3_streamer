#!/bin/bash
# Force clean the environment file
rm -f /home/saurabh/Development/meta_openxr/environment.usd

# Run setup
./python.sh setup_stage.py

echo "Environment regenerated. Please open in Isaac Sim, check /World/Franka/panda_hand/gripper_camera, and Save."
