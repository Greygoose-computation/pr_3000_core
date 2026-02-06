# Peer Perception Workspace – Simulation + Perception Runbook

## Workspace Overview./sta
- Workspace root:
  ~/testing/ananth/peer_perception
- This is NOT a standard ROS workspace
- Runtime is controlled via:
  - NVIDIA Docker
  - ROS 2 using CycloneDDS middleware
  - Menu-driven controller script: startScript.sh
- ROS nodes run inside Docker containers, not on the host

## Entry Point
Always start from the workspace root:
cd ~/testing/ananth/peer_perception
./startScript.sh

## Main Menu
1. Running Robot Stack
2. Code Build
3. Advance Options

Correct selection:
- Select 1 (Running Robot Stack)

## Robot Stack Submenu
1. Launching empty nvidia docker container
2. Pull/Build nvidia docker
3. Launching zed2i and recording bag
4. Isaac simulator options
5. Robot Stack in simulation
6. Applying Robot Calibration
7. Vision Advanced Options

## Correct Startup Order (IMPORTANT)

Step 1 – Start Simulation
- Select option 1: Running Robot Stack

Step 2 – Start Perception in Simulation
- After simulation is running, select option 5: Robot Stack in simulation
- Launches the peer_perception Docker container
- Starts the perception stack (vision, detection, tracking)
- Subscribes to simulated sensor topics

## Expected Warnings (Safe in Simulation)
The following warnings are expected on a non-robot machine and can be ignored:
- Cannot find device "can0" (physical CAN bus not present)
- chmod: /dev/devices/imu missing (physical IMU not present)
- Cyclone DDS interface set to 'lo'

## What NOT to Do
- Do NOT manually run ros2 launch on the host
- Do NOT source install/setup.bash on the host
- Do NOT start perception before simulation
- Do NOT expect hardware devices (CAN, IMU, ZED) in simulation mode

## Quick Start Summary (TL;DR)
1. cd ~/testing/ananth/peer_perception
2. ./startScript.sh
3. Select 1 – Running Robot Stack
4. Select 4 – Isaac simulator options
5. Wait for simulation to fully start
6. Select 5 – Robot Stack in simulation

# Layer-by-layer questions for company team (for solver + simulator testing)

## Layer 0 — What exactly does startScript launch?
1. For “Robot Stack in simulation” (option 5), what containers and processes are started? - (repo-simulation-(world, gazebo),peer perception(actual firmware) )  
2. Which launch files are effectively executed (package + launch filename)? - (peer robot controller pkg,  )
3. What is the expected startup order between:
   - Isaac sim (option 4)
   - Robot stack in sim (option 5)
   - RViz (if any)
4. What is the “healthy” baseline after startup?
   - expected nodes count, expected topics count, expected services
5. Is there a one-command way to stop/cleanup everything (proper shutdown)?

## Layer 1 — ROS graph basics (nodes, topics, services)
6. After option 5, how many ROS 2 nodes should be running (rough expected range)?
7. Which nodes are “core” for sim navigation testing (must be alive)?
8. Which nodes can be ignored for solver testing (perception, docking, etc.)?
9. Is teleop:
   - a node that runs continuously? - yes 
   - or just a package with a service interface that gets activated? - 
10. Which node provides `/robot1/teleop/command` when the system is correct? 

## Layer 2 — TF, frames, and RViz configuration
11. What is the correct RViz config to use (file path)? 
12. What is the fixed frame in RViz (map / odom / base_link)? 
13. What TF tree is expected in simulation?
   - map -> odom -> base_link, etc.
14. If RViz shows “No transform”, what is the standard fix?

## Layer 3 — Maps, localization, and planning frame
15. Where is the map loaded from (yaml/pgm)? Which package/config?
16. Is localization AMCL, slam_toolbox, or Isaac-provided ground truth?
17. What topics provide:
   - robot pose estimate (e.g., /amcl_pose)
   - odometry (e.g., /odom)
   - TF (from /tf)
18. What is the global frame used by navigation/planning (map vs odom)?

## Layer 4 — Setting destinations (goals) in RViz
19. Are we using Nav2 “Goal Pose” in RViz, or a custom goal interface?
20. What exact RViz tool should be used:
   - “Nav2 Goal”
   - “2D Nav Goal”
   - something custom?
21. What topic/action receives the goal?
   - which action name? (e.g., /navigate_to_pose) - 
22. Any required header frame_id for goals (map vs odom)?
23. Is there a “click-to-goal” workflow documented for simulation?

## Layer 5 — Command path to motion (what moves the robot)
24. What is the final command topic into the base/sim controller?
   - /cmd_vel or /robot1/cmd_vel or something else?
25. Is there a cmd_vel mux/arbiter? How do we ensure Nav is in control (not teleop)?
26. What safety gates can block motion in sim (estop, watchdog, mode)?
27. If goals are accepted but robot doesn’t move, what are the top 3 checks?

## Layer 6 — Where should your solver integrate?
28. Should your solver:
   - replace Nav2 planner?
   - publish a path to Nav2 controller?
   - publish cmd_vel directly?
   - run as a standalone node and command via service/action?
29. What inputs should your solver consume?
   - map, costmap, robot pose, goal pose, obstacles
30. What outputs are expected from your solver?
   - Path, trajectory, cmd_vel, or action response
31. What message types should you use (Path, PoseStamped, Twist, etc.)?

## Layer 7 — Validation & test procedure (simulation)
32. What is the canonical simulator test scenario?
   - which map/world, start pose, goal examples
33. How do you log/record runs (rosbag commands, what topics)?
34. What metrics are used to judge success?
   - time-to-goal, collisions, path length, smoothness
35. Any known simulator quirks (topics delayed, TF jumps, localization resets)?

## Layer 8 — Practical dev workflow
36. Where is the “right place” to modify code (in src vs mounted volume vs image rebuild)?
37. What is the recommended loop for changes:
   - edit → colcon build → restart container/stack?
38. How do we run in “debug mode” (verbose logs, rqt, rviz overlays)?





