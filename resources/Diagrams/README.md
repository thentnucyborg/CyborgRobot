# Proposed system architecture for 2020

![System architecture](system_proposal_2020.png)

Changes from current system: 
- Add a command center with a ROS Commander node interfaced by external GUI and/or CLI
- Launch the whole ROS system at boot. No more script switching.
- Creation of incapsulating modules: Command, Behaviour, Controllers (need to change some ROS node names to hinder confusion)

Specification:
- Allow Commander to turn Behaviour module silent so that it does not publish to output/controller nodes.
- Commander can communicate with output/controller nodes directly to set:
  - LED visualization
  - Audio
  - Navigate to point, wander or dock
  - Manual mode: which lets us operate the Pioneer LX by attached Joystick or remotely by keyboard.
- Visual and audio nodes have built-in behaviours that can be requested by Commander or Behaviour: Idle, Navigating, Manual, Play sound, Text-to-speach, LED show etc.  