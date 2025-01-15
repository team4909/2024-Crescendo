# 2024 Robot Code - FRC Crescendo

## Features:

### Fully AdvantageKit Replay compatible, with notable AdvantageKit implementations such as:
- AprilTag vision with PhotonVision
- Neural network game piece detection with Limelight
- High-Frequency odometry with Phoenix v6

### Full desktop simulation capability for the following:
- Drivetrain with CTRE sim API
- AprilTag vision with PhotonVision sim API
- Feeder rollers
- Shooter flywheels
- Double-jointed arm
- Intake sim with game piece pickup mocking

### Other notable features:
- Cantilevered Double-Jointed Arm implementation
- Choreo Autonoumous routines using game piece detection + april tag localization for correction
- Extensive use of WPILIB's Command Based library
- CTRE CANDle-based LEDs implementation
- Use of MotionMagic & MotionMagicVelocity for motion profiling
- Both AdvantageScope 3D and Mechanism2d implementation of double-jointed Arm visualization
- AdvantageScope game piece visualization
- SysId routines for the drivetrain, shooter, and arm
- Code written to be switchable for two robots, with multiple vendor implementations for certain subsystems, e.g. Phoenix v6 or REVLib
- Driver Convenience Features (eg. Auto Drive to Chain & Auto Aim to Speaker)
***
Thanks to 6328, for making AdvantageKit/Scope and inspiring many parts of this codebase
