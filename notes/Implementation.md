Source: https://docs.yagsl.com/fundamentals/swerve-drive

## The Basics
A Swerve Drive moves around by moving each wheel to a specific angle/azimuth and rotating the wheel to go in that direction. Swerve Drives are unique because they can rotate independently of their translational movement, meaning you can move in any direction while facing any direction.
<br>The rotation of your robot is referred to as the <b>heading</b>.

## What is a Swerve Drive?
A Swerve Drive typically consists of 4 Swerve Modules (which are in essence a drive motor, a angle/azimuth motor, and an absolute encoder) and a gyroscope (centered is best).
<br><br>
The motors, absolute encoders, and gyroscope can all work toegether with varying degrees of success. As a rule of thumb, if you can stick to one system do it (all REV, all CTRE, etc). 

## How does a Swerve Drive work in code?
Swerve Drvies move each module into a specific angle determined by the direction you want to go and heading you want to face. You can get these values by hand by calculating the kinematics of the robot, or use SwerveDriveKinematics[https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/SwerveDriveKinematics.html]

SwerveDriveKinematics is a helper class that converts a chassis velocity (dx, dy, and dtheta components) into individual module states (speed and angle).


