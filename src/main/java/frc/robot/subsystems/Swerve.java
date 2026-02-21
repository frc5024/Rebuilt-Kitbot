// Based off of https://github.com/HoltRobotics/BaseFalconSwerve/ but using a navX instead of a pigeon
/* PID: 
P: Proportional; "I want to go over there please waiter, also at ____ speed since i enjoy the view" 
I: Intergal; "Im alowed to move this much?! Are you mad?!"
D: Derivertive; Slows you down the closer you are to the target
*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    double translationVal = 0;
    double strafeVal = 0;
    double rotationVal = 0;

    boolean lockRotation = false;
    boolean lockStrafe = false;
    boolean lockTranslation = false;

    // Singleton
    private static Swerve mInstance;

    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }


    private Swerve() {
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.zeroYaw();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        
        RobotConfig config = null;
            try {
            config = RobotConfig.fromGUISettings();} 
            catch (Exception e) 
            {e.printStackTrace();}
            // Handle exception as needed
            

        // Required to access the robot in PathPlanner
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds),/* Method that will drive the robot given ROBOT RELATIVE 
                                                                    ChassisSpeeds. Also optionally outputs individual module 
                                                                    feedforwards */
            new PPHolonomicDriveController(/* PPHolonomicController is 
                                              the built in path following controller for holonomic drive trains */
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants 
            ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    // Controller called values - only call when Robot is not locked
    public void controllerTranslationalVal(double translationOutput) {
        if (!lockTranslation) {
            translationVal = translationOutput;
        }
    }

    public void controllerRotationVal(double rotationOutput) {
        if (!lockRotation) {
            rotationVal = rotationOutput;
        }
    }

    public void controllerStrafeVal(double strafeOutput) {
        if (!lockStrafe) {
            strafeVal = strafeOutput;
        }
    }

    /*
     * Set desired ChassisSpeed, then call to apply this ChassisSpeed
     */
    public void drive(boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds = null;

        double xVelocity = translationVal * Constants.Swerve.maxSpeed;
        double yVelocity = strafeVal * Constants.Swerve.maxSpeed;
        double rVelocity = rotationVal * Constants.Swerve.maxAngularVelocity;

        chassisSpeeds = new ChassisSpeeds(
            xVelocity,
            yVelocity,
            rVelocity
        );
        drive(chassisSpeeds, isOpenLoop);
    }

    /**
     * Apply ChassisSpeed from above method: drive(boolean isOpenLoop)
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        // Apply speed to each SwerveModule
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        if (pose == null)
            pose = Pose2d.kZero;

        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void resetSwerve() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(),heading));
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getGyroYaw() {
        double angle = (gyro.getAngle()) % 360.0;
        if (angle > 180) {
            angle = angle - 360;
        }
        return Rotation2d.fromDegrees(-angle);
    }

    public Rotation2d getGyroYaw360() {
        double angle = (gyro.getAngle()) % 360.0;
        return Rotation2d.fromDegrees(-angle);
    }

    public void zeroHeadingWithOffset(double degOffset) {
        swerveOdometry.resetPosition(getGyroYawWithOffset(degOffset), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYawWithOffset(double degOffset) {
        // negative to fix field relative
        return Rotation2d.fromDegrees((-gyro.getYaw()) - degOffset);

    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            mod.update();
        }
    }
}