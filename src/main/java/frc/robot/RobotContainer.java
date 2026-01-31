package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Blower;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Spin_blow_motor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final Swerve s_Swerve = Swerve.getInstance();
    //private final Blower s_Blower = Blower.getInstance();
    private final Intake s_Intake = Intake.getInstance();
    private final Shooter s_Shooter = Shooter.getInstance();


    

    // Drive Controls
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;



    // Vision Variables
    boolean visionMode = false;
    public String mode;


    public RobotContainer() {

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> false // true =
                                                                                                         // robotcentric
        ));

        //s_Blower.setDefaultCommand(new Spin_blow_motor(s_Blower, () -> -operator.getRawAxis(translationAxis)));

        configureBindings();
    }

    // Driver Controls
    private void configureBindings() {
        // Driver Controls
        // Drive
        

        driver.leftBumper().whileTrue(new InstantCommand(() -> s_Swerve.isSlowMode = true));
        driver.leftBumper().onFalse(new InstantCommand(() -> s_Swerve.isSlowMode = false));

        //driver.x().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        //robot shoot
        driver.a().whileTrue(new ParallelCommandGroup(
            s_Intake.getIntake(1.0),
            s_Shooter.getShooter(-0.8)
        ));
        
        //Robot feed
        driver.b().whileTrue(new ParallelCommandGroup(
            s_Intake.getIntake(1.0),
            s_Shooter.getShooter(0.8)
        ));
    }

        public Command getAutonomousCommand() {
                throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
        }

}
