package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.leds.LEDPreset;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final Swerve s_Swerve = Swerve.getInstance();

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

        configureBindings();
    }

    // Driver Controls
    private void configureBindings() {
        // Driver Controls
        // Drive

        driver.leftBumper().whileTrue(new InstantCommand(() -> s_Swerve.isSlowMode = true));
        driver.leftBumper().onFalse(new InstantCommand(() -> s_Swerve.isSlowMode = false));

        driver.x().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    }

        public Command getAutonomousCommand() {
                throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
        }

}
