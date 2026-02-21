//Quite litterally copied from my old codes

package frc.robot.commands;
 
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;


public class ShooterCmd extends Command {

  private final Shooter m_shooter;
  double m_speed = 0.0;


  //Creats a shooter cmd if none exist
  public ShooterCmd(Shooter subsystem, double speed) {
    m_shooter = subsystem;
    m_speed = speed;

    addRequirements(subsystem);
  }

  // Use when the command is initially called for.
  @Override
  public void initialize() {
    m_shooter.setMotorPower(m_speed);
  }

  // Start every time the caller runs while the command is called.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
