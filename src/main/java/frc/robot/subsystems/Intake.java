package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeCmd;
import frc.robot.Constants;


public class Intake extends SubsystemBase {

  private static Intake mInstance; 

  private SparkMax IntakeMotor = new SparkMax(Constants.Intake.motorID, MotorType.kBrushed);

  
  public static final Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance; 
  }

    
  public void setMotorPower(double speed) {
    IntakeMotor.set(-speed);
  }

  public Command getIntake(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return new IntakeCmd(this, speed);
  }    
}
