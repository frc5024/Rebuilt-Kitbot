 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeCmd;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Intake mInstance; 
  private int motor1ID = 1;

  private SparkMax IntakeMotor1 = new SparkMax(motor1ID, MotorType.kBrushless);

  public static final Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance; 
  }

    
  public void setMotorPower(double speed) {
    IntakeMotor1.set(speed);
  }

  public Command getIntake() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return new IntakeCmd(this);
  }    
}
