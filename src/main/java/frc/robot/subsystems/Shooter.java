// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShooterCmd;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Shooter mInstance; 
  private int motorID = 60;
  private SparkMax ShooterMotor = new SparkMax(motorID, MotorType.kBrushed);      

  public static final Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance; 
  }

    
  public void setMotorPower(double speed) {
    ShooterMotor.set(speed);
  }

  public Command getShooter() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return new ShooterCmd(this);
  }
}
