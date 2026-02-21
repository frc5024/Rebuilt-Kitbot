// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCmd extends Command {

  //creating a new hybrid command
  private final Intake m_intake;
  double m_speed = 0.0;
  
  //Creates a new intake command (and a speed variable) 
  public IntakeCmd(Intake subsystem, double speed) {
    m_intake = subsystem;
    m_speed = speed;

    addRequirements(subsystem);
  }

  // Start when the command is called.
  @Override
  public void initialize() {
    m_intake.setMotorPower(m_speed);
  }


  // Start every time the caller runs while the command is called.
  @Override
  public void execute() {
  }
    
// Called once the command ends or is interrupted.
// Stop motor
  @Override
  public void end(boolean interrupted) {
    m_intake.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
