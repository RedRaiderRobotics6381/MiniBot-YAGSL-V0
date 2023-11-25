// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.Rotate;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;

public class ArmRotateCmd extends CommandBase {
  private double ArmRotateSetpoint;
  private ArmRotateSubsystem armRotateSubsystem;



  public ArmRotateCmd(ArmRotateSubsystem armRotateSubsystem, double ArmRotateSetpoint) { 
    this.armRotateSubsystem = armRotateSubsystem;
    this.ArmRotateSetpoint = ArmRotateSetpoint;
    addRequirements(armRotateSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armRotateSubsystem.m_armPIDController.setReference(ArmRotateSetpoint, CANSparkMax.ControlType.kSmartMotion);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
