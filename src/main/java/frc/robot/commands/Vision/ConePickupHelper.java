// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Arm.Intake.ArmIntakeInCmd;
import frc.robot.commands.Arm.Rotate.ArmRotateToDrivePosCmd;
import frc.robot.commands.Arm.Rotate.ArmRotateToIntakePos;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Secondary.ArmIntakeSubsystem;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;
import swervelib.SwerveDrive;

public class ConePickupHelper extends CommandBase {
  public final static ArmRotateSubsystem armRotateSubsystem = new ArmRotateSubsystem();
  public final static ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  /** Creates a new ConePickupHelper. */
  private final swervelib.SwerveDrive swerveDrive;
  private final PIDController   controller;

  public ConePickupHelper() {
    swerveDrive = new SwerveDrive(null, null);
    controller = new PIDController(1.0, 0.0, 0.0);
  }

  @Override
  public void initialize() {
    // Turn on Limelight LED and set camera mode
    LimelightHelpers.setLEDMode_ForceOn(getName());
    LimelightHelpers.setCameraMode_Processor(getName());
    LimelightHelpers.setPipelineIndex(getName(), 1); // Set the Limelight to the cone pipeline
    controller.setTolerance(1);
    controller.setSetpoint(0.0);
  }

  @Override
  public void execute() {
    new ArmRotateToIntakePos(armRotateSubsystem);
    new ArmIntakeInCmd(armIntakeSubsystem);
    // Get the TX and TY values from the Limelight
    Double TX = LimelightHelpers.getTX(getName()) + .5; // Add .5 to TX to center the robot on the target
    Double TY = LimelightHelpers.getTY(getName()) - .1; // Subtract .1 from TY to center the robot on the target
    
    double translationValX = MathUtil.clamp(controller.calculate(TX, 0.25), 0, 2); // Clamp the translation values
    double translationValY = MathUtil.clamp(controller.calculate(TY, -.1), -2, 2); // Clamp the translation values

    // Set the speed and angle of each wheel on the swerve drive
    swerveDrive.drive(new Translation2d(translationValX,translationValY), 0.0, true, false);
    new ArmRotateToDrivePosCmd(armRotateSubsystem);
  }

  @Override
  public void end(boolean interrupted) {
    // Turn off Limelight LED and set camera mode
    //LimelightHelpers.setLEDMode_ForceOff(getName());
    //LimelightHelpers.setCameraMode_Processor(getName());
    //LimelightHelpers.setCameraMode_Driver(getName());
  }
}