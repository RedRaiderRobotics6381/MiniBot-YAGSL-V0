package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java
 */
public class DriveToCube extends CommandBase
{

  private final SwerveSubsystem swerveSubsystem;
  private final PIDController   controller;

  public DriveToCube(SwerveSubsystem swerveSubsystem)
  {
    this.swerveSubsystem = swerveSubsystem;
    controller = new PIDController(1.0, 0.0, 0.0);
    controller.setTolerance(1);
    controller.setSetpoint(0.0);
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    LimelightHelpers.setLEDMode_ForceOn("");
    LimelightHelpers.setCameraMode_Processor("");
    LimelightHelpers.setPipelineIndex("", 1); // Set the Limelight to the cone pipeline

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {    // Get the TX and TY values from the Limelight
    Double TX = LimelightHelpers.getTX("") + .5; // Add .5 to TX to center the robot on the target
    Double TY = LimelightHelpers.getTY("") - .1; // Subtract .1 from TY to center the robot on the target
    
    Double translationValX = MathUtil.clamp(controller.calculate(TX, 0.25), 0, 2); // Clamp the translation values
    Double translationValY = MathUtil.clamp(controller.calculate(TY, -.1), -2, 2); // Clamp the translation values
    
    SmartDashboard.putNumber("TX", LimelightHelpers.getTX(""));
    SmartDashboard.putNumber("TY", LimelightHelpers.getTY(""));
    SmartDashboard.putNumber("ID", LimelightHelpers.getNeuralClassID(""));


    SmartDashboard.putBoolean("At Tolerance", controller.atSetpoint());

    //double translationVal = MathUtil.clamp(controller.calculate(swerveSubsystem.getPitch().getDegrees(), 0.0), -0.5,
    //                                       0.5);
    swerveSubsystem.drive(new Translation2d(translationValX, translationValY), 0.0, true, false);
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return controller.atSetpoint();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    swerveSubsystem.lock();
    // Turn off Limelight LED and set camera mode
    LimelightHelpers.setLEDMode_ForceOff("");
    LimelightHelpers.setCameraMode_Processor("");
    LimelightHelpers.setCameraMode_Driver("");
    
  }
}