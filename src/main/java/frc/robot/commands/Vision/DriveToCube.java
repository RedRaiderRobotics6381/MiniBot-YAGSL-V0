package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  private final SwerveSubsystem drivebase;
  private final PIDController   controller;

  public DriveToCube(SwerveSubsystem drivebase)
  {
    this.drivebase = drivebase;
    controller = new PIDController(.25, 0.0, 0.0);
    controller.setTolerance(.1);
    controller.setSetpoint(0.0);
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drivebase);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    LimelightHelpers.setPipelineIndex("", 0); // Set the Limelight to the cone pipeline
    //LimelightHelpers.setLEDMode_ForceOn("");
    //LimelightHelpers.setCameraMode_Processor("");
    //LimelightHelpers.setPipelineIndex("", 0); // Set the Limelight to the cone pipeline
    //SmartDashboard.putNumber("TX", LimelightHelpers.getTargetPose_CameraSpace("")[0]);
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute(){
    // SmartDashboard.putNumber("Limelight X", LimelightHelpers.getTX(""));
    // SmartDashboard.putNumber("Limelight Y", LimelightHelpers.getTY(""));
    Boolean HasTarget = LimelightHelpers.getTV("");
    if (HasTarget == true){
      SmartDashboard.putNumber("Limelight Target X",LimelightHelpers.getTargetPose3d_CameraSpace("").getX());
      SmartDashboard.putNumber("Limelight Target Y",LimelightHelpers.getTargetPose3d_CameraSpace("").getY());
      SmartDashboard.putNumber("Limelight TX",LimelightHelpers.getTX(""));
      SmartDashboard.putNumber("Limelight TY",LimelightHelpers.getTY(""));
      if (LimelightHelpers.getNeuralClassID("") == 0) {
        //SmartDashboard.putNumber("Limelight ID", LimelightHelpers.getNeuralClassID(""));
          //Double TX = LimelightHelpers.getTargetPose_CameraSpace("")[0];
          //Double TY = LimelightHelpers.getTargetPose_CameraSpace("")[1];
          Double TX = LimelightHelpers.getTX("");
          Double TY = LimelightHelpers.getTY("");
          //Double CX = LimelightHelpers.getCameraPose3d_RobotSpace("").getX();
          //Double CY = LimelightHelpers.getCameraPose3d_RobotSpace("").getY();
          //Double CX = LimelightHelpers.getCameraPose_TargetSpace("")[0];
          //Double CY = LimelightHelpers.getCameraPose_TargetSpace("")[1];
          Double translationValX = controller.calculate(TX, 0); 
          Double translationValY = controller.calculate(TY, 0); 
          drivebase.drive(new Translation2d(translationValX, translationValY), 0, true, false);
      }
    }
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
    //drivebase.lock();
    // Turn off Limelight LED and set camera mode
    //LimelightHelpers.setLEDMode_ForceOff("");
    //LimelightHelpers.setCameraMode_Processor("");
    //LimelightHelpers.setCameraMode_Driver("");
    
  }
}
