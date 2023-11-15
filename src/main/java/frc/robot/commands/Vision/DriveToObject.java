package frc.robot.commands.Vision;

//import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.Intake.ArmIntakeInCmd;
import frc.robot.subsystems.Secondary.ArmIntakeSubsystem;
//import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java
 */
public class DriveToObject extends CommandBase
{

  private final SwerveSubsystem drivebase;
  private final PIDController   controller;
  //private final String visionObject;
  private double visionObject;
  ///private final ArmIntakeSubsystem ArmIntakeSubsystem = new ArmIntakeSubsystem();

  public DriveToObject(SwerveSubsystem drivebase, double visionObject)
  {
    this.visionObject = visionObject;
    this.drivebase = drivebase;
    controller = new PIDController(.50, 0.0, 0.0);
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
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); // Use the LED Mode set in the current pipeline
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); // Set the Limelight to the driver camera mode
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(visionObject); // Set the Limelight pipeline
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute(){
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
      RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kLeftRumble, 1);
      Double TX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      Double TY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      SmartDashboard.putString("Limelight TV", "True");
      SmartDashboard.putNumber("Limelight TX", TX);
      SmartDashboard.putNumber("Limelight TY", TY);
      Double translationValX = controller.calculate(TX, 0);
      Double translationValY = controller.calculate(TY, 0);
      SmartDashboard.putNumber("TranslationX", translationValX);
      SmartDashboard.putNumber("TranslationY", translationValY);
      
      //q: multiply translationValX by the value of the right trigger on the xbox controller? 
      if (visionObject == 0) {
        drivebase.drive(new Translation2d(translationValY * RobotContainer.driverXbox.getLeftTriggerAxis(),
                                          translationValX * RobotContainer.driverXbox.getLeftTriggerAxis()),
                                          0, true, false);
        while(RobotContainer.driverXbox.getLeftTriggerAxis() > 0) {
          //ArmIntakeInCmd(ArmIntakeSubsystem);
        }
      }
      if (visionObject == 1) {
        drivebase.drive(new Translation2d(translationValY * RobotContainer.driverXbox.getRightTriggerAxis(),
                                          translationValX * RobotContainer.driverXbox.getRightTriggerAxis()),
                                          0, true, false);
          while(RobotContainer.driverXbox.getRightTriggerAxis() > 0) {
            //new ArmIntakeInCmd(armIntakeSubsystem);
          }
        }
      }
        else {
          SmartDashboard.putString("Limelight TV", "False");
          RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kLeftRumble, 0);
          RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kRightRumble, 0);
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
    // Turn off Limelight LED and set camera mode);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); // Turn off the Limelight LEDs
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); // Set the Limelight to the driver camera mode
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); // Set the Limelight to the cone pipeline
    
  }
}
