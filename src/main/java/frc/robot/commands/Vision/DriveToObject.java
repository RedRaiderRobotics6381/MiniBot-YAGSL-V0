package frc.robot.commands.Vision;

//import java.util.List;

//use this for PhotonVision
  // import org.photonvision.PhotonCamera;
  // import org.photonvision.common.hardware.VisionLEDMode;
  // import org.photonvision.targeting.PhotonTrackedTarget;
//
//use this for LimeLight
  //import edu.wpi.first.networktables.NetworkTable;
  //import edu.wpi.first.networktables.NetworkTableEntry;
  import edu.wpi.first.networktables.*;
//use this for LimelightHelpers  
  //import frc.robot.subsystems.LimelightHelpers;
//

//import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
//import frc.robot.commands.Arm.Intake.ArmIntakeInCmd;
//import frc.robot.subsystems.Secondary.ArmIntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
//import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * Auto Balance command using a simple PID controller. Created by Team 3512
 * https://github.com/frc3512/Robot-2023/blob/main/src/main/java/frc3512/robot/commands/AutoBalance.java
 */
public class DriveToObject extends CommandBase
{

  //private static final ArmIntakeSubsystem armIntakeSubsystem = null;
  private final SwerveSubsystem drivebase;
  private final PIDController   controller;
  //private final String visionObject;
  //private Double visionObject;
  private int visionObject; //vision object int was double, change back to double for limelight
  //private final ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  
  //use this for PhotonVision
    //PhotonCamera camera = new PhotonCamera("OV5647");
  //
  
  public DriveToObject(SwerveSubsystem drivebase, int visionObject)  //vision object int was double, change back to double for limelight
  {
    this.visionObject = visionObject;
    this.drivebase = drivebase;
    controller = new PIDController(1.0, 0.0, 0.0);
    controller.setTolerance(1);
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

    //use this for PhotonVision
      //camera.setLED(VisionLEDMode.kOn);
      //camera.setPipelineIndex(visionObject);
      //camera.setDriverMode(false);
    //
    //use this for LimeLight
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(visionObject);
    

    //use this for LimelightHelpers
      // LimelightHelpers.setCameraMode_Processor("");
      // LimelightHelpers.setLEDMode_ForceOn("");
      // LimelightHelpers.setPipelineIndex("",visionObject);
    //
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute(){


    //use this for PhotonVision
      // var result = camera.getLatestResult();  // Get the latest result from PhotonVision
      // boolean hasTargets = result.hasTargets(); // Check if the latest result has any targets.
      // PhotonTrackedTarget target = result.getBestTarget();
      //int targetID = result.
    //
    //use this for LimeLight

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    //use this for LimelightHelpers
      //double tx = LimelightHelpers.getTX("");
      //double ty = LimelightHelpers.getTY("");
      //double tv = LimelightHelpers.getTV("");
      //boolean hasTargets = LimelightHelpers.getTV("");
    //

    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("TX").getDouble(0); // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    //double hasTargets = NetworkTableInstance.getDefault().getTable("limelight").getEntry("TV").getDouble(0); // Whether the limelight has any valid targets (0 or 1)
    
    while (tv > 0) {
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      //double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      //RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kLeftRumble, 0.25);
      //double TX = target.getYaw();  //Uncomment this line if using PhotonVision
      //double TX = LimelightHelpers.getTX("");
      SmartDashboard.putString("Vision Target", "True");
      SmartDashboard.putNumber("Vision Target Y", tx);
      Double translationValY = controller.calculate(tx, 0);
      SmartDashboard.putNumber("TranslationY", translationValY);

      //if (visionObject == 0) {
      //    RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kLeftRumble, 0.25);
      drivebase.drive(new Translation2d(0.0, translationValY),
                      0,
                      false,
                      false);}
      // drivebase.drive(new Translation2d(0.0, translationValY * RobotContainer.driverXbox.getLeftTriggerAxis() * .25),
      //                                       0,
      //                                       false, false);}
      //}
      
      //  }

      //if (visionObject == 1) {
        //  RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kRightRumble, 0.25);
        //  drivebase.drive(new Translation2d(0.0, translationValY * RobotContainer.driverXbox.getRightTriggerAxis() * .25),
        //                                    0,
        //                                    false, false);
      //  }
      //} //else {
      //     SmartDashboard.putString("PhotoVision Target", "False");
      //     RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kLeftRumble, 0);
      //     RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kRightRumble, 0);
      //     end(true);
      //     }
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
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); // Turn off the Limelight LEDs
    RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kLeftRumble, 0);
    RobotContainer.driverXbox.setRumble(XboxController.RumbleType.kRightRumble, 0);
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); // Set the Limelight to the driver camera mode
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); // Set the Limelight to the cone pipeline
    
  }
}
