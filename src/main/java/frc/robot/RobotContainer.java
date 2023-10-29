// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Arm.Rotate.ArmRotateToDrivePosCmd;
import frc.robot.commands.Arm.Rotate.ArmRotateToIntakePos;
//import frc.robot.commands.Vision.CubePickupHelper;
import frc.robot.commands.Vision.DriveToCube;
// import frc.robot.commands.Vision.ConePickupHelper;
//import frc.robot.commands.Vision.CubePickupHelper;
import frc.robot.commands.Arm.Intake.ArmIntakeInCmd;
import frc.robot.commands.Arm.Intake.ArmIntakeOutCmd;
//import frc.robot.commands.Arm.Rotate.ArmRotateToDrivePosCmd;
//import frc.robot.commands.Arm.Rotate.ArmRotateToIntakePos;
// import frc.robot.commands.Arm.Intake.ArmIntakeInCmd;
// import frc.robot.commands.Arm.Intake.ArmIntakeOutCmd;
// import frc.robot.commands.Arm.Manipulator.ArmManipulatorDriveCmd;
// import frc.robot.commands.Arm.Manipulator.ArmManipulatorIntakeCmd;
import frc.robot.commands.swervedrive.auto.Autos;
// import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
// import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
// import frc.robot.subsystems.LimelightHelpers;
// import frc.robot.commands.DefaultCommands.DefaultLimelightObjectDectionCommand;
// import frc.robot.commands.DefaultCommands.DefaultLimelightScoringDectionCommand;
// import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Secondary.ArmIntakeSubsystem;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import swervelib.SwerveDrive;
//import swervelib.SwerveDrive;

import java.io.File;
// import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  //CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  // public final static XboxController secondaryJoystick = new XboxController(1);

  private final ArmRotateSubsystem armRotateSubsystem = new ArmRotateSubsystem();
  private final ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  // private final PIDController controller;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
    //   // Applies deadbands and inverts controls because joysticks
    //   // are back-right positive while robot
    //   // controls are front-left positive
    //   () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //   () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //   () -> -driverXbox.getRightX(),
    //   () -> -driverXbox.getRightX(),                                                          
    //   false);

    // AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
    //   () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //   () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //   () -> -driverXbox.getRightX(),
    //   false);

    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverXbox.getRightX(),
      () -> true, false, false);

    TeleopDrive closedFieldRel = new TeleopDrive(drivebase,
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverXbox.getRightX(),
      () -> true, false, false);

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedFieldRel : simClosedFieldRel);
    // drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedFieldAbsoluteDrive : closedAbsoluteDrive);
    //drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    // Secondary

    new JoystickButton(driverXbox, 1).onTrue(Commands.parallel(new ArmRotateToDrivePosCmd(armRotateSubsystem)));
    new JoystickButton(driverXbox, 4).onTrue(Commands.parallel(new ArmRotateToIntakePos(armRotateSubsystem)));
    

    new JoystickButton(driverXbox, 3).whileTrue(new ArmIntakeInCmd(armIntakeSubsystem));
    new JoystickButton(driverXbox, 2).whileTrue(new ArmIntakeOutCmd(armIntakeSubsystem));
    new JoystickButton(driverXbox, 5).onTrue(new DriveToCube(drivebase)); 
    // double TX;
    // double TY;
    // DoubleSupplier translationValX;
    // DoubleSupplier translationValY;
    // new JoystickButton(driverXbox, 5).onTrue(Commands.sequence(
            // Turn on Limelight LED and set camera mode
      //LimelightHelpers.setLEDMode_ForceOn("");
      //LimelightHelpers.setCameraMode_Processor("");
      //LimelightHelpers.setPipelineIndex("", 1); // Set the Limelight to the cone pipeline
      // new ArmRotateToIntakePos(armRotateSubsystem),
      // new ArmIntakeInCmd(armIntakeSubsystem),
      // // Get the TX and TY values from the Limelight
      // TX = LimelightHelpers.getTX("") + .5, // Add .5 to TX to center the robot on the target
      // TY = LimelightHelpers.getTY("") - .1, // Subtract .1 from TY to center the robot on the target
      // controller.setTolerance(1),
      // controller.setSetpoint(0.0),
      //translationValX = (MathUtil.clamp(controller.calculate(LimelightHelpers.getTX("") + .5, 0.25), 0, 2)), // Clamp the translation values
      //translationValY = (MathUtil.clamp(controller.calculate(LimelightHelpers.getTY("") - .1, -.1), -2, 2)), // Clamp the translation values
      
      //SmartDashboard.putNumber("Cone TX", LimelightHelpers.getTX(""));
      //SmartDashboard.putNumber("TY", LimelightHelpers.getTY(""));
      //SmartDashboard.putNumber("ID", LimelightHelpers.getNeuralClassID(""));
      
      // Set the speed and angle of each wheel on the swerve drive
      //new TeleopDrive(drivebase, null, null, null, null, false, false)
    //   drivebase.drive(new Translation2d(
    //                                     (MathUtil.clamp(controller.calculate(LimelightHelpers.getTX("") + .5, 0.25), 0, 2)),
    //                                     (MathUtil.clamp(controller.calculate(LimelightHelpers.getTY("") - .1, -.1), -2, 2))),
    //                                     0,
    //                                     false,
    //                                     false),
    //   new ArmRotateToDrivePosCmd(armRotateSubsystem)
    // ));
    
    //new JoystickButton(driverXbox, 6).onTrue(new CubePickupHelper(armIntakeSubsystem,armRotateSubsystem));
    


    
    // new JoystickButton(secondaryJoystick, 9).onTrue(Commands.parallel(new ArmSliderBottomCmd(armSubsystem),
    //               new ArmManipulatorSingleHumanCmd(rotateSubsystem)));

    // new JoystickButton(secondaryJoystick, 3).onTrue(Commands.parallel(new ArmSliderBottomCmd(armSubsystem),
    //               new ArmManipulatorHybridCmd(rotateSubsystem)));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Autos.exampleAuto(drivebase);
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}
