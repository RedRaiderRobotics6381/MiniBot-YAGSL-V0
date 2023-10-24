// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Arm.Intake.ArmIntakeInCmd;
// import frc.robot.commands.Arm.Intake.ArmIntakeOutCmd;
// import frc.robot.commands.Arm.Manipulator.ArmManipulatorDriveCmd;
// import frc.robot.commands.Arm.Manipulator.ArmManipulatorIntakeCmd;
import frc.robot.commands.swervedrive.auto.Autos;
// import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
// import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
// import frc.robot.commands.DefaultCommands.DefaultLimelightObjectDectionCommand;
// import frc.robot.commands.DefaultCommands.DefaultLimelightScoringDectionCommand;
// import frc.robot.subsystems.LimelightHelpers;

// import frc.robot.subsystems.Secondary.ArmSubsystem;
// import frc.robot.subsystems.Secondary.RotateSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

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

  // public final static ArmSubsystem armSubsystem = new ArmSubsystem();
  // public final static RotateSubsystem rotateSubsystem = new RotateSubsystem();

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

    // new JoystickButton(secondaryJoystick, 1).onTrue(Commands.parallel(new ArmManipulatorDriveCmd(rotateSubsystem)));
    // new JoystickButton(secondaryJoystick, 4).onTrue(Commands.parallel(new ArmManipulatorIntakeCmd(rotateSubsystem)));
    

    // new JoystickButton(secondaryJoystick, 3).whileTrue(new ArmIntakeInCmd(armSubsystem));
    // new JoystickButton(secondaryJoystick, 2).whileTrue(new ArmIntakeOutCmd(armSubsystem));

    
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
