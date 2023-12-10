 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Arm.Rotate.ArmPosition;
//import frc.robot.commands.Arm.Rotate.ArmRotateToDrivePosCmd;
//import frc.robot.commands.Arm.Rotate.ArmRotateToIntakePos;
//import frc.robot.commands.Arm.Rotate.ArmRotateCmd;
//import frc.robot.commands.Vision.DriveToObject;
import frc.robot.commands.Vision.LLDriveToObjectCmd;
//import frc.robot.commands.Vision.PVDriveToObjectCmd;
import frc.robot.commands.Arm.Intake.ArmIntakeInCmd;
import frc.robot.commands.Arm.Intake.ArmIntakeOutCmd;
import frc.robot.commands.swervedrive.auto.AutoBalanceCommand;
import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.Secondary.ArmIntakeSubsystem;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;
//import frc.robot.subsystems.Secondary.OldArmRotateSubsystem;
//import frc.robot.subsystems.Secondary.NewArmRotateSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


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
  public final static XboxController driverXbox = new XboxController(0);
  public final static XboxController engineerXbox = new XboxController(1);



  
  private final ArmIntakeSubsystem armIntakeSubsystem = new ArmIntakeSubsystem();
  private final ArmRotateSubsystem armRotateSubsystem = new ArmRotateSubsystem();
  // private final PIDController controller;
  public static double RotateManualPos;

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

    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    // Secondary

    new JoystickButton(driverXbox, 4).onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(driverXbox, 2).whileTrue(new AutoBalanceCommand(drivebase));

    new JoystickButton(engineerXbox, 1).onTrue(armRotateSubsystem.rotatePosCommand(ArmConstants.posDrive)); // 180 is vertical
    new JoystickButton(engineerXbox, 4).onTrue(armRotateSubsystem.rotatePosCommand(ArmConstants.posIntake)); //90 is horizontal

    new JoystickButton(engineerXbox,3 ).whileTrue(new ArmIntakeInCmd(armIntakeSubsystem));
    new JoystickButton(engineerXbox,2 ).whileTrue(new ArmIntakeOutCmd(armIntakeSubsystem));
    
    

    //new JoystickButton(engineerXbox,7 ).whileTrue(new DriveGyro180Cmd(swerveSubsystem));

    new JoystickButton(driverXbox, 5).whileTrue(new LLDriveToObjectCmd(drivebase, 0));
    //new JoystickButton(driverXbox, 5).onFalse((new InstantCommand(drivebase::lock)));
    new JoystickButton(driverXbox, 6).whileTrue(new LLDriveToObjectCmd(drivebase, 1));
    //new JoystickButton(driverXbox, 6).onFalse((new InstantCommand(drivebase::lock)));
    //new JoystickButton(driverXbox, 5).whileTrue(new PVDriveToObjectCmd(drivebase, 3));
    //new JoystickButton(driverXbox, 6).whileTrue(new PVDriveToObjectCmd(drivebase, 1));

    if(RobotContainer.engineerXbox.getRightY() > 0.1 || RobotContainer.engineerXbox.getRightY() < -0.1){
    while (ArmRotateSubsystem.ArmRotateSetpoint < ArmConstants.posDrive && ArmRotateSubsystem.ArmRotateSetpoint > ArmConstants.posIntake){
      RotateManualPos += ArmRotateSubsystem.ArmEncoder.getPosition() + (engineerXbox.getRightY() * 2);
      armRotateSubsystem.rotatePosCommand(RotateManualPos);
      }
    }
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
