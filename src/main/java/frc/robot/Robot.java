// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
//import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.Constants.ArmConstants;
import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.subsystems.LimelightHelpers;
//import frc.robot.subsystems.Secondary.OldArmRotateSubsystem;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;

import java.io.File;
import java.io.IOException;

//import com.revrobotics.CANSparkMax;

// import javax.swing.plaf.TreeUI;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static boolean ManualRotation= true;
  //private final ArmRotateSubsystem armRotateSubsystem = new ArmRotateSubsystem();

  private Timer disabledTimer;
  double targetPos = 150;
  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    //PathPlannerServer.startServer(5811);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    Autos.init();
    // Make sure you only configure port forwarding once in your robot code.
    // Do not place these function calls in any periodic functions
    // for (int port = 5800; port <= 5807; port++) {
    //     PortForwarder.add(port, "limelight.local", port);
    //   }
    DriverStation.silenceJoystickConnectionWarning(true); // Disable joystick connection warning
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    //m_autonomousCommand = Autos.getAutonomousCommand();
    //m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.setDriveMode();
    //m_robotContainer.setMotorBrake(true);
    ArmRotateSubsystem.ArmRotateSetpoint = 90;
    LimelightHelpers.setLEDMode_ForceOn("");
    LimelightHelpers.setCameraMode_Processor("");


  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic(){
    //ArmRotateSubsystem.m_armPIDController.setReference(ArmRotateSubsystem.ArmRotateSetpoint, CANSparkMax.ControlType.kSmartMotion);
  // SmartDashboard.putNumber("Arm Position", OldArmRotateSubsystem.armRotateEncoder.getPosition());
  //    if(ManualRotation){
  //     if(OldArmRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posIntake){
  //       if(RobotContainer.engineerXbox.getRawAxis(1) > 0.05){
  //         OldArmRotateSubsystem.armRotateMotor.set(RobotContainer.engineerXbox.getRawAxis(1)*0.25);
  //         }else OldArmRotateSubsystem.armRotateMotor.set(0);
  //       }
  //         else OldArmRotateSubsystem.armRotateMotor.set(0);
  //      if(OldArmRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posDrive-5){
  //       if(RobotContainer.engineerXbox.getRawAxis(1) < -0.05){
  //         OldArmRotateSubsystem.armRotateMotor.set(RobotContainer.engineerXbox.getRawAxis(1)*0.25);
  //         }else OldArmRotateSubsystem.armRotateMotor.set(0);
  //       }
  //     if(OldArmRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posIntake &&
  //        OldArmRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posDrive){
  //          OldArmRotateSubsystem.armRotateMotor.set(RobotContainer.engineerXbox.getRawAxis(1)*0.25);
  //        }
  //     }
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  //   if(RobotBase.isSimulation()) {
  //     NetworkTableInstance inst = NetworkTableInstance.getDefault();
  //     inst.stopServer();
  //     // Change the IP address in the below function to the IP address you use to connect to the PhotonVision UI.
  //     inst.setServer("10.63.81.11");
  //     inst.startClient4("Robot Simulation");
  //  }
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
