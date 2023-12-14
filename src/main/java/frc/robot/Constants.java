// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import org.opencv.core.Mat.Atable;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (80 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
  }
  
  public static final class ArmConstants {
       public static final int kArmRotateMotor = 13;
       public static final int kManipulatorIntakeMotorL = 14;
       public static final int kManipulatorIntakeMotorR = 15;
       public static final int kMotorPort = 4;


       public static final double gOutputSpeed = 1.00;
       public static final double gIntakeSpeed = 0.50;
       public static final double posOffset = 72.5;
       public static final double posDrive = 190; //Was 200 see note in ArmRotateSubsystem.java
       public static final double posIntake = 90; //Was 132.5 see note in ArmRotateSubsystem.java

       public static final double armRotatekP = .00000024;
       public static final double armRotatekI = .000000;
       public static final double armRotatekD = 0;
       public static final double armRotatekIz = 0; 
       public static final double armRotatekFF = 0.000156;
       public static final double armRotatekMaxOutput = 1; 
       public static final double armRotatekMinOutput = -1;
       public static final double armRotateMaxRPM = 5700;
       public static final double armRotateMaxVel = 5000;
       public static final double armRotateMaxAcc = 3000;
       public static final double armRotateMinVel = 0;
       public static final double armRotateAllowedErr = .01;
       public static final int armRotateSmartMotionSlot = 0;
 
       public static boolean manipulatorOn = false;
       public static boolean manipulatorManual = false;

       //Simulation
       //public static final int kMotorPort = 0;
       public static final int kEncoderAChannel = 0;
       public static final int kEncoderBChannel = 1;
       public static final String kArmPositionKey = "ArmPosition";
       public static final String kArmPKey = "ArmP";
     
       // The P gain for the PID controller that drives this arm.
       public static final double kDefaultArmKp = armRotatekP; //50.0;
       public static final double kDefaultArmSetpointDegrees = posIntake; //100.0;
       public static final double kArmSetpoint = ArmRotateSubsystem.ArmRotateSetpoint; //100;
     
       // distance per pulse = (angle per revolution) / (pulses per revolution)
       //  = (2 * PI rads) / (8192 pulses)
       public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 8192;
     
       public static final double kArmReduction = 125;
       public static final double kArmMass = 4.536; // Kilograms = 10lbs
       public static final double kArmLength = Units.inchesToMeters(10);
       public static final double kMinAngleRads = Units.degreesToRadians(0);
       public static final double kMaxAngleRads = Units.degreesToRadians(240);
  }
}
