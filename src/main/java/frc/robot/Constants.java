// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import org.opencv.core.Mat.Atable;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

       public static final double kP = 1;
   
       // These are fake gains; in actuality these must be determined individually for each robot
       public static final double kSVolts = 1;
       public static final double kGVolts = 1;
       public static final double kVVoltSecondPerRad = 0.5;
       public static final double kAVoltSecondSquaredPerRad = 0.1;
   
       public static final double kMaxVelocityRadPerSecond = 3;
       public static final double kMaxAccelerationRadPerSecSquared = 10;
   
       public static final int[] kEncoderPorts = new int[] {4, 5};
       public static final int kEncoderPPR = 256;
       public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;
   
       // The offset of the arm from the horizontal in its neutral position,
       // measured from the horizontal
       public static final double kArmOffset = 0.5;

       public static final double gOutputSpeed = 1.00;
       public static final double gIntakeSpeed = 0.50;
       public static final double rotateoffset = Math.toRadians(2.5);
       public static final double posOffset = 72.5;
       public static final double posDrive = 190; //Was 200 see note in ArmRotateSubsystem.java
       public static final double posIntake = 90; //Was 132.5 see note in ArmRotateSubsystem.java
       public static final double posDriveGravity = 0.01;
       public static final double posIntakeGravity = 0.03;
       public static final double rotateSpeed = 1;

       public static final double armRotatekP = .000005; 
       public static final double armRotatekI = .000001;
       public static final double armRotatekD = 0; 
       public static final double armRotatekIz = 0; 
       public static final double armRotatekFF = 0.000156; 
       public static final double armRotatekMaxOutput = 1; 
       public static final double armRotatekMinOutput = -1;
       public static final double armRotateMaxRPM = 5700;
       public static final double armRotateMaxVel = 2000;
       public static final double armRotateMaxAcc = 1500;
       public static final double armRotateMinVel = 0;
       public static final double armRotateAllowedErr = 0;
       public static final int armRotateSmartMotionSlot = 0;
 
       public static boolean manipulatorOn = false;
       public static boolean manipulatorManual = false;
  
      //  uncertain
      //  public static final int gArmSliderBottom = 1;
      //  public static final int gArmSliderTop = 41;
      //  public static final int gArmSliderLow = 22;
      //  public static final int gArmSliderHumanPlayer = 41;
      //  public static final double gArmOffset = 1;
      //  public static final double gRotateoffset = 0.25;
      //  public static final double gSliderSpeed = -0.6;
      //  public static final double gSliderDown = -0.4;
      //  public static final double gRotateSpeed = 0.85;
      //  public static final double posDoubleHuman = 100;
      //  public static final double posPlace = 117;
      //  Single human player station
      //  public static final double posSingularHuman = 125; //Match 24, was at 130
      //  Hybrid Node on front
      //  public static final double posHybrid = 70;
       
      //  public static final double posDoubleHumanGravity = 0.03;
      //  public static final double posPlaceGravity = 0.03;
      //  public static final double posSingularHumanGravity = 0.02;
      //  public static final double posHybridGravity = 0.04;
      //  public static final double restriction1 = 70;
      //  public static final double restriction2 = 291;
      //  public static int rightYPort = 5;
  }
  
  public static final class SensorConstants {
    public static PIDController PIDspeed = new PIDController(0.20, 0, 0);
    public static PIDController PIDside = new PIDController(0.06, 0, 0);
    public static PIDController PIDturn = new PIDController(0.005, 0, 0);
    public static PIDController PIDcharging = new PIDController(0.05, 0, 0);
  }

}
