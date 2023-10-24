// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  
//   public static final class ArmConstants {
//        // public static final int kLeftSliderMotor = 13;
//        //public static final int kRightSliderMotor = 14;
//        public static final int kArmRotateMotor = 13;
        
//        //public static final int kManipulatorWristMotor = 17;
//        public static final int kManipulatorIntakeMotorL = 14;
//        public static final int kManipulatorIntakeMotorR = 15;

//        // uncertain
//        public static final int gArmSliderBottom = 1;
//        public static final int gArmSliderTop = 41;
//        public static final int gArmSliderLow = 22;
//        public static final int gArmSliderHumanPlayer = 41;

//        public static final double gArmOffset = 1;
//        public static final double gRotateoffset = 0.25;

//        public static final double gSliderSpeed = -0.6;
//        public static final double gSliderDown = -0.4;
//        public static final double gRotateSpeed = 0.85;
//        public static final double gOutputSpeed = 1.00;
//        public static final double gIntakeSpeed = 0.50;

//        public static final double rotateoffset = 2.5;

//        // manipulator rotations
//        // human player
//        public static final double posDoubleHuman = 100;
//        // placing
//        public static final double posPlace = 117;
//        // up
//        public static final double posDrive = 245;
//        // intake
//        public static final double posIntake = 132.5;
//        // Single human player station
//        public static final double posSingularHuman = 125; //Match 24, was at 130
//        //Hybrid Node on front
//        public static final double posHybrid = 70;
       
//        public static final double posDoubleHumanGravity = 0.03;
//        public static final double posPlaceGravity = 0.03;
//        public static final double posDriveGravity = 0.01;
//        public static final double posIntakeGravity = 0.03;
//        public static final double posSingularHumanGravity = 0.02;
//        public static final double posHybridGravity = 0.04;
//        public static final double restriction1 = 70;
//        public static final double restriction2 = 291;
//        public static final double rotateSpeed = 1;

//        public static boolean manipulatorOn = false;
//        public static boolean manipulatorManual = false;
//        public static int rightYPort = 5;
// }
}
