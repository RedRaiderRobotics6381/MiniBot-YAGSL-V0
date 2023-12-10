// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.Secondary;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.revrobotics.SparkMaxAbsoluteEncoder;

// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ArmConstants;

// public class NewArmRotateSubsystem extends SubsystemBase {
//   public static CANSparkMax m_armMotor;
//   public static SparkMaxPIDController m_armPIDController;
//   public static SparkMaxAbsoluteEncoder m_armEncoder;
//   private double ArmRotateSetpoint;
//   //public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  
//   /**
//    * An alternate encoder object is constructed using the GetAlternateEncoder() 
//    * method on an existing CANSparkMax object. If using a REV Through Bore 
//    * Encoder, the type should be set to quadrature and the counts per 
//    * revolution set to 8192
//    */

//   /** Creates a new NewArmRotateSubsystem. */
//   public NewArmRotateSubsystem(double ArmRotateSetpoint) {
//     // initialize motor
//     m_armMotor = new CANSparkMax(ArmConstants.kArmRotateMotor, MotorType.kBrushless);

//     /**
//      * The RestoreFactoryDefaults method can be used to reset the configuration parameters
//      * in the SPARK MAX to their factory default state. If no argument is passed, these
//      * parameters will not persist between power cycles
//      */
//     m_armMotor.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
//     m_armEncoder = m_armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
//     m_armEncoder.setPositionConversionFactor(360);
//     m_armEncoder.setZeroOffset(ArmConstants.posOffset);

//     // initialze PID controller and encoder objects
//     m_armPIDController = m_armMotor.getPIDController();
//     m_armPIDController.setFeedbackDevice(m_armEncoder);
//     m_armMotor.burnFlash();  //Remove this after everything is up and running to save flash wear

//     // set PID coefficients
//     m_armPIDController.setP(ArmConstants.armRotatekP);
//     m_armPIDController.setI(ArmConstants.armRotatekI);
//     m_armPIDController.setD(ArmConstants.armRotatekD);
//     m_armPIDController.setIZone(ArmConstants.armRotatekIz);
//     m_armPIDController.setFF(ArmConstants.armRotatekFF);
//     m_armPIDController.setOutputRange(ArmConstants.armRotatekMinOutput, ArmConstants.armRotatekMaxOutput);

//     /**
//      * Smart Motion coefficients are set on a SparkMaxPIDController object
//      * 
//      * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
//      * the pid controller in Smart Motion mode
//      * - setSmartMotionMinOutputVelocity() will put a lower bound in
//      * RPM of the pid controller in Smart Motion mode
//      * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
//      * of the pid controller in Smart Motion mode
//      * - setSmartMotionAllowedClosedLoopError() will set the max allowed
//      * error for the pid controller in Smart Motion mode
//      */
//     m_armPIDController.setSmartMotionMaxVelocity(ArmConstants.armRotateMaxVel, ArmConstants.armRotateSmartMotionSlot);
//     m_armPIDController.setSmartMotionMinOutputVelocity(ArmConstants.armRotateMinVel, ArmConstants.armRotateSmartMotionSlot);
//     m_armPIDController.setSmartMotionMaxAccel(ArmConstants.armRotateMaxAcc, ArmConstants.armRotateSmartMotionSlot);
//     m_armPIDController.setSmartMotionAllowedClosedLoopError(ArmConstants.armRotateAllowedErr, ArmConstants.armRotateSmartMotionSlot);

//     // display PID coefficients on SmartDashboard
//     SmartDashboard.putNumber("P Gain", ArmConstants.armRotatekP);
//     SmartDashboard.putNumber("I Gain", ArmConstants.armRotatekI);
//     SmartDashboard.putNumber("D Gain", ArmConstants.armRotatekD);
//     SmartDashboard.putNumber("I Zone", ArmConstants.armRotatekIz);
//     SmartDashboard.putNumber("Feed Forward", ArmConstants.armRotatekFF);
//     SmartDashboard.putNumber("Max Output", ArmConstants.armRotatekMaxOutput);
//     SmartDashboard.putNumber("Min Output", ArmConstants.armRotatekMinOutput);

//     // display Smart Motion coefficients
//     SmartDashboard.putNumber("Max Velocity", ArmConstants.armRotateMaxVel);
//     SmartDashboard.putNumber("Min Velocity", ArmConstants.armRotateMinVel);
//     SmartDashboard.putNumber("Max Acceleration", ArmConstants.armRotateMaxAcc);
//     SmartDashboard.putNumber("Allowed Closed Loop Error", ArmConstants.armRotateAllowedErr);
//     //SmartDashboard.putNumber("Set Position", 0);
//     //SmartDashboard.putNumber("Set Velocity", 0);

//     SmartDashboard.putNumber("Encoder Position", m_armEncoder.getPosition());
//   }

//   // @Override
//   // public void periodic() {
//   //   // This method will be called once per scheduler run
//   // }
//   @Override
//   public void periodic() {

//     double setPoint, processVariable;
//     //boolean mode = SmartDashboard.getBoolean("Mode", false);
//     //if(mode) {
//     setPoint = SmartDashboard.getNumber("Set Velocity", 0);
//     //q: how can I get setPoint to be a variable that I can change with a joystick?  
//     //a: create a new variable, setPoint, and set it equal to the joystick value
//     //Q: how can I pass setvalue from robotcontainer to here?




//     /**
//      * As with other PID modes, Smart Motion is set by calling the
//       * setReference method on an existing pid object and setting
//       * the control type to kSmartMotion
//     */
//     m_armPIDController.setReference(ArmRotateSetpoint, CANSparkMax.ControlType.kSmartMotion);
//     processVariable = m_armEncoder.getPosition();
    
    
//     SmartDashboard.putNumber("SetPoint", setPoint);
//     SmartDashboard.putNumber("Process Variable", processVariable);
//     SmartDashboard.putNumber("Output", m_armMotor.getAppliedOutput());
//     SmartDashboard.putNumber("Encoder Position", m_armEncoder.getPosition());

//   }

// }
