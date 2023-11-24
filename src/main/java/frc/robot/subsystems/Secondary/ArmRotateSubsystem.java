// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotateSubsystem extends SubsystemBase {
  public static CANSparkMax m_armMotor;
  public SparkMaxPIDController m_armPIDController;
  public static SparkMaxAbsoluteEncoder m_armEncoder;
  public static double ArmRotateSetpoint;
  /** Creates a new ArmRotateSubSys. */
  public ArmRotateSubsystem(double ArmRotateSetpoint) {
        // initialize motor
        m_armMotor = new CANSparkMax(ArmConstants.kArmRotateMotor, MotorType.kBrushless);

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_armMotor.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        m_armEncoder = m_armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_armEncoder.setPositionConversionFactor(360);
        m_armEncoder.setZeroOffset(ArmConstants.posOffset);
        //m_armEncoder.setInverted(true);
    
        // initialze PID controller and encoder objects
        m_armPIDController = m_armMotor.getPIDController();
        m_armPIDController.setFeedbackDevice(m_armEncoder);
        m_armMotor.burnFlash();  //Remove this after everything is up and running to save flash wear
    
        // set PID coefficients
        m_armPIDController.setP(ArmConstants.armRotatekP);
        m_armPIDController.setI(ArmConstants.armRotatekI);
        m_armPIDController.setD(ArmConstants.armRotatekD);
        m_armPIDController.setIZone(ArmConstants.armRotatekIz);
        m_armPIDController.setFF(ArmConstants.armRotatekFF);
        m_armPIDController.setOutputRange(ArmConstants.armRotatekMinOutput, ArmConstants.armRotatekMaxOutput);
    
        /**
         * Smart Motion coefficients are set on a SparkMaxPIDController object
         * 
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */
        m_armPIDController.setSmartMotionMaxVelocity(ArmConstants.armRotateMaxVel, ArmConstants.armRotateSmartMotionSlot);
        m_armPIDController.setSmartMotionMinOutputVelocity(ArmConstants.armRotateMinVel, ArmConstants.armRotateSmartMotionSlot);
        m_armPIDController.setSmartMotionMaxAccel(ArmConstants.armRotateMaxAcc, ArmConstants.armRotateSmartMotionSlot);
        m_armPIDController.setSmartMotionAllowedClosedLoopError(ArmConstants.armRotateAllowedErr, ArmConstants.armRotateSmartMotionSlot);
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /**
     * As with other PID modes, Smart Motion is set by calling the
      * setReference method on an existing pid object and setting
      * the control type to kSmartMotion
    */
    m_armPIDController.setReference(ArmRotateSetpoint, CANSparkMax.ControlType.kSmartMotion);
  }
}
