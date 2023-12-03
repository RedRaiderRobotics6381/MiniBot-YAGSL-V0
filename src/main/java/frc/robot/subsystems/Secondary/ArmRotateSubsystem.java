// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotateSubsystem extends SubsystemBase {
  public static CANSparkMax m_armMotor;
  public static SparkMaxPIDController m_armPIDController;
  public static SparkMaxAbsoluteEncoder m_armEncoder;
  public static double ArmRotateSetpoint;
  public static double RotateManualPos;
  //public static double RotateManualPos;
  /** Creates a new ArmRotateSubSys. 
 * @param armRotateSubsystem*/
  public ArmRotateSubsystem() {
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
        m_armEncoder.setZeroOffset(72.5); //ArmConstants.posOffset);
        //m_armEncoder.setInverted(true);
    
        // initialze PID controller and encoder objects
        m_armPIDController = m_armMotor.getPIDController();
        m_armPIDController.setFeedbackDevice(m_armEncoder);
        m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 85); //ArmConstants.posLowerLimit
        m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 220); //ArmConstants.posUpperLimit); 
        m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_armMotor.setSmartCurrentLimit(20);
        m_armMotor.burnFlash();  //Remove this after everything is up and running to save flash wear
    
        // set PID coefficients
        m_armPIDController.setP(.00000024); //ArmConstants.armRotatekP);
        m_armPIDController.setI(0.0); //ArmConstants.armRotatekI);
        m_armPIDController.setD(0.0); //ArmConstants.armRotatekD);
        m_armPIDController.setIZone(0.0); //ArmConstants.armRotatekIz);
        m_armPIDController.setFF(.000156); //ArmConstants.armRotatekFF);
        m_armPIDController.setOutputRange(-1.0, 1.0); //ArmConstants.armRotatekMinOutput, ArmConstants.armRotatekMaxOutput);
    
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
        m_armPIDController.setSmartMotionMaxVelocity(5000.0,0); //ArmConstants.armRotateMaxVel, ArmConstants.armRotateSmartMotionSlot);
        m_armPIDController.setSmartMotionMinOutputVelocity(0.0, 0); //ArmConstants.armRotateMinVel, ArmConstants.armRotateSmartMotionSlot);
        m_armPIDController.setSmartMotionMaxAccel(3000.0,0); //ArmConstants.armRotateMaxAcc, ArmConstants.armRotateSmartMotionSlot);
        m_armPIDController.setSmartMotionAllowedClosedLoopError(0.01, 0); //ArmConstants.armRotateAllowedErr, ArmConstants.armRotateSmartMotionSlot);
    
    
  }

@Override
  public void periodic() {
    // This method will be called once per scheduler run

    /**
     * As with other PID modes, Smart Motion is set by calling the
      * setReference method on an existing pid object and setting
      * the control type to kSmartMotion
    */
    // while(RobotContainer.engineerXbox.getRawAxis(1) > 0.1 || RobotContainer.engineerXbox.getRawAxis(1) < -0.1){
    //   if (m_armEncoder.getPosition() * 360 > ArmConstants.posDrive && m_armEncoder.getPosition() * 360 < ArmConstants.posIntake) {
    //     ArmRotateSetpoint = ArmRotateSetpoint + 1 * RobotContainer.engineerXbox.getRawAxis(1);
    //     }
    //   }
    //m_armPIDController.setReference(ArmRotateSetpoint, CANSparkMax.ControlType.kSmartMotion);
  }


  
  public CommandBase rotatePosCommand(double ArmRotateSetpoint) {
    // implicitly require `this`
    return this.run(() -> m_armPIDController.setReference(ArmRotateSetpoint, CANSparkMax.ControlType.kSmartMotion));
  }
  
  // public void rotateDriveCommand() {
  //   // implicitly require `this`
  //   // return this.run(() -> m_armPIDController.setReference(ArmConstants.posDrive, CANSparkMax.ControlType.kSmartMotion));
  //   //return this.runOnce(() -> ArmRotateSubsystem.ArmRotateSetpoint =  ArmConstants.posDrive);
  //   ArmRotateSubsystem.ArmRotateSetpoint =  ArmConstants.posDrive;
  //   //m_armPIDController.setReference(ArmConstants.posDrive, CANSparkMax.ControlType.kSmartMotion);

  // }
  // public void rotateIntakeCommand() {
  //   // implicitly require `this`
  //   //return this.run(() -> m_armPIDController.setReference(ArmConstants.posIntake, CANSparkMax.ControlType.kSmartMotion));
  //   //return this.runOnce(() -> ArmRotateSubsystem.ArmRotateSetpoint =  ArmConstants.posIntake);
  //   ArmRotateSubsystem.ArmRotateSetpoint =  ArmConstants.posIntake;
  //   //m_armPIDController.setReference(ArmConstants.posIntake, CANSparkMax.ControlType.kSmartMotion);
  // }
  // public CommandBase rotateManualCommand() {
  //   // implicitly require `this`
  //   return this.runOnce(() -> m_armPIDController.setReference(RobotContainer.RotateManualPos, CANSparkMax.ControlType.kSmartMotion));
  // }
  public void setDefaultCommand() {
    m_armPIDController.setReference(ArmRotateSetpoint, CANSparkMax.ControlType.kSmartMotion);
  }
}
