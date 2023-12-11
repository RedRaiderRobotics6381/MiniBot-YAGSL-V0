// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import frc.robot.Constants;
//import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotateSubsystem extends SubsystemBase {
  public static CANSparkMax m_armMotor;
  public static SparkMaxPIDController m_armPIDController;
  public static SparkMaxAbsoluteEncoder ArmEncoder;
  public static double ArmRotateSetpoint;
  public static double RotateManualPos;
  
  // The P gain for the PID controller that drives this arm.
  private double m_armKp = ArmConstants.kDefaultArmKp;
  private double m_armSetpointDegrees = ArmConstants.kDefaultArmSetpointDegrees;

  // The arm gearbox represents a gearbox containing one Neo motor.
  private final DCMotor m_armGearbox = DCMotor.getNEO(1);
  private final Encoder m_encoder =
  new Encoder(ArmConstants.kEncoderAChannel, ArmConstants.kEncoderBChannel);
  private final PIDController m_controller = new PIDController(m_armKp, 0, 0);
  
  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from 90 degrees (rotated down front)
  // to 190 degrees (rotated up).
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          ArmConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(ArmConstants.kArmLength, ArmConstants.kArmMass),
          ArmConstants.kArmLength,
          ArmConstants.kMinAngleRads,
          ArmConstants.kMaxAngleRads,
          true,
          VecBuilder.fill(ArmConstants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 10, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));
  //public static double RotateManualPos;
  /** Creates a new ArmRotateSubSys. 
 * @param armRotateSubsystem*/
  public ArmRotateSubsystem() {
        // initialize motor
        m_armMotor = new CANSparkMax(ArmConstants.kArmRotateMotor, MotorType.kBrushless);
        m_encoder.setDistancePerPulse(ArmConstants.kArmEncoderDistPerPulse); //for simulation
        
        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm Sim", m_mech2d);
        m_armTower.setColor(new Color8Bit(Color.kBlue));

        // Set the Arm position setpoint and P constant to Preferences if the keys don't already exist
        Preferences.initDouble(ArmConstants.kArmPositionKey, m_armSetpointDegrees);
        Preferences.initDouble(ArmConstants.kArmPKey, m_armKp);

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_armMotor.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        ArmEncoder = m_armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        ArmEncoder.setPositionConversionFactor(360);
        ArmEncoder.setZeroOffset(72.5); //ArmConstants.posOffset);
        //m_armEncoder.setInverted(true);
    
        // initialze PID controller and encoder objects
        m_armPIDController = m_armMotor.getPIDController();
        m_armPIDController.setFeedbackDevice(ArmEncoder);
        m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 85); //ArmConstants.posLowerLimit
        m_armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 241); //ArmConstants.posUpperLimit); 
        m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_armMotor.enableVoltageCompensation(12.0);
        m_armMotor.setSmartCurrentLimit(25);
        m_armMotor.burnFlash();  //Remove this after everything is up and running to save flash wear
    
        // set PID coefficients
        m_armPIDController.setP(0.000062); //ArmConstants.armRotatekP);
        m_armPIDController.setI(0.0); //ArmConstants.armRotatekI);
        m_armPIDController.setD(0.0); //ArmConstants.armRotatekD);
        m_armPIDController.setIZone(0.0); //ArmConstants.armRotatekIz);
        // This is an arbitrary feedforward value that is multiplied by the positon of the arm to account
        // for the reduction in force needed to hold the arm vertical instead of hortizontal.  The .abs
        //ensures the value is always positive.  The .cos function uses radians instead of degrees,
        // so the .toRadians converts from degrees to radians.
       // m_armPIDController.setFF(.00005 * (Math.abs(Math.cos(Math.toRadians(ArmRotateSetpoint-90)))));
       //m_armPIDController.setFF(.005 * (Math.abs(Math.cos(Math.toRadians(ArmRotateSetpoint-90)))));
       double ArmRotateSetpointRad = Math.toRadians(ArmRotateSetpoint);
       m_armPIDController.setFF(.005 * (Math.abs(Math.cos(ArmRotateSetpointRad - (Math.toRadians(90))))));
       //m_armPIDController.setFF(.005);
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
    //Q: How can I ensure a multipled value always returns a positive number?
    
  }

@Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Enc Val", ArmEncoder.getPosition());
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
    /** Update the simulation model. */
    public void simulationPeriodic() {
      // In this method, we update our simulation of what our arm is doing
      // First, we set our "inputs" (voltages)
      m_armSim.setInput(m_armMotor.get() * RobotController.getBatteryVoltage());
  
      // Next, we update it. The standard loop time is 20ms.
      m_armSim.update(0.020);
  
      // Finally, we set our simulated encoder's readings and simulated battery voltage
      m_encoderSim.setDistance(m_armSim.getAngleRads());
      // SimBattery estimates loaded battery voltages
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
  
      // Update the Mechanism Arm angle based on the simulated arm angle
      m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    }
  
    /** Load setpoint and kP from preferences. */
    public void loadPreferences() {
      // Read Preferences for Arm setpoint and kP on entering Teleop
      m_armSetpointDegrees = Preferences.getDouble(ArmConstants.kArmPositionKey, m_armSetpointDegrees);
      if (m_armKp != Preferences.getDouble(ArmConstants.kArmPKey, m_armKp)) {
        m_armKp = Preferences.getDouble(ArmConstants.kArmPKey, m_armKp);
        m_controller.setP(m_armKp);
      }
    }
  
    /** Run the control loop to reach and maintain the setpoint from the preferences. */
    public void reachSetpoint(double kArmSetpoint) {
      var pidOutput =
          m_controller.calculate(
              m_encoder.getDistance(), Units.degreesToRadians(kArmSetpoint));
          m_armMotor.setVoltage(pidOutput);
    }
  
}
