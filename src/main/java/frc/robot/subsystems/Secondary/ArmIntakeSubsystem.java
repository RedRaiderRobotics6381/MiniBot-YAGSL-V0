package frc.robot.subsystems.Secondary;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmIntakeSubsystem extends SubsystemBase {

    public CANSparkMax intakeMotorL;
    public CANSparkMax intakeMotorR;

    public ArmIntakeSubsystem() {
        intakeMotorL =  new CANSparkMax(Constants.ArmConstants.kManipulatorIntakeMotorL, MotorType.kBrushless);
        intakeMotorR =  new CANSparkMax(Constants.ArmConstants.kManipulatorIntakeMotorR, MotorType.kBrushless);
    }
}