package frc.robot.subsystems.Secondary;

import com.fasterxml.jackson.databind.ser.std.StaticListSerializerBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmRotateSubsystem extends SubsystemBase {

    public static CANSparkMax armRotateMotor;
    public static SparkMaxAbsoluteEncoder armRotateEncoder;

    public ArmRotateSubsystem() {

        armRotateMotor = new CANSparkMax(Constants.ArmConstants.kArmRotateMotor,MotorType.kBrushless);
        armRotateMotor.setInverted(false);

        armRotateEncoder = armRotateMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        armRotateEncoder.setPositionConversionFactor(360);
        armRotateEncoder.setZeroOffset(25   );
        armRotateEncoder.setPositionConversionFactor(0);
    }
}