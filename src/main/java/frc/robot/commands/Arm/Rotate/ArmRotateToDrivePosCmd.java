// package frc.robot.commands.Arm.Rotate;

// import com.fasterxml.jackson.annotation.JsonCreator.Mode;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.ArmFeedforward;
// //import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.subsystems.Secondary.ArmRotateSubsystem;

// public class ArmRotateToDrivePosCmd extends CommandBase {
//     double P;

//     public ArmRotateToDrivePosCmd(OldArmRotateSubsystem rotateSubsystem) {
//         addRequirements(rotateSubsystem);
//     }

//     @Override
//     public void initialize() {
//         //m_motor = new CANSparkMax(ArmConstants.kArmRotateMotor, MotorType.kBrushless);;
//         //armProfiledPIDController.
//         Constants.ArmConstants.manipulatorOn = true;
//         ArmConstants.manipulatorManual = false;
//     }

//     @Override
//     public void execute() {
//         P = ((Math.abs(OldArmRotateSubsystem.armRotateEncoder.getPosition() - ArmConstants.posDrive) + 50) / 300);
//         if (OldArmRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posDrive) {
//             OldArmRotateSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P * .5);
//             // System.out.println("up");
//         }
//         if (OldArmRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posDrive) {
//             OldArmRotateSubsystem.armRotateMotor.set(ArmConstants.rotateSpeed * P * .5);
//             // System.out.println("down");
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         OldArmRotateSubsystem.armRotateMotor.set(ArmConstants.posDriveGravity);
//         Constants.ArmConstants.manipulatorOn = false;
//         Robot.ManualRotation = true;
//     }

//     @Override
//     public boolean isFinished() {
//         if(OldArmRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posDrive - ArmConstants.rotateoffset &&
//            OldArmRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posDrive + ArmConstants.rotateoffset){
//             return true;
//         }else{
//             return false;
//         }
//     }
// }