// package frc.robot.commands.Arm.Rotate;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.subsystems.Secondary.ArmRotateSubsystem;

// public class ArmRotateToIntakePos extends CommandBase {

//     double P;

//     public ArmRotateToIntakePos(ArmRotateSubsystem armRotateSubsystem) {
//         addRequirements(armRotateSubsystem);
//     }

//     @Override
//     public void initialize() {
//         Constants.ArmConstants.manipulatorOn = true;
//         ArmConstants.manipulatorManual = false;
//     }

//     @Override
//     public void execute() {
//         P = ((Math.abs(ArmRotateSubsystem.armRotateEncoder.getPosition() - ArmConstants.posIntake)+30)/300);
//         if(ArmRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posIntake + ArmConstants.rotateoffset){
//             ArmRotateSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P * .5);
//             // System.out.println("up");
//            }
//            if(ArmRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posIntake - ArmConstants.rotateoffset){
//             ArmRotateSubsystem.armRotateMotor.set(ArmConstants.rotateSpeed * P * .5);
//             // System.out.println("down");
//            }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         ArmRotateSubsystem.armRotateMotor.set(ArmConstants.posIntakeGravity);
//         Constants.ArmConstants.manipulatorOn = false;
//         Robot.ManualRotation = true;
//     }

//     @Override
//     public boolean isFinished() {
//         if(ArmRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posIntake - ArmConstants.rotateoffset && 
//            ArmRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posIntake + ArmConstants.rotateoffset){
//             return true;
//         }else{
//             return false;
//         }
//     }
// }