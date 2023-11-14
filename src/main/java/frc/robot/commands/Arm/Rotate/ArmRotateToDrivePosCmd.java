package frc.robot.commands.Arm.Rotate;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;

public class ArmRotateToDrivePosCmd extends CommandBase {

    double P;

    public ArmRotateToDrivePosCmd(ArmRotateSubsystem rotateSubsystem) {
        addRequirements(rotateSubsystem);
    }

    @Override
    public void initialize() {
        Constants.ArmConstants.manipulatorOn = true;
        ArmConstants.manipulatorManual = false;
    }

    @Override
    public void execute() {
        P = ((Math.abs(ArmRotateSubsystem.armRotateEncoder.getPosition() - ArmConstants.posDrive) + 50) / 300);
        if (ArmRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posDrive + ArmConstants.rotateoffset) {
            ArmRotateSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P*.75);
            // System.out.println("up");
        }
        if (ArmRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posDrive - ArmConstants.rotateoffset) {
            ArmRotateSubsystem.armRotateMotor.set(ArmConstants.rotateSpeed * P*.75);
            // System.out.println("down");
        }
    }

    @Override
    public void end(boolean interrupted) {
        ArmRotateSubsystem.armRotateMotor.set(ArmConstants.posDriveGravity);
        Constants.ArmConstants.manipulatorOn = false;
        Robot.ManualRotation = true;
    }

    @Override
    public boolean isFinished() {
        if(ArmRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posDrive - ArmConstants.rotateoffset &&
           ArmRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posDrive + ArmConstants.rotateoffset){
            return true;
        }else{
            return false;
        }
    }
}