package frc.robot.commands.Arm.Rotate;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;

public class ArmRotateToDrivePosCmd extends CommandBase {

    private final ArmRotateSubsystem rotateSubsystem;
    double P;

    public ArmRotateToDrivePosCmd(ArmRotateSubsystem rotateSubsystem) {
        this.rotateSubsystem = rotateSubsystem;
        addRequirements(rotateSubsystem);
    }

    @Override
    public void initialize() {
        Constants.ArmConstants.manipulatorOn = true;
        ArmConstants.manipulatorManual = false;
    }

    @Override
    public void execute() {
        P = ((Math.abs(rotateSubsystem.armRotateEncoder.getPosition() - ArmConstants.posDrive) + 50) / 300);
        if (rotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posDrive + ArmConstants.rotateoffset) {
            rotateSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P*.75);
            // System.out.println("up");
        }
        if (rotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posDrive - ArmConstants.rotateoffset) {
            rotateSubsystem.armRotateMotor.set(ArmConstants.rotateSpeed * P*.75);
            // System.out.println("down");
        }
    }

    @Override
    public void end(boolean interrupted) {
        rotateSubsystem.armRotateMotor.set(ArmConstants.posDriveGravity);
        Constants.ArmConstants.manipulatorOn = false;
    }

    @Override
    public boolean isFinished() {
        if(rotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posDrive - ArmConstants.rotateoffset && rotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posDrive + ArmConstants.rotateoffset){
            return true;
        }else{
            return false;
        }
    }
}