package frc.robot.commands.Arm.Rotate;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;

public class ArmRotateToIntakePos extends CommandBase {

    private final ArmRotateSubsystem armRotateSubsystem;
    double P;

    public ArmRotateToIntakePos(ArmRotateSubsystem armRotateSubsystem) {
        this.armRotateSubsystem = armRotateSubsystem;
        addRequirements(armRotateSubsystem);
    }

    @Override
    public void initialize() {
        Constants.ArmConstants.manipulatorOn = true;
        ArmConstants.manipulatorManual = false;
    }

    @Override
    public void execute() {
        P = ((Math.abs(armRotateSubsystem.armRotateEncoder.getPosition() - ArmConstants.posIntake)+30)/300);
        if(armRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posIntake + ArmConstants.rotateoffset){
            armRotateSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P*.75);
            // System.out.println("up");
           }
           if(armRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posIntake - ArmConstants.rotateoffset){
            armRotateSubsystem.armRotateMotor.set(ArmConstants.rotateSpeed * P*.75);
            // System.out.println("down");
           }
    }

    @Override
    public void end(boolean interrupted) {
        armRotateSubsystem.armRotateMotor.set(ArmConstants.posIntakeGravity);
        Constants.ArmConstants.manipulatorOn = false;
        Robot.ManualRotation = true;
    }

    @Override
    public boolean isFinished() {
        if(armRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posIntake - ArmConstants.rotateoffset && 
           armRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posIntake + ArmConstants.rotateoffset){
            return true;
        }else{
            return false;
        }
    }
}