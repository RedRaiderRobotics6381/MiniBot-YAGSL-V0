package frc.robot.commands.Arm.Rotate;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Secondary.ArmRotateSubsystem;

public class ArmRotateToDrivePosCmd extends CommandBase {
    private static final double kS = 0.0; //kS = static friction
    private static final double kG = 0.0; //kG = gravity
    private static final double kV = 0.0; //kV = velocity
    private static final double kA = 0.0; //kA = acceleration   
    private static final double kP = 0.0; //kP = proportional
    private static final double kI = 0.0; //kI = integral
    private static final double kD = 0.0; //kD = derivative
    ArmFeedforward armFeedForward = new ArmFeedforward(kS, kG, kV, kA);
    PIDController armPIDController = new PIDController(kP, kI, kD);

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
        if (ArmRotateSubsystem.armRotateEncoder.getPosition() > ArmConstants.posDrive) {
            ArmRotateSubsystem.armRotateMotor.set(-ArmConstants.rotateSpeed * P*.75);
            // System.out.println("up");
        }
        if (ArmRotateSubsystem.armRotateEncoder.getPosition() < ArmConstants.posDrive) {
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