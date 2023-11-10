package frc.robot.commands.Arm.Intake;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
//import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.Secondary.ArmIntakeSubsystem;

public class ArmIntakeInCmd extends CommandBase {

    private final ArmIntakeSubsystem armSubsystem;

    public ArmIntakeInCmd(ArmIntakeSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.engineerXbox.setRumble(RumbleType.kRightRumble, 0.5);
    }

    @Override
    public void execute() {
        armSubsystem.intakeMotorL.set(Constants.ArmConstants.gIntakeSpeed);
        armSubsystem.intakeMotorR.set(Constants.ArmConstants.gIntakeSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.intakeMotorL.set(0.062);
        armSubsystem.intakeMotorR.set(0.062);
        RobotContainer.engineerXbox.setRumble(RumbleType.kRightRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}