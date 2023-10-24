// package frc.robot.commands.Arm.Intake;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Secondary.ArmSubsystem;

// public class ArmIntakeOutCmd extends CommandBase {

//     private final ArmSubsystem armSubsystem;
//     Supplier<Boolean> button;

//     public ArmIntakeOutCmd(ArmSubsystem armSubsystem) {
//         this.armSubsystem = armSubsystem;
//         addRequirements(armSubsystem);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {
//         armSubsystem.intakeMotorL.set(Constants.ArmConstants.gOutputSpeed);
//         armSubsystem.intakeMotorR.set(-Constants.ArmConstants.gOutputSpeed);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         armSubsystem.intakeMotorL.set(0.0);
//         armSubsystem.intakeMotorR.set(0.0);
//         RobotContainer.secondaryJoystick.setRumble(RumbleType.kRightRumble, 0);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }