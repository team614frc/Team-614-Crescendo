// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.drivetrain.vision;

// import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import frc.robot.RobotContainer;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TurnPID extends ProfiledPIDCommand {
//   /** Creates a new turnPID. */
//   public TurnPID(double angle) {
//     super(
//         // The ProfiledPIDController used by the command
//         RobotContainer.swerveDrive.getTurnController(),
//         // This should return the measurement
//         () -> RobotContainer.swerveDrive.getHeading().getDegrees(),
//         // This should return the goal (can also be a constant)
//         () -> RobotContainer.swerveDrive.getCorrectAngleTarget(angle),
//         // This uses the output
//         (output, setpoint) -> {
//           double feed = RobotContainer.swerveDrive.getTurnFF().calculate(setpoint.position, setpoint.velocity);
//           RobotContainer.swerveDrive.turnToAngle((output + RobotContainer.swerveDrive.getTurnController().calculate(
//             RobotContainer.swerveDrive.getHeading().getDegrees() + feed)));
//         });
//     addRequirements(RobotContainer.swerveDrive);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
