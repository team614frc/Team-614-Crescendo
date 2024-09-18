// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

/** Creates a new SpinMove. */
public class SpinMove extends Command {

  private double angle, target, turn;

  public SpinMove() {
    addRequirements(RobotContainer.limeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = RobotContainer.swerveDrive.getHeading().getDegrees() - 180;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = RobotContainer.swerveDrive.getDisplacementToTarget(target);
    turn = -angle / 180.0;

    if (Math.abs(angle) <= VisionConstants.ALIGN_THRESHOLD) {
      turn = RobotContainer.getDriverRightX();
    } else {
      RobotContainer.swerveDrive.drive(
          RobotContainer.getDriverLeftY(), RobotContainer.getDriverLeftX(), turn, true, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
