// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class AlignScore extends Command {
  /** Creates a new AlignScore. */
  private double angle, target, turn;
  private boolean isTarget;

  public AlignScore() {
    addRequirements(RobotContainer.swerveDrive);
    isTarget = false;
  }

  public AlignScore(double target) {
    addRequirements(RobotContainer.swerveDrive);
    this.target = target;
    isTarget = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = isTarget ? RobotContainer.swerveDrive.getDisplacementToTarget(target)
        : RobotContainer.limeSubsystem.getAngleOffset();

    turn = -VisionConstants.simpleAlignYInput * angle / 100.0;

    if (turn > 0.2) {
      turn = 0.35;
    } else if (turn < -0.2) {
      turn = -0.35;
    }

    if (Math.abs(angle) > VisionConstants.threshold) {
      RobotContainer.swerveDrive.drive(
          RobotContainer.getDriverLeftY(),
          RobotContainer.getDriverLeftX(),
          turn,
          true, true);
    } else {
      RobotContainer.swerveDrive.drive(
          RobotContainer.getDriverLeftY(),
          RobotContainer.getDriverLeftX(),
          RobotContainer.getDriverRightX(),
          true, true);
    }
  }
  // insert code to adjust robot angle here

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}