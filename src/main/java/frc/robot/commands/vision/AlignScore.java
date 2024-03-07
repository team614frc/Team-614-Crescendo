// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class AlignScore extends Command {
  /** Creates a new alignScore. */
  private double angle, turn, rightX, poseAngle;
  private boolean isVisionBased;

  public AlignScore() {
    addRequirements(RobotContainer.swerveDrive);
    isVisionBased = false;
  }

  public AlignScore (double set) {
    addRequirements(RobotContainer.swerveDrive);
    turn = set;
    isVisionBased = true;
  }

  public double getVisionBasedAngle() {
    return RobotContainer.limeSubsystem.getAngleOffset();
  }

  public double getPositionBasedAngle() {
    if (RobotContainer.swerveDrive.getHeading().getDegrees() <= 0) {
      turn = -turn;
    }
    poseAngle = RobotContainer.swerveDrive.getHeading().getDegrees() % 360 - turn;
    return poseAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = isVisionBased ? getVisionBasedAngle() : getPositionBasedAngle();

    if (Math.abs(angle) < 20 && turn > 0) {
      rightX = 0.7 * angle / 100.0;
    } else if (turn < 0) {
      rightX = 0.25;
    }

    if (Math.abs(angle) > VisionConstants.threshold){
      RobotContainer.swerveDrive.drive(
        RobotContainer.getDriverLeftY(),
        RobotContainer.getDriverLeftX(),
        -(rightX), //VisionConstants.simpleAlignYInput * angle / 100.0
            true, true);
    } else {
      RobotContainer.swerveDrive.drive(
        RobotContainer.getDriverLeftY(),
        RobotContainer.getDriverLeftX(),
        RobotContainer.getDriverRightX(),
            true, true);
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
