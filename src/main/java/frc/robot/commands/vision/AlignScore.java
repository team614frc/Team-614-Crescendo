// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class AlignScore extends Command {
  /** Creates a new alignScore. */
  private double angle;

  public AlignScore() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // angle = (RobotContainer.limeSubsystem.getHorizontalAngle()-(Math.PI/2)) + RobotContainer.swerveDrive.getHeading().getRadians();
    angle = RobotContainer.limeSubsystem.getHorizontalAngle();
    if (angle < -VisionConstants.threshold || angle > VisionConstants.threshold){
      RobotContainer.swerveDrive.drive(
        (.5)*Math.pow(RobotContainer.getDriverLeftY(), 5) + (.5)*RobotContainer.getDriverLeftY(),
        (.5)*Math.pow(RobotContainer.getDriverLeftX(), 5) + (.5)*RobotContainer.getDriverLeftX(),
        -(VisionConstants.simpleAlignYInput * angle / 100.0),
            true, true);
    } else {
      RobotContainer.swerveDrive.drive(
        (.5)*Math.pow(RobotContainer.getDriverLeftY(), 5) + (.5)*RobotContainer.getDriverLeftY(),
        (.5)*Math.pow(RobotContainer.getDriverLeftX(), 5) + (.5)*RobotContainer.getDriverLeftX(),
        (.5)*Math.pow(RobotContainer.getDriverRightX(), 5) + (.5)*RobotContainer.getDriverRightX(),
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
