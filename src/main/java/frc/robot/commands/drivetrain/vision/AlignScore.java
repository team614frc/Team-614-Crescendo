// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AlignScore extends Command {
  /** Creates a new alignScore. */
  private double angle, rightX, offset;
  private boolean isVisionBased;

  public AlignScore() {
    addRequirements(RobotContainer.swerveDrive);
    isVisionBased = true;
  }

  public AlignScore (double set) {
    addRequirements(RobotContainer.swerveDrive);
    angle = set;
    isVisionBased = false;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = RobotContainer.swerveDrive.getCorrectAngleTarget(angle);
    offset = isVisionBased ? 
      RobotContainer.limeSubsystem.getAngleOffset() : 
      RobotContainer.swerveDrive.getHeading().getDegrees() - angle;
    
    if (Math.abs(offset) < 10) {
      rightX = VisionConstants.simpleAlignYInput * angle / 100.0; // Formula for turn value when the distance to angle is less than 10 degrees
    } else if (Math.abs(offset) < 20) {
      rightX = offset < 0 ? -0.15 : 0.15;  // Turn value when the distance to angle is less than 20 degrees
    } else {
      rightX = offset < 0 ? -0.4 : 0.4; // Turn value when the distance to angle is greater than 20 degrees
    }

    if (Math.abs(angle) > VisionConstants.threshold){
      RobotContainer.swerveDrive.turnToAngle(rightX);
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