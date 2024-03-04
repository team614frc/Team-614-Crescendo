// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

public class AlignScore extends Command {
  /** Creates a new alignScore. */
  private double angle, turn, rightX;
  private boolean picker;

  public AlignScore() {
    addRequirements(RobotContainer.swerveDrive);
    picker = false;
  }

  public AlignScore (double set) {
    addRequirements(RobotContainer.swerveDrive);
    turn = set;
    picker = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // angle = (RobotContainer.limeSubsystem.getHorizontalAngle()-(Math.PI/2)) + RobotContainer.swerveDrive.getHeading().getRadians();
    if (picker) {
      if (RobotContainer.swerveDrive.getHeading().getDegrees()>0) {
        angle = ((RobotContainer.swerveDrive.getHeading().getDegrees()%360) - turn);
      } else  {
        turn = -turn;
        angle = ((RobotContainer.swerveDrive.getHeading().getDegrees()%360) - turn);
      }
    }   else    {
        angle = RobotContainer.limeSubsystem.getHorizontalAngle();
    }

    if (Math.abs(angle)<20 && turn > 0) {
      rightX = 0.7 * angle / 100.0;
    } else if (turn < 0) {
      rightX = 0.25;
    }

    SmartDashboard.putNumber("HEADING", RobotContainer.swerveDrive.getHeading().getDegrees());

    if (angle < -VisionConstants.threshold || angle > VisionConstants.threshold){
      RobotContainer.swerveDrive.drive(
        (.5)*Math.pow(RobotContainer.getDriverLeftY(), 5) + (.5)*RobotContainer.getDriverLeftY(),
        (.5)*Math.pow(RobotContainer.getDriverLeftX(), 5) + (.5)*RobotContainer.getDriverLeftX(),
        -(rightX), //VisionConstants.simpleAlignYInput * angle / 100.0
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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
