// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.visionCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class alignScore extends Command {
  /** Creates a new alignScore. */
  private double angle;

  public alignScore() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = RobotContainer.limeSubsystem.getHorizontalAngle();
    if (angle < -VisionConstants.threshold || angle > VisionConstants.threshold){
      RobotContainer.swerveDrive.drive(
                RobotContainer.getDriverLeftY(),
                RobotContainer.getDriverLeftX(),
                -(VisionConstants.simpleAlignYInput * (angle/100.0)),
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
    SmartDashboard.putNumber("ANGLE VALUE IN ALIGN SCORE", angle);
    return false;
  }
}