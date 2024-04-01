// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotContainer;

/**
 * The Pivot Command uses the PivotSubsystem in order to set a specific value to the Pivot for the
 * intake of the robot -
 *
 * @param pivotSpeed,RobotContainer.pivotSubsystem takes in the speed that the pivot is supposed to
 *     be set to
 */
public class PivotUp extends Command {

  private double pivotSpeed;

  /** Creates a new Pivot. */
  public PivotUp(double pivotSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivotSubsystem);
    this.pivotSpeed = pivotSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.pivotSubsystem.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(RobotContainer.pivotSubsystem.getPivotLEncoder())
        < ManipulatorConstants.PIVOT_MAX) {
      RobotContainer.pivotSubsystem.set(pivotSpeed);
      SmartDashboard.putNumber(
          "Encoder Position in Command", RobotContainer.pivotSubsystem.getPivotLEncoder());
    } else {
      RobotContainer.pivotSubsystem.set(ManipulatorConstants.MOTOR_ZERO_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.pivotSubsystem.set(ManipulatorConstants.MOTOR_ZERO_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
