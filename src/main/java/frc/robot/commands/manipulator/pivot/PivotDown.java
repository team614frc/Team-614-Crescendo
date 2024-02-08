// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Pivot Command uses the PivotSubsystem in order
 * to set a specific value to the Pivot for the intake
 * of the robot
 * -
 * 
 * @param pivotSpeed,RobotContainer.pivotSubsystem takes in the speed that the
 *                                                 pivot is supposed to be set
 *                                                 to
 */

public class PivotDown extends Command {

  private double pivotSpeed;

  /** Creates a new Pivot. */
  public PivotDown(double pivotSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivotSubsystem);
    this.pivotSpeed = pivotSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(RobotContainer.pivotSubsystem.getPivotMotorLHeight()) > 0 ) { //18 = pivot min height i think
      RobotContainer.pivotSubsystem.set(-pivotSpeed);
      SmartDashboard.putNumber("Encoder Position in Command",
      RobotContainer.pivotSubsystem.getPivotMotorLHeight());
    } else {
      RobotContainer.pivotSubsystem.set(ManipulatorConstants.MOTOR_GRAV_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.pivotSubsystem.set(ManipulatorConstants.MOTOR_GRAV_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
