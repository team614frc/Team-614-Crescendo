// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;

public class Feeder extends Command {

  public double feedSpeed;

  /** Creates a new shooter. */
  public Feeder(double feedSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeSubsystem);
    addRequirements(RobotContainer.shooterSubsystem);
    this.feedSpeed = feedSpeed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooterSubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intakeSubsystem.setFeed(feedSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setFeed(ManipulatorConstants.INTAKE_REST_SPEED);
    RobotContainer.shooterSubsystem.setSetpoint(ManipulatorConstants.INTAKE_REST_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.intakeSubsystem.isSensorTripped();
  }
}