// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.TimeConstants;

public class Shooter extends Command {

  public double shootSpeed, pivotGoal;
  public Timer commandTimer;

  /** Creates a new shooter. */
  public Shooter(double shootSpeed, double pivotGoal) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
    addRequirements(RobotContainer.intakeSubsystem);
    addRequirements(RobotContainer.pivotSubsystem);
    addRequirements(RobotContainer.pivotSubsystem);
    commandTimer = new Timer();
    this.shootSpeed = shootSpeed;
    this.pivotGoal = pivotGoal;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooterSubsystem.enable();
    RobotContainer.pivotSubsystem.enable();
    commandTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooterSubsystem.setSetpoint(shootSpeed);
    RobotContainer.pivotSubsystem.setGoal(pivotGoal);
    if (RobotContainer.pivotSubsystem.atGoal(ManipulatorConstants.SCORE_SIMPLE_PID) && RobotContainer.shooterSubsystem.atGoal()) {
      RobotContainer.intakeSubsystem.setFeed(ManipulatorConstants.LOADING_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.setSetpoint(ManipulatorConstants.MOTOR_ZERO_SPEED);
    RobotContainer.intakeSubsystem.setFeed(ManipulatorConstants.INTAKE_REST_SPEED);
    RobotContainer.pivotSubsystem.setGoal(ManipulatorConstants.PIVOT_MIN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.intakeSubsystem.isSensorTripped();
  }
}