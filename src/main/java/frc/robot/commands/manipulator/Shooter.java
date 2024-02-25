// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;

public class Shooter extends Command {

  public double shootSpeed;
  public Timer commandTimer;

  /** Creates a new shooter. */
  public Shooter(double shootSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
    addRequirements(RobotContainer.intakeSubsystem);
    commandTimer = new Timer();
    this.shootSpeed = shootSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooterSubsystem.enable();
    commandTimer.reset();
    commandTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooterSubsystem.setSetpoint(shootSpeed);
    if (commandTimer.get() > 2) {
      RobotContainer.intakeSubsystem.setFeed(ManipulatorConstants.MOTOR_LOADING_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.set(ManipulatorConstants.MOTOR_ZERO_SPEED);
    RobotContainer.intakeSubsystem.setFeed(ManipulatorConstants.INTAKE_REST_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (commandTimer.get() > 3){
      return true;
    } else {
      return false;
    }
  }
}