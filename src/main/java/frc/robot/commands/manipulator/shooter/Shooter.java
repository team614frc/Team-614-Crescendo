// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.TimeConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shooter extends Command {

  public double shooterSpeed;
  ShooterSubsystem sub;

  /** Creates a new shooter. */
  public Shooter(double shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
    sub = RobotContainer.shooterSubsystem;
    this.shooterSpeed = shooterSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sub.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sub.setSetpoint(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sub.atGoal();
  }
}