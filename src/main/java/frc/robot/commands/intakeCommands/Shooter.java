// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

/**
 * The Shooter Command simply uses the ShooterSubsystem
 * to set the Shooter rollers to a specific value.
 * Also sets the shooter to a rest speed in order to hold the game piece
 * when there is no input on the shooter.
 * -
 * @param shooterSpeed,RobotContainer.shooterSubsystem this is the value that the intake will get set to
 */

public class Shooter extends Command {
  
  public double shooterSpeed;

  /** Creates a new Intake. */
  public Shooter(double shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
    this.shooterSpeed = shooterSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooterSubsystem.set(shooterSpeed);
    RobotContainer.shooterSubsystem.getSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.set(ShooterConstants.SHOOTER_REST_SPEED);
    RobotContainer.shooterSubsystem.getSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
