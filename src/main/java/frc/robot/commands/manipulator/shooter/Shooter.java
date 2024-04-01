// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class Shooter extends Command {
  private double shooterSpeed;
  private final ShooterSubsystem shooter;
  private boolean test = false;

  /** Creates a new shooter. */
  public Shooter() {
    addRequirements(RobotContainer.shooterSubsystem);
    shooter = RobotContainer.shooterSubsystem;
    test = true;
  }

  public Shooter(double shooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
    shooter = RobotContainer.shooterSubsystem;
    this.shooterSpeed = shooterSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.enable();
    if (test) {
      shooterSpeed = SmartDashboard.getNumber("Shooter Test", 3000);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSetpoint(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.atGoal(shooterSpeed);
  }
}
