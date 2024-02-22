// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroup;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;

public class CloseScore extends Command {
  /** Creates a new CloseScore. */
  public double shootSpeed;
  public double pivotGoal;
  public Timer scoreTimer;

  public CloseScore() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivotSubsystem);
    shootSpeed = ManipulatorConstants.SCORE_HIGH_SPEED;
    pivotGoal = ManipulatorConstants.PIVOT_CLOSE_SCORE;
    scoreTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.pivotSubsystem.enable();
    scoreTimer.reset();
    scoreTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.pivotSubsystem.setGoal(pivotGoal);
    RobotContainer.shooterSubsystem.set(shootSpeed);
    if (scoreTimer.get()>=2.5 && (RobotContainer.pivotSubsystem.atGoal(pivotGoal))) {
      RobotContainer.intakeSubsystem.setFeed(ManipulatorConstants.FEEDER_MOTOR);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (scoreTimer.get() > 4){
      return true;
    } else {
      return false;
    }
  }
}
