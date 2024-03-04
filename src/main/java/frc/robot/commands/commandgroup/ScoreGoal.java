// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandgroup;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.TimeConstants;

public class ScoreGoal extends Command {
  /** Creates a new CloseScore. */
  public double shootSpeed;
  public double pivotGoal;
  public Timer scoreTimer;

  public ScoreGoal(double pivotGoal) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivotSubsystem);
    addRequirements(RobotContainer.shooterSubsystem);
    addRequirements(RobotContainer.intakeSubsystem);
    addRequirements(RobotContainer.limeSubsystem);
    shootSpeed = ManipulatorConstants.SCORE_SIMPLE;
    this.pivotGoal = pivotGoal;
    scoreTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.pivotSubsystem.enable();
    scoreTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.pivotSubsystem.setGoal(pivotGoal);
    RobotContainer.shooterSubsystem.set(shootSpeed);
    if (scoreTimer.get() >= TimeConstants.SpeakerFeed) {
      RobotContainer.intakeSubsystem.setFeed(ManipulatorConstants.LOADING_SPEED);
      RobotContainer.limeSubsystem.turnOffLEDs();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.set(0);
    RobotContainer.intakeSubsystem.setFeed(0);
    RobotContainer.pivotSubsystem.setGoal(ManipulatorConstants.PIVOT_MIN);
    scoreTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return scoreTimer.get() > TimeConstants.SpeakerEnd;
  }
}
