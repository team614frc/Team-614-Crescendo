// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.SensorConstants;

/**
 * The Intake Command simply uses the IntakeSubsystem
 * to set the Intake rollers to a specific value.
 * Also sets the intake to a rest speed in order to hold the game piece
 * when there is no input on the intake.
 * -
 * @param intakeSpeed,RobotContainer.intakeSubsystem this is the value that the intake will get set to
 */

public class SimpleIntake extends Command {
  
  public double intakeSpeed;

  /** Creates a new Intake. */
  public SimpleIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeSubsystem);
    // intakeSpeed = ManipulatorConstants.INTAKE_SPEED;
    intakeSpeed = ManipulatorConstants.INTAKE_SPEED;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      RobotContainer.intakeSubsystem.setIntake(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setIntake(ManipulatorConstants.INTAKE_REST_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
