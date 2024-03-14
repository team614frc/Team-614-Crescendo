// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;

public class IntakeFeed extends Command {
  /** Creates a new intakeFeed */
  private final FeederSubsystem feeder;

  public IntakeFeed() {
    addRequirements(RobotContainer.feederSubsystem);
    feeder = RobotContainer.feederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.setFeed(ManipulatorConstants.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setFeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return feeder.isSensorTripped();
  }
}
