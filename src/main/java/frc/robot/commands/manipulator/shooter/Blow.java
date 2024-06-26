// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LeafBlowerSubsytem;

public class Blow extends Command {
  private final LeafBlowerSubsytem blower;

  public Blow() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.leafBlowerSubsystem);
    blower = RobotContainer.leafBlowerSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    blower.blow(ManipulatorConstants.LEAF_BLOWER_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    blower.blow(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
