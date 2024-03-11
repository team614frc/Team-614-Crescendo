package frc.robot.commands.manipulator.feeder;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * The Intake Command simply uses the IntakeSubsystem
 * to set the Intake rollers to a specific value.
 * Also sets the intake to a rest speed in order to hold the game piece
 * when there is no input on the intake.
 * -
 * @param intakeSpeed,RobotContainer.intakeSubsystem this is the value that the intake will get set to
 */

public class Rumble extends Command {
  
  public double intakeSpeed;
  public Timer commandTimer;

  /** Creates a new Intake. */
  public Rumble() {
    // Use addRequirements() here to declare subsystem dependencies.
    commandTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      RobotContainer.getDriverController().getHID().setRumble(RumbleType.kRightRumble, .4);
      RobotContainer.getDriverController().getHID().setRumble(RumbleType.kLeftRumble, .4);
      RobotContainer.getCoDriverController().getHID().setRumble(RumbleType.kRightRumble, .4);
      RobotContainer.getCoDriverController().getHID().setRumble(RumbleType.kLeftRumble, .4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotContainer.getDriverController().getHID().setRumble(RumbleType.kRightRumble, 0);
      RobotContainer.getDriverController().getHID().setRumble(RumbleType.kLeftRumble, 0);
      RobotContainer.getCoDriverController().getHID().setRumble(RumbleType.kRightRumble, 0);
      RobotContainer.getCoDriverController().getHID().setRumble(RumbleType.kLeftRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandTimer.get() > 2;
  }
}
