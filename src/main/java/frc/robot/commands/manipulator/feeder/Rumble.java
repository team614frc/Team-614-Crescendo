package frc.robot.commands.manipulator.feeder;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotContainer;

/**
 * Rumble command rumbles the controller when the TimeofFlight sensor detects 
 * a note and while the 
 * IntakeNote command is running
 */

public class Rumble extends Command {
  
  /** Creates a new Rumble. */
  public Rumble() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      RobotContainer.getDriverController().getHID().setRumble(RumbleType.kRightRumble, ManipulatorConstants.RUMBLE_SETTING);
      RobotContainer.getDriverController().getHID().setRumble(RumbleType.kLeftRumble, ManipulatorConstants.RUMBLE_SETTING);
      RobotContainer.getCoDriverController().getHID().setRumble(RumbleType.kRightRumble, ManipulatorConstants.RUMBLE_SETTING);
      RobotContainer.getCoDriverController().getHID().setRumble(RumbleType.kLeftRumble, ManipulatorConstants.RUMBLE_SETTING);
      RobotContainer.limeSubsystem.blinkLEDs();
      RobotContainer.ledSubsystem.turnGreen();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotContainer.getDriverController().getHID().setRumble(RumbleType.kRightRumble, 0);
      RobotContainer.getDriverController().getHID().setRumble(RumbleType.kLeftRumble, 0);
      RobotContainer.getCoDriverController().getHID().setRumble(RumbleType.kRightRumble, 0);
      RobotContainer.getCoDriverController().getHID().setRumble(RumbleType.kLeftRumble, 0);
      RobotContainer.limeSubsystem.turnOffLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.feederSubsystem.isSensorTripped();
  }
}
