// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.feeder;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class IntakeFeed extends Command {
  /** Creates a new intakeFeed */
  private final FeederSubsystem feeder;
  private final LimelightSubsystem lime;
  public Timer commandTimer;

  public IntakeFeed() {
    addRequirements(RobotContainer.feederSubsystem);
    addRequirements(RobotContainer.limeSubsystem);
    commandTimer = new Timer();
    feeder = RobotContainer.feederSubsystem;
    lime = RobotContainer.limeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.setFeed(ManipulatorConstants.INTAKE_SPEED);
    lime.blinkLEDs();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setFeed(0);
    commandTimer.reset();
    if(commandTimer.get() > 2) {
      RobotContainer.getDriverController().getHID().setRumble(RumbleType.kRightRumble, .4);
      RobotContainer.getDriverController().getHID().setRumble(RumbleType.kLeftRumble, .4);
      RobotContainer.getCoDriverController().getHID().setRumble(RumbleType.kRightRumble, .4);
      RobotContainer.getCoDriverController().getHID().setRumble(RumbleType.kLeftRumble, .4);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return feeder.isSensorTripped();
  }
}
