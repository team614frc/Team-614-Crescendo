// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /** Creates a new TurnToAngle. */
  public TurnToAngle(double angle) {
    super(
        // The controller that the command will use
        new PIDController(0.3, 0, 0),
        // This should return the measurement
        () -> RobotContainer.swerveDrive.getHeading().getDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> RobotContainer.swerveDrive.getCorrectAngleTarget(angle),
        // This uses the output
        output -> {
          RobotContainer.swerveDrive.turnToAngle(output);
        });
    addRequirements(RobotContainer.swerveDrive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
