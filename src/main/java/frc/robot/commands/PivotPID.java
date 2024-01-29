// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotPID extends PIDCommand {
  private static final double PIVOT_kP = 0.025;
  private static final double PIVOT_kI = 0;
  private static final double PIVOT_kD = 0;


  /** Creates a new PivotPID. */
  public PivotPID(double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(PIVOT_kP, PIVOT_kI, PIVOT_kD),
        // This should return the measurement
        () -> RobotContainer.pivotSubsystem.getPivotMotorHeight(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          // Use the output here
          RobotContainer.pivotSubsystem.set(-output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivotSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
