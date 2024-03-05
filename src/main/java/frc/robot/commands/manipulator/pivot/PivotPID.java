package frc.robot.commands.manipulator.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class PivotPID extends Command {
  /** Creates a new TiltPIDCommand. */
  public double setpoint;

  public PivotPID(double set) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivotSubsystem);
    setpoint = set;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.pivotSubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.pivotSubsystem.setGoal(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}