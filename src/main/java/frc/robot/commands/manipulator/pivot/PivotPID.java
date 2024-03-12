package frc.robot.commands.manipulator.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;

public class PivotPID extends Command {
  /** Creates a new TiltPIDCommand. */
  public double setpoint;
  private boolean test = false;

  public PivotPID() {
    addRequirements(RobotContainer.pivotSubsystem);
    test = true;
  }

  public PivotPID(double set) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivotSubsystem);
    test = false;
    setpoint = set;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.pivotSubsystem.enable();
    if (test)
      setpoint = SmartDashboard.getNumber("Test Pivot", ManipulatorConstants.PIVOT_MAX);
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
    return RobotContainer.pivotSubsystem.atGoal(setpoint);
  }
}