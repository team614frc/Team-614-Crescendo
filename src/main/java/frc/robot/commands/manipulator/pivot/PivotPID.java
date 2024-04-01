package frc.robot.commands.manipulator.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotContainer;

public class PivotPID extends Command {
  /** Creates a new TiltPIDCommand. */
  public double setpoint;

  public double threshold;

  private boolean test = false;

  public PivotPID(double threshold) {
    addRequirements(RobotContainer.pivotSubsystem);
    test = true;
    this.threshold = threshold;
  }

  public PivotPID(double set, double threshold) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivotSubsystem);
    setpoint = set;
    this.threshold = threshold;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.pivotSubsystem.enable();
    if (test) {
      setpoint = SmartDashboard.getNumber("Test Pivot", ManipulatorConstants.PIVOT_FAR_SCORE);
    }
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
    return RobotContainer.pivotSubsystem.atGoal(setpoint, threshold);
  }
}
