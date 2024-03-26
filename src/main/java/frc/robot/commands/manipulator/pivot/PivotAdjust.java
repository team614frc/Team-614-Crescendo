package frc.robot.commands.manipulator.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotAdjust extends Command {
  /** Creates a new TiltPIDCommand. */
  private double distanceFromAprilTag;
  private double setpoint;

  public PivotAdjust() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivotSubsystem);
    addRequirements(RobotContainer.limeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.pivotSubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceFromAprilTag = RobotContainer.limeSubsystem.estimateDistance();
    if(distanceFromAprilTag > 0) {
    setpoint = RobotContainer.limeSubsystem.interpolateAngle(distanceFromAprilTag);
    RobotContainer.pivotSubsystem.setGoal(setpoint);
    SmartDashboard.putNumber("End pivot", RobotContainer.pivotSubsystem.getMeasurement());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.pivotSubsystem.atGoal(setpoint, ManipulatorConstants.PIVOT_SHOOTER_THRESHOLD);
  }
}