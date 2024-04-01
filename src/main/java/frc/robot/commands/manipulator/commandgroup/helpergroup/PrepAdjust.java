package frc.robot.commands.manipulator.commandgroup.helpergroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.manipulator.pivot.PivotAdjust;
import frc.robot.commands.manipulator.shooter.Shooter;

public class PrepAdjust extends ParallelCommandGroup {
  public PrepAdjust() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PivotAdjust(), new Shooter(ManipulatorConstants.SCORE_SIMPLE_RPM));
  }
}
