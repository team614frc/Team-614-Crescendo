package frc.robot.commands.manipulator.commandgroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.manipulator.commandgroup.helpergroup.PrepAdjust;
import frc.robot.commands.manipulator.commandgroup.helpergroup.ScoreReset;
import frc.robot.commands.manipulator.feeder.ShooterFeed;

public class SimpleScoreAdjust extends SequentialCommandGroup {
  public SimpleScoreAdjust() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PrepAdjust(), new ShooterFeed(), new ScoreReset());
  }
}
