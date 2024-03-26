package frc.robot.commands.manipulator.commandgroup.helpergroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.manipulator.feeder.ShooterFeed;

public class SimpleScoreAdjust extends SequentialCommandGroup{
    public SimpleScoreAdjust() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootPrep(),
      new ShooterFeed(),
      new ScoreReset()
    );
  }

}
