// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.commandgroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.manipulator.commandgroup.independantGroups.ScoreReset;
import frc.robot.commands.manipulator.commandgroup.independantGroups.ShootPrep;
import frc.robot.commands.manipulator.feeder.ShooterFeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleScoreNote extends SequentialCommandGroup {
  /** Creates a new SimpleScoreNote. */
  public SimpleScoreNote(double pivotGoal) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootPrep(pivotGoal),
      new ShooterFeed(),
      new ScoreReset()
    );
  }
}
