// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.commandgroup.helpergroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.manipulator.feeder.SimpleFeed;
import frc.robot.commands.manipulator.intake.SimpleIntake;
import frc.robot.commands.manipulator.pivot.PivotPID;
import frc.robot.commands.manipulator.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EmptyStomach extends ParallelCommandGroup {
  /** Creates a new PukeHelper. */
  public EmptyStomach() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SimpleFeed(-ManipulatorConstants.INTAKE_SPEED),
      new SimpleIntake(-ManipulatorConstants.INTAKE_SPEED),
      new Shooter(-ManipulatorConstants.SCORE_LOW_SPEED),
      new PivotPID(ManipulatorConstants.PIVOT_AMP_GOAL)
    );
  }
}
