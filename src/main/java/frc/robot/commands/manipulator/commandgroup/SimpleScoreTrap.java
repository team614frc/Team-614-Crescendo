// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.commandgroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.manipulator.shooter.Blow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleScoreTrap extends ParallelCommandGroup {
  /** Creates a new SimpleScoreTrap. */
  public SimpleScoreTrap() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SimpleScoreNote(
            ManipulatorConstants.PIVOT_MIN,
            ManipulatorConstants.TRAP_SPEED,
            ManipulatorConstants.PIVOT_INTAKE_THRESHOLD),
        // new SimpleScoreTest(),
        new Blow());
  }
}
