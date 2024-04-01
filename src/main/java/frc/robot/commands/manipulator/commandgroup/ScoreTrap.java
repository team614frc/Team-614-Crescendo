// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.commandgroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.manipulator.shooter.Blow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTrap extends ParallelCommandGroup {
    /** Creates a new PukeHelper. */
    public ScoreTrap() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new SimpleScoreTest(), // new
                                       // SimpleScoreNote(ManipulatorConstants.PIVOT_TRAP_SCORE,ManipulatorConstants.TRAP_SPEED,ManipulatorConstants.PIVOT_SHOOTER_THRESHOLD),
                new Blow());
    }
}
