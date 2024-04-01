// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.commandgroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.manipulator.commandgroup.helpergroup.ShootPrep;
import frc.robot.commands.manipulator.feeder.ShooterFeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFirstShot extends ParallelCommandGroup {
  /** Creates a new AutoFirstShot. */
  public AutoFirstShot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShootPrep(
            ManipulatorConstants.PIVOT_CLOSE_SCORE,
            ManipulatorConstants.SCORE_SIMPLE_RPM,
            ManipulatorConstants.PIVOT_SHOOTER_THRESHOLD),
        new ShooterFeed());
  }
}
