// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.commandgroup;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.drivetrain.vision.AlignScore;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlignScore extends ParallelDeadlineGroup {
  /** Creates a new AutoAlignScore. */
  public AutoAlignScore() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(
      new AutoScore(ManipulatorConstants.PIVOT_FAR_SCORE)
      );
    addCommands(new AlignScore());
  }
}
