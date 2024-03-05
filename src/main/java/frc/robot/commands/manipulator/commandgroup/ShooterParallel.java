// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator.commandgroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.manipulator.pivot.PivotPID;
import frc.robot.commands.manipulator.shooter.ShooterPID;

public class ShooterParallel extends ParallelCommandGroup {

  public double shootSpeed, pivotGoal, feedSpeed;

  /** Creates a new shooter. */
  public ShooterParallel(double shootSpeed, double pivotGoal) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new PivotPID(pivotGoal), 
    new ShooterPID(shootSpeed));
  }

}