package frc.robot.commands.manipulator.commandgroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.manipulator.pivot.PivotPID;
import frc.robot.commands.manipulator.shooter.Feeder;
import frc.robot.commands.vision.AlignScore;

public class ShooterSequence extends SequentialCommandGroup{
    
    public ShooterSequence(double shootSpeed, double pivotGoal) {
    addCommands(
        new AlignScore(), 
        new ShooterParallel(shootSpeed, pivotGoal), 
        new Feeder(ManipulatorConstants.LOADING_SPEED), 
        new PivotPID(ManipulatorConstants.PIVOT_MIN));
    }
}