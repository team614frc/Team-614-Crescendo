// //package frc.robot.utils;
// package frc.robot.commands.autonomous.PathPlannerCommands;   

// import java.util.HashMap;

// import com.pathplanner.lib.PathPlannerTrajectory;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
// import frc.robot.commands.intakeCommands.Intake;
// //import frc.robot.RobotManager;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.PivotSubsystem;

// /** This utility class is built for selecting made autos */
// public class AutoBuilder {
//   DriveSubsystem drivetrain;
//   PivotSubsystem pivot;
//   IntakeSubsystem intake;
 

//   private final SendableChooser<Command> chooser = new SendableChooser<>();

//   // ====================================================================
//   //                          Trajectories
//   // ====================================================================

//   PathPlannerTrajectory dummyPath = PathPlannerBase.getTrajectory("StraightLine", true);
//   PathPlannerTrajectory dummyDonut = PathPlannerBase.getTrajectory("DummyPathDonut", true);
//   PathPlannerTrajectory figure8 = PathPlannerBase.getTrajectory("FigureThis", true);
//   PathPlannerTrajectory score2Path = PathPlannerBase.getTrajectory("OutNBack", true);
//   PathPlannerTrajectory score1NoCable = PathPlannerBase.getTrajectory("DriveBackNoCable", true);
//   PathPlannerTrajectory score1Cable = PathPlannerBase.getTrajectory("DriveBackCable", true);


//   // ====================================================================
//   //                          Routines
//   // ====================================================================

//  /* private Command dummyPathOne() {
//     return PathPlannerBase.followTrajectoryCommand(dummyPath, true);
//   } */

//  /*  private Command dummyPathDonut() {
//     return PathPlannerBase.followTrajectoryCommand(dummyDonut, true);
//   } */

//  /*  private Command figureEight() {
//     return PathPlannerBase.followTrajectoryCommand(figure8, true);
//   } */

//   private Command score1HighBackNoCable(){
//     return new SequentialCommandGroup(
//       new Intake(Constants.IntakeConstants.SCORE_HIGH_SPEED).withTimeout(0.5),
//       new WaitCommand(.5),
//       PathPlannerBase.generateAuto(score1NoCable)
//     );
//   }

//   private Command score1HighBackCable(){
//     return new SequentialCommandGroup(
//      // new Intake(Constants.IntakeConstants.SCORE_HIGH_SPEED).withTimeout(0.5),
//       new WaitCommand(.5),
//       PathPlannerBase.generateAuto(dummyPath)
//     );
//   }

//   private Command score1MidBackNoCable(){
//     return new SequentialCommandGroup(
//       new Intake(Constants.IntakeConstants.SCORE_MID_SPEED).withTimeout(0.5),
//       new WaitCommand(.5),
//       PathPlannerBase.generateAuto(score1NoCable)
//     );
//   }

//   private Command score1MidBackCable(){
//     return new SequentialCommandGroup(
//      // new Intake(Constants.IntakeConstants.SCORE_MID_SPEED).withTimeout(0.5),
//       new WaitCommand(.5),
//       PathPlannerBase.generateAuto(score1Cable)
//     );
//   }

//   private Command score1LowBackNoCable(){
//     return new SequentialCommandGroup(
//       new Intake(Constants.IntakeConstants.SCORE_MID_SPEED).withTimeout(0.5),
//       new WaitCommand(.5),
//       PathPlannerBase.generateAuto(score1NoCable)
//     );
//   }

//   private Command score1LowBackCable(){
//     return new SequentialCommandGroup(
//       new Intake(Constants.IntakeConstants.SCORE_MID_SPEED).withTimeout(0.5),
//       new WaitCommand(.5),
//       PathPlannerBase.generateAuto(score1Cable)
//     );
//   }

//   // public Command ScoreOne(){
//   //   new InstantCommand(()->manager.setScoringHeightHigh());
//   //   return new SequentialCommandGroup(
//   //       new InstantCommand(()->manuiplator.setTargetPosition(manager.getScoringHeight(), manuiplator)),
//   //       new WaitCommand(.75),
//   //       extension.driveUntil(60, false),
//   //       new RunCommand(()-> intake.runIntake(-.6), intake).withTimeout(0.5),
//   //       new InstantCommand(()->intake.runIntake(0)),
//   //       returnManipulator()
//   //   );
//   // }
//   // public Command ScoreOneMid(){
//   //   new InstantCommand(()->manager.setScoringHeightMid());
//   //   return new SequentialCommandGroup(
//   //       new InstantCommand(()->manuiplator.setTargetPosition(manager.getScoringHeight(), manuiplator)),
//   //       new WaitCommand(.75),
//   //       extension.driveUntil(60, false),
//   //       new RunCommand(()-> intake.runIntake(-.6), intake).withTimeout(0.5),
//   //       new InstantCommand(()->intake.runIntake(0)),
//   //       returnManipulator()
//   //   );
//   // }

//   private Command nonCableSide2Pc(){
//     HashMap<String, Command> eventMap = new HashMap<>();
//     eventMap.put("pickUp", new RunCommand(()->intake.set(Constants.IntakeConstants.INTAKE_SPEED), intake));
//     eventMap.put("stopPickup", new RunCommand(()->intake.set(0.0)));
//     return new SequentialCommandGroup(
//       score1HighBackNoCable(),
//         PathPlannerBase.generateAuto(eventMap, score2Path),
//        score1MidBackCable()

//     );
//   }

//   public AutoBuilder(DriveSubsystem drivetrain, IntakeSubsystem intake, PivotSubsystem pivot) {
//     this.drivetrain = drivetrain;
//     this.intake = intake;
//     this.pivot = pivot;

//     /* chooser.addOption("Dummy 1", dummyPathOne());
//     chooser.addOption("Dummy Donut", dummyPathDonut());
//     chooser.addOption("8's HEHEHEHE", figureEight());
//     */
//     chooser.addOption("Score Two NonCable", nonCableSide2Pc());
//     chooser.addOption("Score One Mid Cable",score1MidBackCable());
//     chooser.addOption("Score One High Cable", score1HighBackCable());


//     chooser.setDefaultOption("Score One High no Cake", score1HighBackNoCable());
//     SmartDashboard.putData("Auto Selector", chooser);
//   }

//   /**
//    * @return Returns chosen auto on Smartdashboard
//    */
//   public Command getSelectedAuto() {
//     return chooser.getSelected();
//   }

//   // ====================================================================
//   //                          Helpers
//   // ====================================================================

//   // private Command returnManipulator(){
//   //   return new SequentialCommandGroup( 
//   //       extension.driveUntil(1, true),
//   //       new WaitCommand(.1),
//   //       new InstantCommand(()->manuiplator.setTargetPosition(Constants.Manuiplator.kGroundPosition, manuiplator))
       

//   //   );
//   }


