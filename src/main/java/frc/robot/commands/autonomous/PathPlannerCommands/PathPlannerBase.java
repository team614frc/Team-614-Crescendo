// package frc.robot.commands.autonomous.PathPlannerCommands;   

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.RobotContainer;

// import frc.robot.subsystems.DriveSubsystem;

// import java.util.HashMap;

// /** Util class for all Path Planner auto builders/generators */
// public class PathPlannerBase {

//   static final DriveSubsystem drivetrain = RobotContainer.swerveDrive;
//   static final PathConstraints constraints = new PathConstraints(4,3 );

//   /**
//    * Generates a usable pathplanner trajectory
//    *
//    * @param plannerFile Name of the file without file type
//    * @param reversed is the robot reversed?
//    * @return Returns path planner trajectory for routine
//    */
//   public static PathPlannerTrajectory getTrajectory(String plannerFile, boolean reversed) {
//     PathPlannerTrajectory trajectoryPath;
//     trajectoryPath =
//         PathPlanner.loadPath(
//             plannerFile,
//             constraints,
//             reversed); // Filesystem.getDeployDirectory().toPath().resolve(plannerFile);

//     return trajectoryPath;
//   }

//   /**
//    * Creates a follow only drive command
//    *
//    * @param traj Pathplanner trajectory
//    * @param isFirstPath Is it the first path of the routine, if not it will find relative position
//    *     to start
//    * @return Returns command for only following the path no events
//    */
//   public static Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
//     return new SequentialCommandGroup(
//         new InstantCommand(
//             () -> {
//               // Reset odometry for the first path you run during auto
//               if (isFirstPath) {
//                 drivetrain.resetOdometry(traj.getInitialHolonomicPose());
//               }
//             }),
//         new PPSwerveControllerCommand(
//             traj,
//             drivetrain::getPose, // Pose supplier
//             DriveConstants.kDriveKinematics, // SwerveDriveKinematics
//             new PIDController(
//                 5, 0,
//                 0), // X controller. Tune these values for your robot. Leaving them 0 will only use
//             // feedforwards.
//             new PIDController(5, 0, 0), // Y controller (usually the same values as X controller)
//             new PIDController(
//                 3, 0,
//                 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
//             // only use feedforwards.
//             drivetrain::setModuleStates, // Module states consumer
//             true, // Should the path be automatically mirrored depending on alliance color.
//             // Optional, defaults to true
//             drivetrain // Requires this drive subsystem
//             ));
//   }

//   /**
//    * Generates an auto routine
//    *
//    * @param traj pathplanner path
//    * @return Returns auto routine with no events
//    */
//   public static Command generateAuto(PathPlannerTrajectory traj) {
//     SwerveAutoBuilder autoBuilder =
//         new SwerveAutoBuilder(
//             drivetrain::getPose,
//             drivetrain::resetOdometry,
//             DriveConstants.kDriveKinematics,
//             new PIDConstants(0.0, 0.0, 0.0),
//             new PIDConstants(0.0, 0.0, 0.0),
//             drivetrain::setModuleStates,
//             new HashMap<String, Command>(),
//             true);
//     return autoBuilder.fullAuto(traj);
//   }

//   /**
//    * Generates an auto routine
//    *
//    * @param eventMap Event map of all the events during the routine
//    * @param traj Pathplanner path
//    * @return returns an auto routine with events
//    */
//   public static Command generateAuto(
//       HashMap<String, Command> eventMap, PathPlannerTrajectory traj) {

//     SwerveAutoBuilder autoBuilder =
//         new SwerveAutoBuilder(
//             drivetrain::getPose,
//             drivetrain::resetOdometry,
//             DriveConstants.kDriveKinematics,
//             new PIDConstants(5.0, 0.0, 0.0),
//             new PIDConstants(0.5, 0.0, 0.0),
//             drivetrain::setModuleStates,
//             eventMap,
//             false);
//     return autoBuilder.fullAuto(traj);
//   }

//   /**
//    * Generates an auto routine
//    *
//    * @param eventMap Event map of all the events during the routine
//    * @param traj Pathplanner path
//    * @param stopEvent Stop event with actions the for the robot to perform
//    * @return returns an auto routine with events and stop events
//    */
//   public static Command generateAuto(
//       HashMap<String, Command> eventMap, PathPlannerTrajectory traj, StopEvent stopEvent) {

//     SwerveAutoBuilder autoBuilder =
//         new SwerveAutoBuilder(
//             drivetrain::getPose,
//             drivetrain::resetOdometry,
//             DriveConstants.kDriveKinematics,
//             new PIDConstants(5.0, 0.0, 0.0),
//             new PIDConstants(0.5, 0.0, 0.0),
//             drivetrain::setModuleStates,
//             eventMap,
//             true);
//     autoBuilder.stopEventGroup(stopEvent);
//     return autoBuilder.fullAuto(traj);
//   }
// }
