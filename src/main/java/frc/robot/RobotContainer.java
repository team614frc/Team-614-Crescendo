// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.setXCommand;
import frc.robot.commands.intakeCommands.Intake;
import frc.robot.commands.intakeCommands.PivotDown;
import frc.robot.commands.intakeCommands.PivotUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  public final static DriveSubsystem swerveDrive = new DriveSubsystem();
  public final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final static ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  public final static LimelightSubsystem limeSubsystem = new LimelightSubsystem();
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_coDriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);
  //private AutoBuilder autoBuilder = new AutoBuilder(swerveDrive, intakeSubsystem, pivotSubsystem);

  // SendableChooser<Command> autoChooser = new SendableChooser<>();
  // private final Command TestPath1
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Starts recording to data log
    DataLogManager.start();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    // (optional) Record only DS control data by uncommenting next line.
    // DriverStation.startDataLog(DataLogManager.getLog(), false);

    // Configure the button bindings
    configureButtonBindings();
    // autoChooser.addOption("Test Path", TestPath1);
    //SmartDashboard.putData(autoChooser);
    SmartDashboard.putNumber("Pivot Motor Height", climbSubsystem.getClimbMotorHeight());
    // Configure default commands
    swerveDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> swerveDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            swerveDrive));
  }

  

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.button(OIConstants.RIGHT_STICK_PRESS).whileTrue(new setXCommand());
    m_driverController.rightTrigger().whileTrue(new Intake(IntakeConstants.SCORE_HIGH_SPEED));
    m_driverController.button(OIConstants.RIGHT_BUMPER).whileTrue(new Intake(IntakeConstants.SCORE_MID_SPEED));
    m_driverController.button(OIConstants.LEFT_BUMPER).whileTrue(new Intake(IntakeConstants.SCORE_LOW_SPEED));
    m_driverController.leftTrigger().whileTrue(new Intake(IntakeConstants.INTAKE_SPEED));
    m_driverController.button(OIConstants.A_BUTTON).onTrue(new PivotDown(IntakeConstants.PIVOT_DOWN_SPEED));
    m_driverController.button(OIConstants.X_BUTTON).onTrue(new PivotUp(IntakeConstants.PIVOT_UP_SPEED));

    // m_coDriverController.button(OIConstants.RIGHT_STICK_PRESS).whileTrue(new
    // setXCommand());
    m_coDriverController.rightTrigger().whileTrue(new Intake(IntakeConstants.SCORE_HIGH_SPEED));
    m_coDriverController.button(OIConstants.RIGHT_BUMPER).whileTrue(new Intake(IntakeConstants.SCORE_MID_SPEED));
    m_coDriverController.button(OIConstants.LEFT_BUMPER).whileTrue(new Intake(IntakeConstants.SCORE_LOW_SPEED));
    m_coDriverController.leftTrigger().whileTrue(new Intake(IntakeConstants.INTAKE_SPEED));
    m_coDriverController.button(OIConstants.A_BUTTON).onTrue(new PivotDown(IntakeConstants.PIVOT_DOWN_SPEED));
    m_coDriverController.button(OIConstants.X_BUTTON).onTrue(new PivotUp(IntakeConstants.PIVOT_UP_SPEED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    config);

    var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0,
    AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new
    SwerveControllerCommand(
    exampleTrajectory,
    swerveDrive::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    new PIDController(AutoConstants.kPXController, 0, 0),
    new PIDController(AutoConstants.kPYController, 0, 0),
    thetaController,
    swerveDrive::setModuleStates,
    swerveDrive);

    // Reset odometry to the starting pose of the trajectory.
    swerveDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> swerveDrive.drive(0, 0, 0,
    false, false));
    }
    //return autoBuilder.getSelectedAuto();
  //}

}
