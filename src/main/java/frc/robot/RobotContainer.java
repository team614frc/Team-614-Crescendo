// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.setXCommand;
import frc.robot.commands.manipulator.Feeder;
import frc.robot.commands.manipulator.Intake;
import frc.robot.commands.manipulator.Shooter;
import frc.robot.commands.manipulator.pivot.PivotDown;
import frc.robot.commands.manipulator.pivot.PivotPIDCommand;
import frc.robot.commands.manipulator.pivot.PivotUp;
import frc.robot.commands.vision.AlignScore;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
  public final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final static PivotSubsystem pivotSubsystem = new PivotSubsystem();
  public final static LimelightSubsystem limeSubsystem = new LimelightSubsystem();
  // The driver's controller
  static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  static CommandXboxController m_coDriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);
  // private AutoBuilder autoBuilder = new AutoBuilder(swerveDrive,
  // intakeSubsystem, pivotSubsystem);
  private final SendableChooser<Command> autoChooser;
  private double testMotorSpeeds;

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
    // limeSubsystem.enableVisionProcessing();
    // Configure the button bindings
    testMotorSpeeds = 0;
    configureButtonBindings();
    // autoChooser.addOption("Test Path", TestPath1);
    // SmartDashboard.putData(autoChooser);
    // Configure default commands
    swerveDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> swerveDrive.drive(
                getDriverLeftY(),
                getDriverLeftX(),
                getDriverRightX(),
                true, true),
            swerveDrive));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);

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

  public static double getDriverLeftX() {
    return -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband);
  }

  public static double getDriverLeftY() {
    return -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
  }

  public static double getDriverRightX() {
    return -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);

  }

  private void configureButtonBindings() {

    m_driverController.button(OIConstants.RIGHT_STICK_PRESS).whileTrue(new setXCommand());
    m_driverController.rightTrigger().whileTrue(new Shooter(ManipulatorConstants.SCORE_HIGH_SPEED));
    m_driverController.button(OIConstants.RIGHT_BUMPER).whileTrue(new Shooter(ManipulatorConstants.SCORE_MID_SPEED));
    m_driverController.button(OIConstants.LEFT_BUMPER).whileTrue(new Shooter(ManipulatorConstants.SCORE_LOW_SPEED));
    m_driverController.leftTrigger().whileTrue(new Intake(ManipulatorConstants.INTAKE_SPEED));
    m_driverController.button(OIConstants.A_BUTTON).whileTrue(new PivotDown(ManipulatorConstants.PIVOT_UP_SPEED));
    m_driverController.button(OIConstants.X_BUTTON).whileTrue(new PivotUp(ManipulatorConstants.PIVOT_DOWN_SPEED));
    m_driverController.button(OIConstants.Y_BUTTON).whileTrue(new Feeder());

    // // m_coDriverController.button(OIConstants.RIGHT_STICK_PRESS).whileTrue(new
    // // setXCommand());
    // m_coDriverController.rightTrigger().whileTrue(new Intake(IntakeConstants.SCORE_HIGH_SPEED));
    // m_coDriverController.button(OIConstants.RIGHT_BUMPER).whileTrue(new Intake(IntakeConstants.SCORE_MID_SPEED));
    // m_coDriverController.button(OIConstants.LEFT_BUMPER).whileTrue(new Intake(IntakeConstants.SCORE_LOW_SPEED));
    // m_coDriverController.leftTrigger().whileTrue(new Intake(IntakeConstants.INTAKE_SPEED));
    // m_coDriverController.button(OIConstants.A_BUTTON).onTrue(new PivotDown(IntakeConstants.PIVOT_DOWN_SPEED));
    // m_coDriverController.button(OIConstants.X_BUTTON).onTrue(new PivotUp(IntakeConstants.PIVOT_UP_SPEED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
