// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.intakeCommands.Shooter;
import frc.robot.commands.intakeCommands.PivotDown;
import frc.robot.commands.intakeCommands.PivotUp;
import frc.robot.commands.setXCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  public final static DriveSubsystem swerveDrive = new DriveSubsystem();
  public final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final static PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_coDriverController = new CommandXboxController(OIConstants.kCoDriverControllerPort);

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

    //Commands for pathplanner to use in autos.
    NamedCommands.registerCommand("Score High", new Shooter(ShooterConstants.SCORE_HIGH_SPEED).withTimeout(0.5));
    NamedCommands.registerCommand("Score Middle", new Shooter(ShooterConstants.SCORE_MID_SPEED).withTimeout(0.5));
    NamedCommands.registerCommand("Score Low", new Shooter(ShooterConstants.SCORE_LOW_SPEED).withTimeout(0.5));
    NamedCommands.registerCommand("Pivot Down", new PivotDown(ShooterConstants.PIVOT_DOWN_SPEED));
    NamedCommands.registerCommand("Pivot Up", new PivotUp(ShooterConstants.PIVOT_UP_SPEED));
    NamedCommands.registerCommand("Long Intake", new Shooter(ShooterConstants.SHOOTER_SPEED).withTimeout(2));
    NamedCommands.registerCommand("Short Intake", new Shooter(ShooterConstants.SHOOTER_SPEED).withTimeout(0.5));

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    // read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    // Configure the button bindings
    configureButtonBindings();
    // autoChooser.addOption("Test Path", TestPath1);
    
    SmartDashboard.putNumber("Pivot Motor Height", pivotSubsystem.getPivotMotorHeight());
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
  private void configureButtonBindings() {
    m_driverController.button(OIConstants.RIGHT_STICK_PRESS).whileTrue(new setXCommand());
    m_driverController.rightTrigger().whileTrue(new Shooter(ShooterConstants.SCORE_HIGH_SPEED));
    m_driverController.button(OIConstants.RIGHT_BUMPER).whileTrue(new Shooter(ShooterConstants.SCORE_MID_SPEED));
    m_driverController.button(OIConstants.LEFT_BUMPER).whileTrue(new Shooter(ShooterConstants.SCORE_LOW_SPEED));
    m_driverController.leftTrigger().whileTrue(new Shooter(ShooterConstants.SHOOTER_SPEED));
    m_driverController.button(OIConstants.A_BUTTON).onTrue(new PivotDown(ShooterConstants.PIVOT_DOWN_SPEED));
    m_driverController.button(OIConstants.X_BUTTON).onTrue(new PivotUp(ShooterConstants.PIVOT_UP_SPEED));

    // m_coDriverController.button(OIConstants.RIGHT_STICK_PRESS).whileTrue(new
    // setXCommand());
    m_coDriverController.button(OIConstants.RIGHT_STICK_PRESS).whileTrue(new setXCommand());
    m_coDriverController.rightTrigger().whileTrue(new Shooter(ShooterConstants.SCORE_HIGH_SPEED));
    m_coDriverController.button(OIConstants.RIGHT_BUMPER).whileTrue(new Shooter(ShooterConstants.SCORE_MID_SPEED));
    m_coDriverController.button(OIConstants.LEFT_BUMPER).whileTrue(new Shooter(ShooterConstants.SCORE_LOW_SPEED));
    m_coDriverController.leftTrigger().whileTrue(new Shooter(ShooterConstants.SHOOTER_SPEED));
    m_coDriverController.button(OIConstants.A_BUTTON).onTrue(new PivotDown(ShooterConstants.PIVOT_DOWN_SPEED));
    m_coDriverController.button(OIConstants.X_BUTTON).onTrue(new PivotUp(ShooterConstants.PIVOT_UP_SPEED));
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
