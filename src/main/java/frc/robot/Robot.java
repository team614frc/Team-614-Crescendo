// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.VisionConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    SmartDashboard.putNumber("Test Pivot", ManipulatorConstants.PIVOT_MAX);
    SmartDashboard.putNumber("Shooter Test", 5000);
    RobotContainer.ledSubsystem.turnOrange();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.putNumber("GYRO", RobotContainer.swerveDrive.getHeading().getDegrees());
    SmartDashboard.putNumber("ROBOT POSE ROTATION", RobotContainer.swerveDrive.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("2D Range", RobotContainer.limeSubsystem.estimateDistance());
    CommandScheduler.getInstance().run();
    RobotContainer.limeSubsystem.estimateDistance();
    if (DriverStation.getAlliance().isPresent()) {
      RobotContainer.setAlliance(DriverStation.getAlliance().get());
    }
    SmartDashboard.putString("DRIVERSTATION", DriverStation.getAlliance().get().toString());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.limeSubsystem.turnOffLEDs();
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.ledSubsystem.turnOrange();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("PIVOT ANGLE", RobotContainer.pivotSubsystem.getMeasurement());
    if (Math.abs(RobotContainer.limeSubsystem.estimateDistance()) < VisionConstants.TreeMapMax
        && RobotContainer.feederSubsystem.isSensorTripped()
        && RobotContainer.limeSubsystem.estimateDistance() != 0) {
      RobotContainer.ledSubsystem.rainbow();
    } else if (RobotContainer.feederSubsystem.isSensorTripped()) {
      RobotContainer.ledSubsystem.turnGreen();
    } else {
      RobotContainer.ledSubsystem.turnOrange();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
