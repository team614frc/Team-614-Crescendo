// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

import com.playingwithfusion.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.RobotContainer;


/**
 * The IntakeSubsystem contains all the motors for the intake
 * of the robot and sets them a value that is passed to it
 * using a command
 * -
 * @param intakeSpeed Variable indicates the speed passed by the commans
 * that the intake motors should be set to
 * @returns through the getSpeed() returns the speed that the intake is going at
 */

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  private CANSparkFlex feedMotor;
  private CANSparkFlex intakeMotor;
  private TimeOfFlight sensor;

  public IntakeSubsystem() {
    // Creates a new motor

    feedMotor = new CANSparkFlex(ManipulatorConstants.FEEDER_MOTOR, MotorType.kBrushless);
    // feedMotor.restoreFactoryDefaults();
    feedMotor.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    feedMotor.setInverted(false);
    feedMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    feedMotor.burnFlash(); 

    intakeMotor = new CANSparkFlex(ManipulatorConstants.INTAKE_MOTOR, MotorType.kBrushless);
    // intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    intakeMotor.setInverted(false);
    intakeMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    intakeMotor.burnFlash(); 

    sensor = new TimeOfFlight(SensorConstants.sensorPort1);
    sensor.setRangeOfInterest(1000, 1000, 1000, 1000);

    // intakeMotorL.setInverted(false);
    // intakeMotorR.setInverted(true);
    //intakeMotorL.follow(intakeMotorR); // Sets the left motor to be the follow of the right intake motor
    SmartDashboard.putData("IntakeSubsystem", this);
  }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getSpeed() {
    return intakeMotor.get();
  }

  public double getSensorRange() {
    return sensor.getRange();
  }

  // Sets the value of the motor to a double, at which the motor will run
  public void setIntake(double intakeSpeed) {
    intakeMotor.set(-intakeSpeed);
  }

  public void setFeed (double feedSpeed) {
    feedMotor.set(-feedSpeed);
  }

  public boolean isSensorTripped() {
    return RobotContainer.intakeSubsystem.getSensorRange() < SensorConstants.sensorThreshold;
  }
}
