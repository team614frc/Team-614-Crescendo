// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.RobotContainer;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
  private TimeOfFlight sensor;
  private CANSparkFlex feedMotor;

  public FeederSubsystem() {
    sensor = new TimeOfFlight(SensorConstants.sensorPort1);
    sensor.setRangeOfInterest(1000, 1000, 1000, 1000);

    feedMotor = new CANSparkFlex(ManipulatorConstants.FEEDER_MOTOR, MotorType.kBrushless);
    // feedMotor.restoreFactoryDefaults();
    feedMotor.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    feedMotor.setInverted(false);
    feedMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    feedMotor.burnFlash(); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getSensorRange() {
    return sensor.getRange();
  }

  public void setFeed (double feedSpeed) {
    feedMotor.set(-feedSpeed);
  }

  public boolean isSensorTripped() {
    return getSensorRange() < SensorConstants.sensorThreshold;
  }

}
