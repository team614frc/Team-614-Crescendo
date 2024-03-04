// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;

public class ShooterSubsystem extends PIDSubsystem {
  /** Creates a new ShooterSubsystem. */
  
  CANSparkFlex shooterMotorR;
  CANSparkFlex shooterMotorL;

  public ShooterSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.ManipulatorConstants.SHOOTER_kP, 0, 0));

    shooterMotorL = new CANSparkFlex(ManipulatorConstants.SHOOTER_MOTOR_LEFT, MotorType.kBrushless);
    // shooterMotorL.restoreFactoryDefaults();
    shooterMotorL.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    shooterMotorL.setInverted(false);
    shooterMotorL.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    shooterMotorL.burnFlash();


    shooterMotorR = new CANSparkFlex(ManipulatorConstants.SHOOTER_MOTOR_RIGHT, MotorType.kBrushless);
    // shooterMotorR.restoreFactoryDefaults();
    shooterMotorR.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    shooterMotorR.setInverted(false);
    shooterMotorR.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    shooterMotorR.follow(shooterMotorL);
    shooterMotorR.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    shooterMotorL.setVoltage(-(output + getController().calculate(getMeasurement(), setpoint)));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("SHOOTER RPM", getShooterLVelocity());
    return getShooterLVelocity();
  }

  public double getMeasurementR() {
    // return the right motor
    return getShooterRVelocity();
  }

  public double getShooterLVelocity() {
    return shooterMotorL.getEncoder().getVelocity();
  }

  public double getShooterRVelocity() {
    return shooterMotorR.getEncoder().getVelocity();
  }

  public void set(double speed) {
    shooterMotorL.set(-speed);
    shooterMotorR.set(-speed);
  }
}
