// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  CANSparkFlex shooterMotorR;
  CANSparkFlex shooterMotorL;

  public ShooterSubsystem() {

    shooterMotorL = new CANSparkFlex(ManipulatorConstants.SHOOTER_MOTOR_LEFT, MotorType.kBrushless);
    shooterMotorL.restoreFactoryDefaults();
    shooterMotorL.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    shooterMotorL.setInverted(false);
    shooterMotorL.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    shooterMotorL.burnFlash();

    shooterMotorR = new CANSparkFlex(ManipulatorConstants.SHOOTER_MOTOR_RIGHT, MotorType.kBrushless);
    shooterMotorR.restoreFactoryDefaults();
    shooterMotorR.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    shooterMotorR.setInverted(false);
    shooterMotorR.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    shooterMotorR.burnFlash(); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double shoot) {
    shooterMotorL.set(-shoot);
    shooterMotorR.set(-shoot);
  }

  public void getShooterVelocity () {
    shooterMotorL.getEncoder().getVelocity();
    shooterMotorR.getEncoder().getVelocity();
  }

}
