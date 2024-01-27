// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The ShooterSubsystem contains all the motors for the shooterw
 * of the robot and sets them a value that is passed to it
 * using a command
 * -
 * @param shooterSpeed Variable indicates the speed passed by the commans
 * that the shooter motors should be set to
 * @returns through the getSpeed() returns the speed that the shooter is going at
 */

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  
  private CANSparkMax shooterMotorR;
  private CANSparkMax shooterMotorL;
  
  public ShooterSubsystem() {
    // Creates a new motor

    shooterMotorR = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_RIGHT, MotorType.kBrushless);
    shooterMotorL = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_LEFT, MotorType.kBrushless);
    shooterMotorR.restoreFactoryDefaults();
    shooterMotorL.restoreFactoryDefaults();
    shooterMotorR.setSmartCurrentLimit(ShooterConstants.MOTOR_CURRENT_LIMIT);
    shooterMotorL.setSmartCurrentLimit(ShooterConstants.MOTOR_CURRENT_LIMIT);
    shooterMotorL.setInverted(false);
    shooterMotorR.setInverted(false);
    shooterMotorL.setIdleMode(CANSparkMax.IdleMode.kBrake);
    shooterMotorR.setIdleMode(CANSparkMax.IdleMode.kBrake);
    shooterMotorL.burnFlash();
    shooterMotorR.burnFlash(); 

    //intakeMotorL.follow(intakeMotorR); // Sets the left motor to be the follow of the right intake motor
    SmartDashboard.putData("ShooterSubsystem", this);
  }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    
    
  }

  public void getSpeed() {
    SmartDashboard.putNumber("Shooter Speed Right", shooterMotorR.get());
    SmartDashboard.putNumber("Shooter Speed Right", shooterMotorL.get());
  }

  // Sets the value of the motor to a double, at which the motor will run
  public void set(double shooterSpeed) {
    shooterMotorR.set(-shooterSpeed);
    shooterMotorL.set(shooterSpeed);
  }

}
