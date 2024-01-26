// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The PivotSubsystem has the motor objects for the motors of the
 * Pivot on the robot. It also sets them a value based on the input
 * received from a command
 * - 
 * @param pivotSpeed Variable represents the speed passed from a command
 * that pivot motors should be set to
 * @returns pivotPosition through the getPosition()
 */

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  private CANSparkMax pivotMotor;

  public PivotSubsystem() {
   
    pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR, MotorType.kBrushless);
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setSmartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT);
    pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    pivotMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Value Subsystem", getPivotMotorHeight());
  }

  public double getPivotMotorHeight () {
    return pivotMotor.getEncoder().getPosition();
  }

  public RelativeEncoder getEncoder()
  {
    return pivotMotor.getEncoder();
  }

  public void set(double pivotSpeed) {
    pivotMotor.set(pivotSpeed); 
  }


}
