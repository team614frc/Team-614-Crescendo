// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
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

  private CANSparkFlex pivotMotorR;
  private CANSparkFlex pivotMotorL;

  public PivotSubsystem() {
   
    pivotMotorR = new CANSparkFlex(ManipulatorConstants.PIVOT_MOTOR_RIGHT, MotorType.kBrushless);
    pivotMotorR.restoreFactoryDefaults();
    pivotMotorR.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    pivotMotorR.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    pivotMotorR.burnFlash();

    pivotMotorL = new CANSparkFlex(ManipulatorConstants.PIVOT_MOTOR_LEFT, MotorType.kBrushless);
    pivotMotorL.restoreFactoryDefaults();
    pivotMotorL.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    pivotMotorL.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    pivotMotorL.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("PivotLHeight", getPivotMotorLHeight());
  }

  public double getPivotMotorRHeight () {
    return pivotMotorR.getEncoder().getPosition();
  }

  public double getPivotMotorLHeight () {
    return pivotMotorL.getEncoder().getPosition();
  }

  public RelativeEncoder getPivotLEncoder()
  {
    return pivotMotorL.getEncoder();
  }

  public void set(double pivotSpeed) {
    pivotMotorR.set(-pivotSpeed);
    pivotMotorL.set(pivotSpeed); 
  }
}
