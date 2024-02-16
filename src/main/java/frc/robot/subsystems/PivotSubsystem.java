// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The PivotSubsystem has the motor objects for the motors of the
 * Pivot on the robot. It also sets them a value based on the input
 * received from a command
 * -
 * 
 * @param pivotSpeed Variable represents the speed passed from a command
 *                   that pivot motors should be set to
 * @returns pivotPosition through the getPosition()
 */

public class PivotSubsystem extends PIDSubsystem {
  /** Creates a new PivotSubsystem. */

  private CANSparkMax pivotMotor;
  public static double pivotP = 0.025;

  public PivotSubsystem() {
    super(
        // The controller that the command will use
        new PIDController(pivotP, ManipulatorConstants.PIVOT_kI, ManipulatorConstants.PIVOT_kD));

    getController().setTolerance(0.1);

    pivotMotor = new CANSparkMax(ManipulatorConstants.PIVOT_MOTOR, MotorType.kBrushless);
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    pivotMotor.getEncoder().setPosition(0);
    pivotMotor.burnFlash();
    SmartDashboard.putData("PivotSubsystem", this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    SmartDashboard.putNumber("Encoder Value Subsystem", getPivotMotorHeight());
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    pivotMotor.set((output + getController().calculate(getMeasurement(), setpoint)));
  }

  @Override
  protected double getMeasurement() {
  return RobotContainer.pivotSubsystem.getPivotMotorHeight();
  }

  public void setPivotP(double pivotP) {
    this.pivotP = pivotP;
  }

  public double getPivotMotorHeight() {
    return pivotMotor.getEncoder().getPosition();
  }

  public RelativeEncoder getEncoder() {
    return pivotMotor.getEncoder();
  }

  public void set(double pivotSpeed) {
    pivotMotor.set(pivotSpeed);
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }
}