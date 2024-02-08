// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkFlex;
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

  private CANSparkFlex pivotMotorR;
  private CANSparkFlex pivotMotorL;

  public PivotSubsystem() {
    super(
        // The controller that the command will use
        new PIDController(ManipulatorConstants.PIVOT_kP, ManipulatorConstants.PIVOT_kI, ManipulatorConstants.PIVOT_kD));

    getController().setTolerance(0.1);

    pivotMotorR = new CANSparkFlex(ManipulatorConstants.PIVOT_MOTOR_RIGHT, MotorType.kBrushless);
    pivotMotorR.restoreFactoryDefaults();
    pivotMotorR.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    pivotMotorR.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    pivotMotorR.getEncoder().setPosition(0);
    pivotMotorR.burnFlash();

    pivotMotorL = new CANSparkFlex(ManipulatorConstants.PIVOT_MOTOR_LEFT, MotorType.kBrushless);
    pivotMotorL.restoreFactoryDefaults();
    pivotMotorL.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    pivotMotorL.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    pivotMotorL.getEncoder().setPosition(0);
    pivotMotorL.burnFlash();

    // SmartDashboard.putData("PivotSubsystem", this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    SmartDashboard.putNumber("Encoder Value Subsystem", getPivotMotorLHeight());
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    pivotMotorR.set(-output + getController().calculate(getMeasurement(), setpoint));
    pivotMotorL.set((output + getController().calculate(getMeasurement(), setpoint)));
  }

  @Override
  protected double getMeasurement() {
    return RobotContainer.pivotSubsystem.getPivotMotorLHeight();
  }

  public double getPivotMotorLHeight() {
    return pivotMotorL.getEncoder().getPosition();
  }

  public RelativeEncoder getEncoder() {
    return pivotMotorL.getEncoder();
  }

  public void set(double pivotSpeed) {
    pivotMotorR.set(-pivotSpeed);
    pivotMotorL.set(pivotSpeed);
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }
}