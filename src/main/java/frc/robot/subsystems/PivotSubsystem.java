// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ManipulatorConstants;

public class PivotSubsystem extends ProfiledPIDSubsystem {
  private CANSparkMax pivotMotor;

  private ArmFeedforward feedforward = new ArmFeedforward(
    ManipulatorConstants.PIVOT_kS,
    ManipulatorConstants.PIVOT_kG,
    ManipulatorConstants.PIVOT_kV,
    ManipulatorConstants.PIVOT_kA);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            ManipulatorConstants.PIVOT_kP,
            ManipulatorConstants.PIVOT_kI,
            ManipulatorConstants.PIVOT_kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ManipulatorConstants.pivotMaxVelocity, ManipulatorConstants.pivotMaxAccel)));
    pivotMotor = new CANSparkMax(ManipulatorConstants.PIVOT_MOTOR, MotorType.kBrushless);
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    pivotMotor.getEncoder().setPosition(0);
    pivotMotor.burnFlash();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    SmartDashboard.putNumber("PivotSubsystem", getMeasurement());  
    SmartDashboard.putNumber("ENCODER", getEncoderinDegrees());  
    double feed = feedforward.calculate(setpoint.position, setpoint.velocity);
    pivotMotor.set(output + getController().calculate(getMeasurement() + feed));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoderinDegrees();
  }

  public double getEncoderinDegrees() {
    return (pivotMotor.getEncoder().getPosition()/7.0);
  }

  public double getEncoderinRadians() {
    return (getEncoderinDegrees()*(Math.PI/180));
  }
}
