// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ManipulatorConstants;

public class PivotPIDSub extends ProfiledPIDSubsystem {
  /** Creates a new PivotPIDSub. */
  private CANSparkFlex pivotMotorR;
  private CANSparkFlex pivotMotorL;
  private final ArmFeedforward pivotfeedforward=
      new ArmFeedforward(1,1,0.5,0.1);

  public PivotPIDSub() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            ManipulatorConstants.PIVOT_kP,
            ManipulatorConstants.PIVOT_kI,
            ManipulatorConstants.PIVOT_kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              ManipulatorConstants.MaxArmVelocity, ManipulatorConstants.MaxArmAccel)));
      
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

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedforward = pivotfeedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    pivotMotorR.set(-(output + feedforward));
    pivotMotorL.set((output + feedforward));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return pivotMotorL.getEncoder().getPosition();
  }
}
