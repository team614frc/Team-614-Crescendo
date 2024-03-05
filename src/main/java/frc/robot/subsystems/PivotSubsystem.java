// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/**
 * The PivotSubsystem has the motor objects for the motors of the
 * Pivot on the robot. It also sets them a value based on the input
 * received from a command
 * - 
 * @param pivotSpeed Variable represents the speed passed from a command
 * that pivot motors should be set to
 * @returns pivotPosition through the getPosition()
 */

public class PivotSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new PivotSubsystem. */

  private CANSparkFlex pivotMotorR;
  private CANSparkFlex pivotMotorL;

  private ArmFeedforward feedforward = new ArmFeedforward(
    PIDConstants.PIVOT_kS,
    PIDConstants.PIVOT_kG,
    PIDConstants.PIVOT_kV,
    PIDConstants.PIVOT_kA);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    super(
        new ProfiledPIDController(
            PIDConstants.PIVOT_kP,
            PIDConstants.PIVOT_kI,
            PIDConstants.PIVOT_kD,
            new TrapezoidProfile.Constraints(
              ManipulatorConstants.PIVOT_MAX_VEL, 
              ManipulatorConstants.PIVOT_MAX_ACCEL)));

    pivotMotorR = new CANSparkFlex(ManipulatorConstants.PIVOT_MOTOR_RIGHT, MotorType.kBrushless);
    // pivotMotorR.restoreFactoryDefaults();
    pivotMotorR.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    pivotMotorR.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    pivotMotorR.getEncoder().setPosition(0);
    pivotMotorR.setInverted(true);
    pivotMotorR.burnFlash();

    pivotMotorL = new CANSparkFlex(ManipulatorConstants.PIVOT_MOTOR_LEFT, MotorType.kBrushless);
    // pivotMotorL.restoreFactoryDefaults();
    pivotMotorL.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    pivotMotorL.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    pivotMotorL.getEncoder().setPosition(0);
    pivotMotorL.burnFlash();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feed = feedforward.calculate(setpoint.position, setpoint.velocity);
    pivotMotorL.set(output + getController().calculate(getMeasurement() + feed));
    pivotMotorR.set(output + getController().calculate(getMeasurement() + feed));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoderinRadians();
  }

  public double getPivotLEncoder() {
    return pivotMotorL.getEncoder().getPosition();
  }

  public double getEncoderinDegrees() {
    double val = 180.0/360.0;  // 7168 ticks per rev, 180:1 gear ratio
    return (getPivotLEncoder()/val);
  }

  public double getEncoderinRadians() {
    return (getEncoderinDegrees()*(Math.PI/180.0));
  }

  public boolean atGoal(double goal) {
    return (getMeasurement() > Math.abs(goal-ManipulatorConstants.PIVOT_THRESHOLD)); 
  }

  public void set(double speed) {
    pivotMotorL.set(speed);
    pivotMotorR.set(speed);
  }
}
