// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ManipulatorConstants;

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
    SmartDashboard.putNumber("PivotSubsystem", getMeasurement());  
    SmartDashboard.putNumber("ENCODER Ticks", pivotMotorL.getEncoder().getCountsPerRevolution());  
    //SmartDashboard.putNumber("Encoder", pivotMotorL.getEncoder().getMeasurementPeriod());
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
    double val = 100/360.0; //7168 ticks per rev, 100:1 gear ratio, ticks per full rotation in degrees
    return (getPivotLEncoder()/val);
  }

  public double getEncoderinRadians() {
    return (getEncoderinDegrees()*(Math.PI/180.0));
  }

  public boolean atGoal(double goal) {
    return (getMeasurement() > (goal-ManipulatorConstants.PIVOT_THRESHOLD)) && (getMeasurement() < (goal+ManipulatorConstants.PIVOT_THRESHOLD)); 
    
  }

  public void set(double speed) {
    pivotMotorL.set(-speed);
    pivotMotorR.set(speed);
  }
}
