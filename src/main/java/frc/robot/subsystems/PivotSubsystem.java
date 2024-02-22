// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< HEAD
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
=======
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
>>>>>>> zabz-profpidtest
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ManipulatorConstants;

<<<<<<< HEAD
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
=======
public class PivotSubsystem extends ProfiledPIDSubsystem {
  private CANSparkMax pivotMotor;
>>>>>>> zabz-profpidtest

  private ArmFeedforward feedforward = new ArmFeedforward(
    ManipulatorConstants.PIVOT_kS,
    ManipulatorConstants.PIVOT_kG,
    ManipulatorConstants.PIVOT_kV,
    ManipulatorConstants.PIVOT_kA);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
<<<<<<< HEAD
   
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
=======
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
>>>>>>> zabz-profpidtest
  }
}
