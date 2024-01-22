// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
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

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  private CANSparkFlex climbMotor;

  public ClimbSubsystem() {
   
    climbMotor = new CANSparkFlex(IntakeConstants.PIVOT_MOTOR, MotorType.kBrushless);
    climbMotor.restoreFactoryDefaults();
    climbMotor.setSmartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT);
    climbMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    climbMotor.burnFlash();
    SmartDashboard.putData("ClimbSubsystem", this);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Value Subsystem", getClimbMotorHeight());
  }

  public double getClimbMotorHeight () {
    return climbMotor.getEncoder().getPosition();
  }

  public RelativeEncoder getEncoder()
  {
    return climbMotor.getEncoder();
  }

  public void set(double pivotSpeed) {
    climbMotor.set(pivotSpeed); 
  }


}
