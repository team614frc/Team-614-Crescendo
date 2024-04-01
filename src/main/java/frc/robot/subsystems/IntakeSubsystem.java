// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

/**
 * The IntakeSubsystem contains all the motors for the intake of the robot and sets them a value
 * that is passed to it using a command -
 *
 * @param intakeSpeed Variable indicates the speed passed by the commans that the intake motors
 *     should be set to
 * @returns through the getSpeed() returns the speed that the intake is going at
 */
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkFlex intakeMotor;

  public IntakeSubsystem() {
    // Creates a new motor
    intakeMotor = new CANSparkFlex(ManipulatorConstants.INTAKE_MOTOR, MotorType.kBrushless);
    // intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
    intakeMotor.setInverted(false);
    intakeMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    intakeMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getSpeed() {
    return intakeMotor.get();
  }

  // Sets the value of the motor to a double, at which the motor will run
  public void setIntake(double intakeSpeed) {
    intakeMotor.set(-intakeSpeed);
  }
}
