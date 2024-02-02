// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


/**
 * The IntakeSubsystem contains all the motors for the intake
 * of the robot and sets them a value that is passed to it
 * using a command
 * -
 * @param intakeSpeed Variable indicates the speed passed by the commans
 * that the intake motors should be set to
 * @returns through the getSpeed() returns the speed that the intake is going at
 */

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  private CANSparkFlex intakeMotorR;
  private CANSparkFlex intakeMotorL;
  
  public IntakeSubsystem() {
    // Creates a new motor

    intakeMotorR = new CANSparkFlex(IntakeConstants.INTAKE_MOTOR_RIGHT, MotorType.kBrushless);
    intakeMotorL = new CANSparkFlex(IntakeConstants.INTAKE_MOTOR_LEFT, MotorType.kBrushless);
    intakeMotorR.restoreFactoryDefaults();
    intakeMotorL.restoreFactoryDefaults();
    intakeMotorR.setSmartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT);
    intakeMotorL.setSmartCurrentLimit(IntakeConstants.MOTOR_CURRENT_LIMIT);
    intakeMotorL.setInverted(false);
    intakeMotorR.setInverted(false);
    intakeMotorL.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    intakeMotorR.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    intakeMotorL.burnFlash();
    intakeMotorR.burnFlash(); 
    // intakeMotorL.setInverted(false);
    // intakeMotorR.setInverted(true);
    //intakeMotorL.follow(intakeMotorR); // Sets the left motor to be the follow of the right intake motor
    SmartDashboard.putData("IntakeSubsystem", this);
  }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getSpeed() {
    //intakeMotorR.get();
    SmartDashboard.putNumber("Intake Speed Right", intakeMotorR.get());
    SmartDashboard.putNumber("Intake Speed Right", intakeMotorL.get());
  }

  // Sets the value of the motor to a double, at which the motor will run
  public void set(double intakeSpeed) {
    intakeMotorR.set(-intakeSpeed);
    intakeMotorL.set(intakeSpeed);
  }

}
