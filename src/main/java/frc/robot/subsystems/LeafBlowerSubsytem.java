// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class LeafBlowerSubsytem extends SubsystemBase {
    /** Creates a new LeafBlowerSubsystem. */
    private CANSparkMax leafBlowerMotor;

    public LeafBlowerSubsytem() {
        leafBlowerMotor = new CANSparkMax(ManipulatorConstants.LEAF_BLOWER_MOTOR, MotorType.kBrushed);
        leafBlowerMotor.setSmartCurrentLimit(ManipulatorConstants.MOTOR_CURRENT_LIMIT);
        leafBlowerMotor.setInverted(false);
        leafBlowerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leafBlowerMotor.burnFlash();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void blow(double power) {
        leafBlowerMotor.set(power);
    }
}
