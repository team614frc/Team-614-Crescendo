// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private NetworkTable limelightTable;
  private double x, y, area;
  
  public LimelightSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void enableVisionProcessing() {
    limelightTable.getEntry("camMode").setNumber(0); // Set Limelight to vision processing mode
  }

  public void turnOnLEDs() {
    limelightTable.getEntry("ledMode").setNumber(3); // Turn on Limelight LEDs
  }

  public double getHorizontalAngle() {
    return x;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = limelightTable.getEntry("tx").getDouble(0.0);
    y = limelightTable.getEntry("ty").getDouble(0.0);
    area = limelightTable.getEntry("ta").getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}