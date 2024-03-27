// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private NetworkTable limelightTable;
  private double x, y, area, angle;
  private Pose2d robotPose;
  private InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  private double[] aprilTagInfo;
  
  public LimelightSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-speaker");
    robotPose = RobotContainer.swerveDrive.getPose();
    this.turnOffLEDs();
    aprilTagInfo = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    //LIST OF VALUES FOR ANGLEMAP GOES HERE
    angleMap.put(1.3, ManipulatorConstants.PIVOT_CLOSE_SCORE);
    angleMap.put(2.705, -.31);
    angleMap.put(3.675, -.45);
  }

  public void enableVisionProcessing() {
    limelightTable.getEntry("camMode").setNumber(0); // Set Limelight to vision processing mode
  }

  public void turnOffLEDs() {
    limelightTable.getEntry("ledMode").setNumber(1); // Turn off Limelight LEDs
  }

  public void blinkLEDs() {
    limelightTable.getEntry("ledMode").setNumber(2); // Blink the LEDs
  }

  public void turnOnLEDs() {
    limelightTable.getEntry("ledMode").setNumber(3); // Turn on Limelight LEDs
  }

  public double getAngleOffset() { // Get the angle offset for the AprilTag in view
    return x;
  }

  public double getTargetAngle(double Y, double X) { // Gets the target angle from where the robot in relation to starting heading
    angle = Math.atan2(
      robotPose.getY() - Y,
      robotPose.getX() - X);
    return angle;
  }

  public double interpolateAngle(double key) {
    return angleMap.get(key);
  }

  public double estimateDistance() {
    double test2 = Math.sqrt(Math.pow(aprilTagInfo[2], 2) + Math.pow(aprilTagInfo[0], 2));
    SmartDashboard.putNumber("limelight range", test2);
    return test2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = limelightTable.getEntry("tx").getDouble(0.0);
    y = limelightTable.getEntry("ty").getDouble(0.0);
    area = limelightTable.getEntry("ta").getDouble(0.0);
    aprilTagInfo = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }
}