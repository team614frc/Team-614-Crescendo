// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ManipulatorConstants {

    // CAN IDs for the Intake and Shooter Motors
    public static final int SHOOTER_MOTOR_RIGHT = 15;
    public static final int SHOOTER_MOTOR_LEFT = 16;
    public static final int LEAF_BLOWER_MOTOR = 34;
    public static final int FEEDER_MOTOR = 17;
    public static final int INTAKE_MOTOR = 25;

    // CAN ID for the Pivot Motors
    public static final int PIVOT_MOTOR_RIGHT = 13;
    public static final int PIVOT_MOTOR_LEFT = 14;

    // Constants for Shooter
    public static final double SHOOT_MAX_VEL_SET = 6630.0;
    public static final double SCORE_AMP_SPEED = 2000;
    public static final double SCORE_SIMPLE = 0.8;
    public static final double SCORE_SIMPLE_RPM = 5000;
    public static final double SHOOTER_FEED = 0.2;
    public static final double AMP_SPEED = 0.2;
    public static final double SHOOTER_kFF = 0.000082;
    public static final double SHOOTER_kP = 0.00045;
    public static final double OUTTAKE_SPEED = -0.5;
    public static final double INTAKE_SPEED = 0.5;
    public static final double INTAKE_REST_SPEED = 0.00;
    public static final double SHOOTER_THRESHOLD = 150;
    public static final double PUKE_SPEED = -0.8;
    public static final double TRAP_SPEED = 4000;

    // Constants for BLOW POWER
    public static final double LEAF_BLOWER_POWER = 0.8;

    // Constants for Feeder
    public static final double LOADBACK_SPEED = -0.08; // MAYBE BACK AND FORTH WAS THE MOVE
    public static final double LOADING_SPEED = 1; // (Note fixed its deformity)
    public static final double AMP_LOAD = 0.5;
    public static final double RUMBLE_TIMER = 2;
    public static final double RUMBLE_SETTING = .6;

    // Speed Constants for Pivot
    public static final double PIVOT_UP_SPEED = 0.1;
    public static final double PIVOT_DOWN_SPEED = -0.1;
    public static final double MOTOR_GRAV_SPEED = 0.02;

    // Encoder Values for the Pivot
    public static final double PIVOT_MAX = -(Math.PI / 2) - 0.02;
    public static final double PIVOT_MIN = 0.08;
    public static final double PIVOT_CLOSE_SCORE = -0.1; // -0.25
    public static final double PIVOT_FAR_SCORE = -0.4; // -0.55
    public static final double PIVOT_AMP_GOAL = -(Math.PI / 2) - 0.02;
    public static final double PIVOT_TRAP_SCORE = -0.15;
    public static final double PIVOT_INTAKE_THRESHOLD = 0.15;
    public static final double PIVOT_SHOOTER_THRESHOLD = 0.1;

    // Others
    public static final double PIVOT_MAX_VEL = 4;
    public static final double PIVOT_MAX_ACCEL = 20;
    public static final double PIVOT_WEIGHT = 9.55;
    public static final double MOTOR_ZERO_SPEED = 0;
    public static final int MOTOR_CURRENT_LIMIT = 40;

  }

  public static final class SensorConstants {

    public static final int sensorPort1 = 0;
    public static final double sensorThreshold = 350;

  }

  public static final class PIDConstants {

    // PID constants for pivot
    public static final double PIVOT_kP = 0.8; // VOLTS
    public static final double PIVOT_kI = 0;
    public static final double PIVOT_kD = 0.0;
    public static final double PIVOT_kS = 0.5;
    public static final double PIVOT_kG = 0.25;
    public static final double PIVOT_kV = 0.25;
    public static final double PIVOT_kA = 0.01;

  }

  public static final class TimeConstants {

    // Timers
    public static final double SpeakerFeed = 2;
    public static final double SpeakerEnd = 2.7;
    public static final double AmpFeed = 1.35;
    public static final double AmpEnd = 2;

  }

  public static final class VisionConstants {

    public static final double alignSetpoint = 0.0;
    public static final double simpleAlignYInput = 1.5;
    public static final double threshold = 0.5;

    public static final double tag7X = -8.308975;
    public static final double tag7Y = 1.442593;

  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK FLEX CAN IDs
    public static final int kFrontLeftDrivingCanId = 10;
    public static final int kRearLeftDrivingCanId = 18;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 19;
    public static final int kFrontRightTurningCanId = 9;
    public static final int kRearRightTurningCanId = 62;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;
    public static final int LEFT_STICK_PRESS = 9;
    public static final int RIGHT_STICK_PRESS = 10;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
