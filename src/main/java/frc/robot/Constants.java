// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4;  // default: 4.8
    public static final double kMaxAngularSpeed = 2 * Math.PI; // default: 2*pi. radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot ^^^
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot ^^^
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -(Math.PI/2);
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = -(Math.PI);
    public static final double kBackRightChassisAngularOffset = (Math.PI/2);

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1; // New bot: 1, B#: 7
    public static final int kFrontRightDrivingCanId = 3; // New bot: 3, B#: 1
    public static final int kRearLeftDrivingCanId = 5; // New bot: 5, B#: 5
    public static final int kRearRightDrivingCanId = 7; // New bot: 7, B#: 3

    public static final int kFrontLeftTurningCanId = 2; // New bot: 2, B#: 8
    public static final int kFrontRightTurningCanId = 4; // New bot: 4, B#: 2
    public static final int kRearLeftTurningCanId = 6; // New bot: 6, B#: 6
    public static final int kRearRightTurningCanId = 8; // New bot: 8, B#: 4

    public static final boolean kGyroReversed = false;

    // Gyro (Pigeon2) ID
    public static final int pigeonID = 30;  // New bot: 30, B#: 32

    // Location constants
    // Unused right now
    public static final int FRONT_LEFT = 0;
    public static final int FRONT_RIGHT = 0;
    public static final int BACK_LEFT = 0;
    public static final int BACK_RIGHT = 0;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ElevatorConstants {
    public static final int MOTOR_ID = 13;
    public static final int kS = 0; // Add output to overcome static friction
    public static final int kV = 0; // A velocity target of 1 rps results in output
    public static final int kA = 0; // An acceleration of 1 rps/s requires output
    public static final double kP = 1.65; // A position error of 0.2 rotations results in output
    public static final int kI = 0; // No output for integrated error
    public static final int kD = 0; // A velocity error of 1 rps results in output
    public static final double kG = -2; // A velocity error of 1 rps results in output

    public static final int MOTIONMAGIC_VELOCITY = 100; // Target cruise velocity of 80 rps
    public static final int MOTIONMAGIC_ACCELERATION = 480; // Target acceleration of 160 rps/s (0.5 seconds)
    public static final int MOTIONMAGIC_JERK = 2400; // Target jerk of 1600 rps/s/s (0.1 seconds) Robin H.

    // measurements in meters the carriage rises. (end effector raises twice these values)
    public static final double POSITION_GROUND = 0;
    public static final double POSITION_L1 = 0.07;
    public static final double POSITION_L2 = 0.242;
    public static final double POSITION_L3 = 0.452;
    public static final double POSITION_L4 = 0.775;
    public static final double POSITION_INTAKE = 0.1;

    // SLOW RISE/FALL
    public static final double SETPOINT_INCREMENT = .01; // how much the setpoint changes per robot loop in manual control

    // CONVERSIONS
    public static final double metersToRotations = 107.37;
    public static final double RotationsToMeters = 1.0 / metersToRotations;

    // MAXIMUMs/MINIMUMs (meters)
    public static final double MAX_HEIGHT = .78;
    public static final double MIN_HEIGHT = 0;
    public static final int CURRENT_LIMIT = 30;

    public static final double SETPOINT_TOLERANCE = 0.01;

  }

  // angles measured in rotations. 0 defined as straight out from robot
  public static final class WristConstants {
    // CAN IDs & Ports
    public static final int MOTOR_CAN_ID = 9;
    public static final int ENCODER_CAN_ID  = 12;

    // Maximums and Minimums Allowed
    public static final double MAXIMUM_VOLTAGE = 9;
    public static final double MINIMUM_VOLTAGE = -MAXIMUM_VOLTAGE;

    // DO NOT SET VALUES BETWEEN 0.29 AND 0.876!!!!!!!!!!
    public static final double MAXIMUM_ANGLE = 0.876;
    public static final double MINIMUM_ANGLE = 0.29;

    public static final int CURRENT_LIMIT = 30;

    // angle setpoints for scoring, intake, stowing
    public static final double L1_ANGLE = 0;
    public static final double L2_ANGLE = 0.91;
    public static final double L3_ANGLE = L2_ANGLE;
    public static final double L4_ANGLE = 0.9;
    public static final double STOW_ANGLE = 0.165;
    public static final double INTAKE_ANGLE = 0.11;

    // PID + Feedforward
    public static final double kG = 0.5;  // constant multiplied by angle of arm to maintain position
    public static final double kP = 50;
    public static final double kI = 0;
    public static final double kD = 0;

    // Tolerance. Used for checking if wrist is at target angle
    public static final double TOLERANCE = 0.02;

  }

  public static final class LEDConstants {
    public static final int TOTAL_PIXELS = 60;

    public static final int FULL_RED_RGB = 0;
    public static final int FULL_GREEN_RGB = 0;
    public static final int FULL_BLUE_RGB = 0;
  }

  public static final class IntakeConstants {
    public static final int MOTOR_ID = 10;
    public static final int SENSOR_ID = 1;

    public static final int CURRENT_LIMIT = 30;

    // TOF RELATED
    public static final double TOF_TIMING_BUDGET = 30;
    public static final double TOF_THRESHOLD = 100;
    public static final double FILTER_TIME_CONSTANT = .1; // decrease for faster response, increase for less noise

    // SPEEDS
    public static final double IDLE_SPEED = 0;
    public static final double INTAKE_SPEED = 5;
    public static final double EJECT_SPEED = -5;

    public static final double APPLIED_VOLTAGE = 10;
  }

  public static final class IOConstants {

    // Ports
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    // Thresholds
    public static final double TRIGGER_THRESHOLD = .8;
    public static final double DRIVE_DEADBAND = 0.05;

    // BUTTON PORTS
    public static final int LEFT_CENTER_BUTTON = 7;
  }

  public static final class ClimberConstants {

    public static final double CLIMB_PULL_STRENGTH_VOLTS = 10;
    public static final int MOTOR_CAN_ID = 11;

    public static final int CURRENT_LIMIT = 30;
    
  }

  public static final class VisionConstants {
    public static final Pose3d LEFT_REEF = new Pose3d(new Translation3d(-.215, 0, -.63), new Rotation3d());
    public static final Pose3d RIGHT_REEF = new Pose3d(new Translation3d(.115, 0, -.63), new Rotation3d());
    public static final List<Pose3d> REEF_POSES = Arrays.asList(LEFT_REEF, RIGHT_REEF);
    public static final List<Double> REEF_TAG_IDS = Arrays.asList(
      (double)6, 
      (double)7, 
      (double)8, 
      (double)9, 
      (double)10, 
      (double)11, 
      (double)17, 
      (double)18, 
      (double)19, 
      (double)20, 
      (double)21, 
      (double)22
    );

    public static final Pose3d LEFT_STATION = new Pose3d(new Translation3d(-.6, 0, -.6), new Rotation3d());
    public static final Pose3d MID_STATION = new Pose3d(new Translation3d(0, 0, -.6), new Rotation3d());
    public static final List<Pose3d> STATION_POSES = Arrays.asList(LEFT_STATION, MID_STATION);
    public static final List<Double> STATION_TAG_IDS = Arrays.asList(
      (double)1, 
      (double)2, 
      (double)12, 
      (double)13 
    );

    public static final String REEF_LIMELIGHT_NAME = "limelight-reef";
    public static final String STATION_LIMELIGHT_NAME = "limelight-station";

    public static final double STATION_TRANSLATION_P = .55;
    public static final double STATION_TRANSLATION_I = 0;
    public static final double STATION_TRANSLATION_D = 0;

    public static final double STATION_ROTATION_P = 0.025;
    public static final double STATION_ROTATION_I = 0;
    public static final double STATION_ROTATION_D = 0;

    public static final double REEF_TRANSLATION_P = .65;
    public static final double REEF_TRANSLATION_I = 0;
    public static final double REEF_TRANSLATION_D = 0;

    public static final double REEF_ROTATION_P = 0.025;
    public static final double REEF_ROTATION_I = 0;
    public static final double REEF_ROTATION_D = 0;

    public static final double MAX_VISION_SPEED = 0.5; //JOYSTICK

    public static final double REEF_TRANSLATION_TOLERANCE = 0.02; //METERS
    public static final double ROTATION_TOLERANCE = 1; //DEGREES?
    public static final double SPEED_TOLERANCE = 0.1; //METERS PER SECOND

    public static final double STATION_TRANSLATION_TOLERANCE = 0.035;
  }

  public static final class SequencingConstants {
    public static final double STOW_DELAY = 0.15;
    public static final double L1_WRIST_DELAY = .4;
    public static final double L2_WRIST_DELAY = .1;
    public static final double L3_WRIST_DELAY = .3;
    public static final double L4_WRIST_DELAY = .8;
  }

}
