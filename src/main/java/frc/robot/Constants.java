// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
    public static final double kMaxSpeedMetersPerSecond = 2.4;  // default: 4.8
    public static final double kMaxAngularSpeed = 2 * Math.PI; // default: 2*pi. radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(20);
    // Distance between centers of right and left wheels on robot ^^^
    public static final double kWheelBase = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot ^^^
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 4;

    public static final boolean kGyroReversed = false;

    // Gyro (Pigeon2) ID
    public static final int pigeonID = 32;

    // Location constants
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
    public static final int MOTOR_ID = 0;
    public static final int kS = 0; // Add output to overcome static friction
    public static final int kV = 0; // A velocity target of 1 rps results in output
    public static final int kA = 0; // An acceleration of 1 rps/s requires output
    public static final int kP = 0; // A position error of 0.2 rotations results in output
    public static final int kI = 0; // No output for integrated error
    public static final int kD = 0; // A velocity error of 1 rps results in output
    public static final int kG = 0; // A velocity error of 1 rps results in output

    public static final int MOTIONMAGIC_VELOCITY = 5; 
    public static final int MOTIONMAGIC_ACCELERATION = 10; 
    public static final int MOTIONMAGIC_JERK = 100;

    public static final double POSITION_GROUND = 0;
    public static final double POSITION_L1 = 4;
    public static final double POSITION_L2 = 5;
    public static final double POSITION_L3 = 6;
    public static final double POSITION_L4 = 7;
    public static final double POSITION_INTAKE = 4;

    // SLOW RISE/FALL
    public static final double SETPOINT_INCREMENT = .01; // how much the setpoint changes per robot loop in manual control

    // CONVERSIONS
    public static final double metersToRotations = 1;

    // MAXIMUMs/MINIMUMs
    public static final double MAX_HEIGHT = 3;
    public static final double MIN_HEIGHT = 1;

  }

  public static final class WristConstants {
    // CAN IDs & Ports
    public static final int MOTOR_CAN_ID = 0;
    public static final int ENCODER_CAN_ID  = 0;

    // Maximums and Minimums Allowed
    public static final double MAXIMUM_VOLTAGE = 3;
    public static final double MINIMUM_VOLTAGE = -MAXIMUM_VOLTAGE;

    public static final double MAXIMUM_ANGLE = .6;
    public static final double MINIMUM_ANGLE = 0.1;

    // angle setpoints for scoring, intake, stowing
    public static final double L1_ANGLE = 0.125;
    public static final double L2_ANGLE = .2;
    public static final double L3_ANGLE = L2_ANGLE;
    public static final double L4_ANGLE = .4;
    public static final double STOW_ANGLE = .5;
    public static final double INTAKE_ANGLE = .3;

    // PID + Feedforward
    public static final double kG = .5;  // constant multiplied by angle of arm to maintain position
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    // SLOW FALL/RISE VOLTAGE. Voltage that slowly moves the motor intended for manual adjustment
    public static final double SLOW_RISE_VOLTAGE = .5;
    public static final double SLOW_FALL_VOLTAGE = -SLOW_RISE_VOLTAGE;

  }

  public static final class LEDConstants {
    public static final int TOTAL_PIXELS = 60;

    public static final int FULL_RED_RGB = 0;
    public static final int FULL_GREEN_RGB = 0;
    public static final int FULL_BLUE_RGB = 0;
  }

  public static final class IntakeConstants {
    public static final int MOTOR_ID = 0;
    public static final int SENSOR_ID = 1;

    public static final double TOF_TIMING_BUDGET = 0.003;
    public static final double TOF_THRESHOLD = 10;

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
}
