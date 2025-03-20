// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.VisionConstants;

@Logged
public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor (replaced ADIS16470_IMU with Pigeon2)
  private final Pigeon2 m_gyro = new Pigeon2(Constants.DriveConstants.pigeonID);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    // sets up AutoBuilder for Path Planner
    initializeAuto();

    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // resets gyro
    zeroHeading();

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

  }

  /*   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public String getPoseString(){
    return getPose().toString();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double elevatorHeight) {

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = adjustSpeed(xSpeed * DriveConstants.kMaxSpeedMetersPerSecond, elevatorHeight);
    double ySpeedDelivered = adjustSpeed(ySpeed * DriveConstants.kMaxSpeedMetersPerSecond, elevatorHeight);
    double rotDelivered = adjustSpeed(rot * DriveConstants.kMaxAngularSpeed, elevatorHeight);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                m_gyro.getRotation2d()))
            : discretize(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  // the solution
  public ChassisSpeeds getRobotRelativeSpeeds(){
    SwerveModuleState[] swerveStates = {m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()};
    return DriveConstants.kDriveKinematics.toChassisSpeeds(swerveStates);
  }

  public double getLinearSpeed(){
    return Math.hypot(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond);
  }

  //Same as drive function but for auto. Accepts chassisSpeeds directly instead of generating it from other arguments
  //Always robot relative
  public void autoDrive(ChassisSpeeds speeds) {
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    var swerveModuleStates =  DriveConstants.kDriveKinematics.toSwerveModuleStates(discretizedSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public boolean pitchTipping(){
    return Math.abs(getPitch() - 2.46) > DriveConstants.MAXIMUM_TIP_DEGREES;
  }

  public boolean rollPitching(){
    return Math.abs(getRoll() + 3.48) > DriveConstants.MAXIMUM_TIP_DEGREES;
  }

  public void setHeading(double angleInDegrees){
    m_gyro.setYaw(angleInDegrees);
  }

  public Command setHeadingCommand(double angleInDegrees){
    return Commands.runOnce(() -> setHeading(angleInDegrees));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getPitch(){
    return m_gyro.getPitch().getValueAsDouble(); // returns pitch in degrees
  }

  public double getRoll(){
    return m_gyro.getRoll().getValueAsDouble(); // returns roll in degrees
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // not sure if this rate from the gyro is done correctly (1/13 8:30PM)
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);

  }

  public boolean speedCondition(){
    return getLinearSpeed() < VisionConstants.REEF_SPEED_TOLERANCE;
  }

  // returns new speed based on elevator height (higher elevator = slower speed)
  public double adjustSpeed(double speed, double elevatorHeight){
    // elevator max height multiplied by 2 because of 2nd stage
    double fractionOfHeight = Math.abs(elevatorHeight / (ElevatorConstants.MAX_HEIGHT * 2));
    double fractionOfSpeed = 0.40 + (0.60 * (1 - fractionOfHeight));
    fractionOfSpeed = MathUtil.clamp(fractionOfSpeed, 0.25, 1);
    return fractionOfSpeed * speed;
  }

  // Helper Function for setting up AutoBuilder for Path Planner. Mostly copied code from Path Planner
  private void initializeAuto(){
    // configs for auto
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> autoDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0,0, 0), // Translation PID constants
                    new PIDConstants(5, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  // copied code from 4738. Helps drift when driving and rotating simultaneously
  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose = new Pose2d(
      speeds.vxMetersPerSecond * dt, 
      speeds.vyMetersPerSecond * dt, 
      new Rotation2d(speeds.omegaRadiansPerSecond * dt * 4)
    );
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

}
