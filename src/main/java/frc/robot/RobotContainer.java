// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Sequencing;
import frc.robot.input.DriverInput;
import frc.robot.input.OperatorInput;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  // WristSubsystem  m_wrist = new WristSubsystem();
  // IntakeSubsystem m_intake = new IntakeSubsystem();

  // Controllers
  DriverInput m_driver = new DriverInput();
  OperatorInput m_operator = new OperatorInput();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driver.driveX(), IOConstants.DRIVE_DEADBAND),
                -MathUtil.applyDeadband(m_driver.driveY(), IOConstants.DRIVE_DEADBAND),
                -MathUtil.applyDeadband(m_driver.driveTurn(), IOConstants.DRIVE_DEADBAND),
                true),
            m_robotDrive));

    //m_wrist.setDefaultCommand(m_wrist.toStow());

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  // default drive command defined in RobotContainer constructor
  private void configureButtonBindings() {

    
    // DRIVER CONTROLS
    // zeroes the pigeon gyro
    new Trigger(() -> m_driver.resetGyro())
      .onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));

      /*
    // sends elevator, wrist, and intake ready to take in coral from loading station
    new Trigger(() -> m_driver.intake())
      .onTrue(Sequencing.intake(m_elevator, m_wrist, m_intake));

    // sends elevator and wrist to L1 position
    new Trigger(() -> m_driver.L1())
      .onTrue(Sequencing.L1(m_elevator, m_wrist));

    // sends elevator and wrist to L2 position
    new Trigger(() -> m_driver.L2())
      .onTrue(Sequencing.L2(m_elevator, m_wrist));

    // sends elevator and wrist to L3 position
    new Trigger(() -> m_driver.L3())
      .onTrue(Sequencing.L3(m_elevator, m_wrist));

    // sends elevator and wrist to L4 position
    new Trigger(() -> m_driver.L4())
      .onTrue(Sequencing.L4(m_elevator, m_wrist));

    // while holding trigger, runs intake backwards for scoring
    new Trigger(() -> m_driver.score())
      .whileTrue(m_intake.ejectCommand(IntakeConstants.EJECT_SPEED));

    // sends elevator and wrist to stow position
    new Trigger(() -> m_driver.stow())
      .onTrue(Sequencing.stow(m_elevator, m_wrist));

    // manual control for slowly lifting elevator
    new Trigger(() -> m_driver.slowRise())
      .whileTrue(m_elevator.slowRise());

    // manual control for slowly descending elevator
    new Trigger(() -> m_driver.slowFall())
      .whileTrue(m_elevator.slowFall());
    */
    // OPERATOR CONTROLS (implement once climber is implemented)
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
  
}
