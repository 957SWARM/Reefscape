// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.LEDStripPatterns;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // Mentor built :c
  // private MjpegServer driveCameraServer = new MjpegServer("drive_server", 1181);
  // private MjpegServer climbCameraServer = new MjpegServer("climb_server", 1182);

  // private UsbCamera driveCamera = new UsbCamera("drive_camera", 1);
  // private UsbCamera climbCamera = new UsbCamera("climb_camera", 0);

  private LEDStripPatterns led;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   public Robot() {
    DataLogManager.start(); // Optional to mirror the NetworkTables-logged data to a file on disk
    Epilogue.bind(this);

    // driveCamera.setResolution(240, 180);
    // climbCamera.setResolution(240, 180);

    // temp comment out
    // driveCameraServer.setSource(driveCamera);
    // climbCameraServer.setSource(climbCamera);

  }

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    led = m_robotContainer.led;

    FollowPathCommand.warmupCommand().schedule(); // For Path Planner. Supposedly speeds up followings paths

    // temp comment out
    // CameraServer.startAutomaticCapture(0).setResolution(240, 180); // For end-effector camera
    // CameraServer.startAutomaticCapture(1).setResolution(240, 180);
    
    //led.scheduleDefaultCommand(led.defaultBlueWavesLightCommand(0, LEDConstants.TOTAL_PIXELS, 0.1, false));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    m_robotContainer.reefAlign.updatePoses();
    m_robotContainer.stationAlign.updatePoses();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    //m_robotContainer.zeroHeading();
    m_robotContainer.grabHeading();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    led.scheduleDefaultCommand(led.autoPatternChasingSingleBlueCommand(0, LEDConstants.TOTAL_PIXELS, 0.03333, false));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.fixHeading();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    led.scheduleDefaultCommand(led.defaultBlueWavesLightCommand(0, LEDConstants.TOTAL_PIXELS, 0.1, false));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
