// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.LEDStripPatterns;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.Sequencing;
import frc.robot.commands.StationAlign;
import frc.robot.input.DriverInput;
import frc.robot.subsystems.ClimbSubsystem;
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

  private final SendableChooser<Command> autoChooser;

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final WristSubsystem  m_wrist = new WristSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ClimbSubsystem m_climber = new ClimbSubsystem();

  // Command classes
  final ReefAlign reefAlign = new ReefAlign();
  final StationAlign stationAlign = new StationAlign();
  final LEDStripPatterns led = new LEDStripPatterns();

  // Controllers
  DriverInput m_driver = new DriverInput();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    NamedCommands.registerCommand("Go L1", Sequencing.L1(m_elevator, m_wrist, m_intake));
    NamedCommands.registerCommand("Stow", Sequencing.stow(m_elevator, m_wrist, m_intake));
    NamedCommands.registerCommand("High Stow", Sequencing.highStow(m_elevator, m_wrist, m_intake));
    NamedCommands.registerCommand("Score", m_intake.autoEject(IntakeConstants.EJECT_SPEED));
    NamedCommands.registerCommand("Reef Align", reefAlign.alignNearestReef(m_robotDrive));
    NamedCommands.registerCommand("Go L4", Sequencing.L4(m_elevator, m_wrist, m_intake));
    NamedCommands.registerCommand("Go L2", Sequencing.L2(m_elevator, m_wrist, m_intake));
    NamedCommands.registerCommand("Station Align", stationAlign.alignNearestStation(m_robotDrive));
    NamedCommands.registerCommand("Intake", Sequencing.autoIntake(m_elevator, m_wrist, m_intake));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //autoChooser.addOption("Test Auto", new PathPlannerAuto("Test Auto"));
    autoChooser.addOption("Nothing", new InstantCommand());
    autoChooser.addOption("Just Leave", new PathPlannerAuto("Just Leave"));
    autoChooser.addOption("Near L4", new PathPlannerAuto("Near L4 Auto"));
    autoChooser.addOption("Right 2 L4", new PathPlannerAuto("Right 2 L4 Auto"));
    autoChooser.addOption("Left 2 L4", new PathPlannerAuto("Left 2 L4 Auto"));
    autoChooser.addOption("Right 2.5 L4", new PathPlannerAuto("Right 2.5 L4 Auto"));
    autoChooser.addOption("Right 3 L4 Auto", new PathPlannerAuto("Right 3 L4 Auto"));
    autoChooser.addOption("Buddy Auto", new PathPlannerAuto("Buddy Auto"));

    //configureNamedCommands();

    // Configure the button bindings
    configureButtonBindings();

    // State Triggers
    //rumbles controller when coral is intaked
    Trigger intakeRumble = new Trigger(() -> m_intake.checkToF());
    intakeRumble.onTrue( Commands.run(() -> m_driver.setRumble(true))
      .withTimeout(.75)
      .andThen(Commands.run(() -> m_driver.setRumble(false))));

    // stops intake when coral leavese
    // Trigger coralLeft = new Trigger(() -> !m_intake.checkToF() && m_intake.getVoltage() == IntakeConstants.EJECT_SPEED
    // && !DriverStation.isAutonomous());
    // coralLeft.onTrue(new WaitCommand(.25).andThen(m_intake.stopIntakeCommand()).andThen(Sequencing.stow(m_elevator, m_wrist, m_intake)));

    // automatically sends robot to stow after intaking
    Trigger coralIn = new Trigger(
      () -> m_elevator.getTargetSetpoint() == ElevatorConstants.POSITION_INTAKE 
      && m_wrist.getTargetSetpoint() == WristConstants.INTAKE_ANGLE
      && m_intake.checkToF() && !DriverStation.isAutonomous());
    coralIn.onTrue(Sequencing.stow(m_elevator, m_wrist, m_intake));

    // Configure default commands

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driver.driveX(), IOConstants.DRIVE_DEADBAND),
                -MathUtil.applyDeadband(m_driver.driveY(), IOConstants.DRIVE_DEADBAND),
                -MathUtil.applyDeadband(m_driver.driveTurn(), IOConstants.DRIVE_DEADBAND),
                true,
                m_elevator.getHeight()),
            m_robotDrive));

    // m_wrist.setDefaultCommand(m_wrist.toStow());

    //led.scheduleDefaultCommand(led.blueWavesLightCommand(0, LEDConstants.TOTAL_PIXELS, 0.1, false));
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

    // sends elevator, wrist, and intake ready to take in coral from loading station
    new Trigger(() -> m_driver.intake())
      .onTrue(Sequencing.intake(m_elevator, m_wrist, m_intake));

    // sends elevator and wrist to L1 position
    new Trigger(() -> m_driver.L1())
      .onTrue(Sequencing.L1(m_elevator, m_wrist, m_intake));

    // sends elevator and wrist to L2 position
    new Trigger(() -> m_driver.L2())
      .onTrue(Sequencing.L2(m_elevator, m_wrist, m_intake));

    // sends elevator and wrist to L3 position
    new Trigger(() -> m_driver.L3())
      .onTrue(Sequencing.L3(m_elevator, m_wrist, m_intake));

    // sends elevator and wrist to L4 position
    new Trigger(() -> m_driver.L4())
      .onTrue(Sequencing.L4(m_elevator, m_wrist, m_intake));

    // while holding trigger, runs intake backwards for scoring
    new Trigger(() -> m_driver.score())
      .whileTrue(m_intake.ejectCommand(IntakeConstants.EJECT_SPEED));

    // sends elevator and wrist to stow position
    new Trigger(() -> m_driver.stow())
      .onTrue(Sequencing.stow(m_elevator, m_wrist, m_intake));

    // dynamic vision align
    new Trigger(() -> m_driver.visionAlign() && reefAlign.checkReefTag())
    .whileTrue(reefAlign.alignNearestReef(m_robotDrive));

    new Trigger(() -> m_driver.visionAlign() && stationAlign.checkStationTag())
    .whileTrue(stationAlign.alignNearestStation(m_robotDrive)
    .andThen(Sequencing.intake(m_elevator, m_wrist, m_intake)
    .alongWith(Commands.run(() -> m_robotDrive.setX()))));
    
    // climbs while up on d-pad is held
    new Trigger(() -> m_driver.deployClimb())
      .whileTrue(m_climber.extend())
      .onFalse(m_climber.stopCommand());
      
    // retracts climber while down on d-pad is held
    new Trigger(() -> m_driver.retractClimb())
      .whileTrue(m_climber.retract())
      .onFalse(m_climber.stopCommand());

    new Trigger(() -> m_driver.lowRemove())
      .whileTrue(Sequencing.removeLow(m_elevator, m_wrist, m_robotDrive))
      .onFalse(Sequencing.stow(m_elevator, m_wrist, m_intake));

      new Trigger(() -> m_driver.highRemove())
      .whileTrue(Sequencing.removeHigh(m_elevator, m_wrist, m_robotDrive))
      .onFalse(Sequencing.stow(m_elevator, m_wrist, m_intake));
  }

  public void configureNamedCommands(){
    NamedCommands.registerCommand("Score L1", Sequencing.L1(m_elevator, m_wrist, m_intake));
    NamedCommands.registerCommand("Stow", Sequencing.stow(m_elevator, m_wrist, m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
}
