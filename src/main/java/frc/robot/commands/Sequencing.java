package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SequencingConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class Sequencing {

    // gets robot ready to intake from coral station
    public static Command intake(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toIntake()
        .alongWith(wrist.toIntake())
        .alongWith(intake.intakeCommand(IntakeConstants.INTAKE_SPEED));
    }

    // intake command for use in auto. Automatically stows when coral intaked without use of trigger
    public static Command autoIntake(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toIntake()
        .alongWith(wrist.toIntake())
        .alongWith(intake.intakeCommand(IntakeConstants.INTAKE_SPEED))
        .andThen(new WaitUntilCommand(() -> intake.checkToF()))
        .andThen(highStow(elevator, wrist, intake));
    }

    public static Command L1(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL1()
        .alongWith(new WaitCommand(SequencingConstants.L1_WRIST_DELAY).andThen(wrist.toL1()))
        .alongWith(intake.stopIntakeCommand())
        .andThen(new WaitUntilCommand(()-> wrist.atSetpoint() && elevator.atSetpoint()));
    }

    public static Command L2(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL2()
        .alongWith(new WaitCommand(SequencingConstants.L2_WRIST_DELAY).andThen(wrist.toL2()))
        .alongWith(intake.stopIntakeCommand())
        .andThen(new WaitUntilCommand(()-> wrist.atSetpoint() && elevator.atSetpoint()));
    }

    public static Command L3(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL3()
        .alongWith(new WaitCommand(SequencingConstants.L3_WRIST_DELAY).andThen(wrist.toL3()))
        .alongWith(intake.stopIntakeCommand())
        .andThen(new WaitUntilCommand(()-> wrist.atSetpoint() && elevator.atSetpoint()));
    }

    public static Command L4(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL4()
        .alongWith(new WaitCommand(SequencingConstants.L4_WRIST_DELAY).andThen(wrist.toL4()))
        .alongWith(intake.stopIntakeCommand())
        .andThen(new WaitUntilCommand(()-> wrist.atSetpoint() && elevator.atSetpoint()));
    }

    public static Command stow(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toStow()
        .alongWith(wrist.toStow())
        .alongWith(intake.stopIntakeCommand())
        .andThen(new WaitUntilCommand(() -> elevator.atSetpoint() && wrist.atSetpoint()));
    }

    public static Command highStow(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL2()
        .alongWith(wrist.toStow())
        .alongWith(intake.stopIntakeCommand());
    }

    // for use in auto when going up from high stow
    public static Command quickL4(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL4()
        .alongWith(new WaitCommand(SequencingConstants.QUICK_L4_WRIST_DELAY).andThen(wrist.toL4()))
        .alongWith(intake.stopIntakeCommand())
        .andThen(new WaitUntilCommand(()-> wrist.atSetpoint() && elevator.atSetpoint()));
    }

    public static Command removeLow(ElevatorSubsystem elevator, WristSubsystem wrist, DriveSubsystem drive){
        return elevator.toLowRemove(0)
        .alongWith(wrist.toL1())
        .andThen(Commands.run(() -> drive.drive(0.1, 0, 0, false, 0))
        .withTimeout(0.55))
        .andThen(new WaitCommand(.25))
        .andThen(wrist.toStow()
        .alongWith(elevator.toLowRemove(ElevatorConstants.REMOVAL_INCREMENT))
        .alongWith(Commands.run(() -> drive.drive(-0.25, 0, 0, false, 0)))
        .withTimeout(0.75));
    }

    public static Command removeHigh(ElevatorSubsystem elevator, WristSubsystem wrist, DriveSubsystem drive){
        return elevator.toHighRemove(0)
        .alongWith(wrist.toL1())
        .andThen(new WaitCommand(0.25))
        .andThen(Commands.run(() -> drive.drive(0.1, 0, 0, false, 0))
        .withTimeout(0.55))
        .andThen(new WaitCommand(.25))
        .andThen(wrist.toStow()
        .alongWith(elevator.toHighRemove(ElevatorConstants.REMOVAL_INCREMENT))
        .alongWith(Commands.run(() -> drive.drive(-0.25, 0, 0, false, 0)))
        .withTimeout(0.75));
    }
}
