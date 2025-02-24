package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

    // fancy sequencing. Coordinates elevator and wrist to Level, then scores coral, then stows

    // full automatic L1 sequence
    public static Command L1Fancy(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return L1(elevator, wrist, intake)
            .andThen(intake.ejectCommand(IntakeConstants.EJECT_SPEED))
            .andThen(new WaitUntilCommand(() -> !intake.checkToF()))
            .andThen(new WaitCommand(SequencingConstants.STOW_DELAY)
            .andThen(intake.stopIntakeCommand()))
            .andThen(stow(elevator, wrist, intake));
    }

    // full automatic L2 sequence
    public static Command L2Fancy(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return L2(elevator, wrist, intake)
            .andThen(intake.ejectCommand(IntakeConstants.EJECT_SPEED))
            .andThen(new WaitUntilCommand(() -> !intake.checkToF()))
            .andThen(new WaitCommand(SequencingConstants.STOW_DELAY)
            .andThen(intake.stopIntakeCommand()))
            .andThen(stow(elevator, wrist, intake));
    }

    // full automatic L3 sequence
    public static Command L3Fancy(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return L3(elevator, wrist, intake)
            .andThen(intake.ejectCommand(IntakeConstants.EJECT_SPEED))
            .andThen(new WaitUntilCommand(() -> !intake.checkToF()))
            .andThen(new WaitCommand(SequencingConstants.STOW_DELAY)
            .andThen(intake.stopIntakeCommand()))
            .andThen(stow(elevator, wrist, intake));
    }

    // full automatic L4 sequence
    public static Command L4Fancy(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return L4(elevator, wrist, intake)
            .andThen(intake.ejectCommand(IntakeConstants.EJECT_SPEED))
            .andThen(new WaitUntilCommand(() -> !intake.checkToF()))
            .andThen(new WaitCommand(SequencingConstants.STOW_DELAY)
            .andThen(intake.stopIntakeCommand()))
            .andThen(stow(elevator, wrist, intake));
    }


    public static Command stow(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toStow()
        .alongWith(wrist.toStow())
        .alongWith(intake.stopIntakeCommand());
    }
}
