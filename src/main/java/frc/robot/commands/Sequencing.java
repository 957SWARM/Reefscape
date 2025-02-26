package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Sequencing {

    // gets robot ready to intake from coral station
    public static Command intake(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toIntake()
        .alongWith(wrist.toIntake())
        .alongWith(intake.intakeCommand(IntakeConstants.INTAKE_SPEED));
    }

    public static Command L1(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL1()
        .alongWith(new WaitCommand(.4).andThen(wrist.toL1()))
        .alongWith(intake.stopIntakeCommand());
    }

    public static Command L2(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL2()
        .alongWith(new WaitCommand(0.1).andThen(wrist.toL2()))
        .alongWith(intake.stopIntakeCommand());
    }

    public static Command L3(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL3()
        .alongWith(new WaitCommand(.3).andThen(wrist.toL3()))
        .alongWith(intake.stopIntakeCommand());
    }

    public static Command L4(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL4()
        .alongWith(new WaitCommand(.8).andThen(wrist.toL4()))
        .alongWith(intake.stopIntakeCommand());
    }

    public static Command stow(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toStow()
        .alongWith(wrist.toStow())
        .alongWith(intake.stopIntakeCommand());
    }
}
