package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Sequencing {

    // gets robot ready to intake from coral station
    public static Command intake(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        // return elevator.toIntake()
        // .alongWith(wrist.toIntake())
        // .alongWith(intake.intakeCommand(IntakeConstants.INTAKE_SPEED));
        return wrist.toIntake();
    }

    public static Command L1(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        // return elevator.toL1()
        // .alongWith(wrist.toL1())
        // .alongWith(intake.stopIntakeCommand());
        return wrist.toL1();
    }

    public static Command L2(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        // return elevator.toL2()
        // .alongWith(wrist.toL2()).
        // alongWith(intake.stopIntakeCommand());
        return wrist.toL2();
    }

    public static Command L3(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        // return elevator.toL3()
        // .alongWith(wrist.toL3())
        // .alongWith(intake.stopIntakeCommand());
        return wrist.toL3();
    }

    public static Command L4(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        // return elevator.toL4()
        // .alongWith(wrist.toL4())
        // .alongWith(intake.stopIntakeCommand());
        return wrist.toL4();
    }

    public static Command stow(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        // return elevator.toL2()
        // .alongWith(wrist.toStow())
        // .alongWith(intake.stopIntakeCommand());
        return wrist.toStow();
    }
}
