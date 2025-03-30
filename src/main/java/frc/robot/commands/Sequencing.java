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
        .alongWith(wrist.toStow().andThen(new WaitUntilCommand(
            () -> -elevator.getCarriageHeight() < elevator.getTargetSetpoint() + ElevatorConstants.UPPER_WRIST_START_TOLERANCE
            && -elevator.getCarriageHeight() > elevator.getTargetSetpoint() + ElevatorConstants.LOWER_WRIST_START_TOLERANCE
            ).andThen(wrist.toL2())))
        .alongWith(intake.stopIntakeCommand())
        .andThen(new WaitUntilCommand(()-> wrist.atSetpoint() && elevator.atSetpoint()));
    }

    public static Command L3(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL3()
        .alongWith(wrist.toStow().andThen(new WaitUntilCommand(
            () -> -elevator.getCarriageHeight() < elevator.getTargetSetpoint() + ElevatorConstants.UPPER_WRIST_START_TOLERANCE
            && -elevator.getCarriageHeight() > elevator.getTargetSetpoint() + ElevatorConstants.LOWER_WRIST_START_TOLERANCE
            ).andThen(wrist.toL3())))
        .alongWith(intake.stopIntakeCommand())
        .andThen(new WaitUntilCommand(()-> wrist.atSetpoint() && elevator.atSetpoint()));
    }

    public static Command L4(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toL4()
        .alongWith(wrist.toStow().andThen(new WaitUntilCommand(
        () -> -elevator.getCarriageHeight() < elevator.getTargetSetpoint() + ElevatorConstants.UPPER_WRIST_START_TOLERANCE
        && -elevator.getCarriageHeight() > elevator.getTargetSetpoint() + ElevatorConstants.LOWER_WRIST_START_TOLERANCE
        ).andThen(wrist.toL4())))
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
        return elevator.toHighStow()
        .alongWith(wrist.toStow())
        .alongWith(intake.stopIntakeCommand());
    }

    public static Command deepStow(ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake){
        return elevator.toStow()
        .alongWith(Commands.runOnce(
            () -> { if (elevator.getTargetSetpoint() == ElevatorConstants.POSITION_GROUND) 
                    wrist.toDeepStow(); 
                })
        )
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
        .andThen(wrist.toStow())
        .andThen(new WaitUntilCommand(() -> elevator.atSetpoint()))
        .andThen(wrist.toL1())
        .andThen(new WaitCommand(0.1))
        .andThen(Commands.run(() -> drive.drive(0.2, 0, 0, false, 0)).until(() -> StationAlign.tof.getRange()/1000 < 0.28))
        //.withTimeout(0.55))
        .andThen(wrist.toStow()
        .alongWith(elevator.toLowRemove(ElevatorConstants.REMOVAL_INCREMENT))
        .alongWith(Commands.run(() -> drive.drive(-0.35, 0, 0, false, 0)))
        .withTimeout(0.25));
    }

    public static Command removeHigh(ElevatorSubsystem elevator, WristSubsystem wrist, DriveSubsystem drive){
        return elevator.toHighRemove(0)
        .andThen(wrist.toStow())
        .andThen(new WaitUntilCommand(() -> elevator.atSetpoint())) //0.1
        .andThen(wrist.toL1())
        .andThen(new WaitCommand(0.1))
        .andThen(Commands.run(() -> drive.driveBasic(0.25, 0, 0, false, 0)).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).until(() -> StationAlign.tof.getRange()/1000 < 0.28))
        //.withTimeout(0.57))
        .andThen(wrist.toStow()
        .alongWith(elevator.toHighRemove(ElevatorConstants.REMOVAL_INCREMENT))
        .alongWith(Commands.run(() -> drive.driveBasic(-0.35, 0, 0, false, 0)))
        .withTimeout(0.25));
    }
}
