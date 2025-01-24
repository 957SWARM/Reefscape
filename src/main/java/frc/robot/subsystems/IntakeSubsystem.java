package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    SparkMax neo;
    TimeOfFlight toF;
    double toFRange;
    double appliedVoltage;

    public IntakeSubsystem(){
        neo = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
        toF = new TimeOfFlight(IntakeConstants.SENSOR_ID);
        
        toF.setRangingMode(RangingMode.Short, IntakeConstants.TOF_TIMING_BUDGET);

        appliedVoltage = 0;
    }

    public void periodic(){
        toFRange = toF.getRange() / 1000; // Convert to millimeters

        if (checkToF() && appliedVoltage > 0) {
            appliedVoltage = 0;
        }

        neo.setVoltage(appliedVoltage);
    }

    public boolean checkToF(){
        return (toFRange <= IntakeConstants.TOF_THRESHOLD);
    }

    public Command stopIntakeCommand(){
        return runOnce(() -> {
            appliedVoltage = 0;
        });
    }
    
    public Command intakeCommand (double intakeSpeed){
        return runOnce(() -> {
            appliedVoltage = intakeSpeed;
        });
    }

    public Command ejectCommand (double ejectSpeed){
        return runOnce(() -> {
            appliedVoltage = ejectSpeed;
        });
    }
}
