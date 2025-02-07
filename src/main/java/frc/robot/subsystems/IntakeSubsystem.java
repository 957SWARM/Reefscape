package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

@Logged
public class IntakeSubsystem extends SubsystemBase{

    // Hardware
    SparkMax neo;
    TimeOfFlight toF;

    double toFRange;
    double appliedVoltage;

    // helps reduce noise in the tof sensor
    LinearFilter tofFilter = LinearFilter.singlePoleIIR(
        IntakeConstants.FILTER_TIME_CONSTANT,
        Units.millisecondsToSeconds(IntakeConstants.TOF_TIMING_BUDGET)
    );

    public IntakeSubsystem(){
        neo = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
        toF = new TimeOfFlight(IntakeConstants.SENSOR_ID);
        
        toF.setRangingMode(RangingMode.Short, IntakeConstants.TOF_TIMING_BUDGET);

        appliedVoltage = 0;
    }

    public void periodic(){
        toFRange = toF.getRange();

        if (checkToF() && appliedVoltage > 0) {
            appliedVoltage = 0;
        }

        neo.setVoltage(appliedVoltage);
    }

    public boolean checkToF(){
        return (tofFilter.calculate(toFRange) <= IntakeConstants.TOF_THRESHOLD);
    }

    public double readToF(){
        return toF.getRange();
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
