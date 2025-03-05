package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SequencingConstants;

@Logged
public class IntakeSubsystem extends SubsystemBase{

    // Hardware
    private SparkMax neo;
    private TimeOfFlight toF;
    public final SparkMaxConfig motorConfig = new SparkMaxConfig();

    private double toFRange;
    private double appliedVoltage;

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

        // Current Limit
        motorConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        motorConfig.idleMode(IdleMode.kBrake);
        neo.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    public double getVoltage(){
        return appliedVoltage;
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

    // eject command used for auto
    public Command autoEject(double ejectSpeed){
        return runOnce(() -> {
            appliedVoltage = ejectSpeed;
        }).andThen(new WaitUntilCommand(() -> !checkToF()).andThen(new WaitCommand(.2)));
    }

    public double getAppliedVoltage(){
        return neo.getAppliedOutput();
    }

    public double getCommandedVoltage(){
        return appliedVoltage;
    }
}
