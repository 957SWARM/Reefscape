package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

@Logged
public class ElevatorSubsystem extends SubsystemBase{

    // Hardware
    private TalonFX kraken;
    private DigitalInput bottomLimitSwitch = new DigitalInput(0);

    private final MotionMagicVoltage request;
    private double targetSetpoint = ElevatorConstants.POSITION_GROUND;

    boolean isReset = false;

    public ElevatorSubsystem(){
        kraken = new TalonFX(ElevatorConstants.MOTOR_ID);
        
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs slot0 = configs.Slot0;
        slot0.kS = ElevatorConstants.kS;
        slot0.kV = ElevatorConstants.kV; 
        slot0.kA = ElevatorConstants.kA;
        slot0.kP = ElevatorConstants.kP; 
        slot0.kI = ElevatorConstants.kI; 
        slot0.kD = ElevatorConstants.kD;

        MotionMagicConfigs mmConfigs = configs.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MOTIONMAGIC_VELOCITY; 
        mmConfigs.MotionMagicAcceleration = ElevatorConstants.MOTIONMAGIC_ACCELERATION;
        mmConfigs.MotionMagicJerk = ElevatorConstants.MOTIONMAGIC_JERK; 

        kraken.getConfigurator().apply(configs);

        // Current Limit to 30 A
        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.SupplyCurrentLimit = ElevatorConstants.CURRENT_LIMIT;
        kraken.getConfigurator().apply(limitConfigs);

        request = new MotionMagicVoltage(ElevatorConstants.kG);
    }

    public void periodic(){
        if (bottomLimitSwitch.get() && !isReset) {
            kraken.setPosition(0);
            targetSetpoint = ElevatorConstants.POSITION_GROUND;
            isReset = !isReset;
        }

        if (!bottomLimitSwitch.get() && isReset){
            isReset = !isReset;
        }
        
        kraken.setControl(request.withPosition(getAsRotations(-targetSetpoint)));

    }

    public static double getAsRotations(double meters){
        double rotations = meters * ElevatorConstants.metersToRotations;
        return rotations;
    }

    // height of end effector from rest position (0)
    // multiply by 2 because of 2nd stage
    public double getHeight(){
        return kraken.getPosition().getValueAsDouble() * ElevatorConstants.RotationsToMeters * 2;
    }

    public double getCarriageHeight(){
        return kraken.getPosition().getValueAsDouble() * ElevatorConstants.RotationsToMeters;
    }

    public double getPosition(){
        return kraken.getPosition().getValueAsDouble();
    }

    public double getTargetSetpoint(){
        return targetSetpoint;
    }

    public boolean atSetpoint(){
        // targetSetpoint added instead of subtracted because targetSetpoint gets inverted later
        return Math.abs(getCarriageHeight() + targetSetpoint) < ElevatorConstants.SETPOINT_TOLERANCE;
    }

    public boolean isStalled(){
        return (!atSetpoint() 
        && Math.abs(kraken.getVelocity().getValueAsDouble()) <= 0.5
        && Math.abs(kraken.getStatorCurrent().getValueAsDouble()) >= 20);
    }

    public double getVelocity(){
        return kraken.getVelocity().getValueAsDouble();
    }

    public double getAppliedVoltage(){
        return kraken.getMotorVoltage().getValueAsDouble();
    }

    private void assignSetpoint(double assignSetpoint){
        // make sure setpoint is within safe range
        targetSetpoint = MathUtil.clamp(
            assignSetpoint, 
            ElevatorConstants.MIN_HEIGHT, 
            ElevatorConstants.MAX_HEIGHT
        );
    }

    public Command toStow(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_STOW);
        });
    }

    public Command toDeepStow(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_GROUND);
        });
    }

    public Command toL1(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_L1);
        });
    }
    
    public Command toL2(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_L2);
        });
    }

    public Command toHighStow(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_HIGH_STOW);
        });
    }

    public Command toL3(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_L3);
        });
    }

    public Command toL4(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_L4);
        });
    }

    public Command toIntake(){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_INTAKE);
        });
    }

    public Command slowRise(){
        return Commands.run(() -> {
            assignSetpoint(targetSetpoint += ElevatorConstants.SETPOINT_INCREMENT);
        });
    }

    public Command fall(){
        return Commands.run(() -> {
            assignSetpoint(targetSetpoint -= ElevatorConstants.SETPOINT_INCREMENT);
        });
    }

    public Command toLowRemove(double increment){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_LOW_REMOVE + increment);
        });
    }

    public Command toHighRemove(double increment){
        return Commands.runOnce(() -> {
            assignSetpoint(ElevatorConstants.POSITION_HIGH_REMOVE + increment);
        });
    }

}
