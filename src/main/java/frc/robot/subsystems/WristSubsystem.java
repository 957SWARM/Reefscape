package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

@Logged
public class WristSubsystem extends SubsystemBase{

    TalonFX motor = new TalonFX(WristConstants.MOTOR_CAN_ID);
    Canandmag encoder = new Canandmag(WristConstants.ENCODER_CAN_ID);
    PIDController pid = new PIDController(
        WristConstants.kP, 
        WristConstants.kI, 
        WristConstants.kD
    );
    double targetSetpoint = WristConstants.STOW_ANGLE;

    public WristSubsystem(){ 
    }

    public void periodic(){

        double modifiedSetpoint = targetSetpoint;
        if (modifiedSetpoint >= 0.5) 
            modifiedSetpoint = modifiedSetpoint - 1;

        pid.setSetpoint(modifiedSetpoint);

        // calculations of feedback and feedforward
        double feedback = pid.calculate(getPosition());  //pid feedback on current position
        double feedforward = getFeedForward();  // feedforwad that counteracts gravity

        // clamping of output to minimum/maximum voltage
        double output = MathUtil.clamp(
            feedback + feedforward,
            WristConstants.MINIMUM_VOLTAGE,
            WristConstants.MAXIMUM_VOLTAGE
        );

        motor.setVoltage(output);
    }

    // angle measured in rotations
    public Command setSetpoint(double setpoint) {
        return runOnce(
                () -> {
                    // makes sure angle is within a reasonable range
                    targetSetpoint = MathUtil.clamp(
                        setpoint,
                        WristConstants.MINIMUM_ANGLE,
                        WristConstants.MAXIMUM_ANGLE
                    );
                });
    }

    public Command toL1(){
        return setSetpoint(WristConstants.L1_ANGLE);
    }

    public Command toL2(){
        return setSetpoint(WristConstants.L2_ANGLE);
    }

    public Command toL3(){
        return setSetpoint(WristConstants.L3_ANGLE);
    }

    public Command toL4(){
        return setSetpoint(WristConstants.L4_ANGLE);
    }

    public Command toIntake(){
        return setSetpoint(WristConstants.INTAKE_ANGLE);
    }

    public Command toStow(){
        return setSetpoint(WristConstants.STOW_ANGLE);
    }

    // gets encoder position
    public double getPosition(){
        return encoder.getAbsPosition();
    }

    // Helper function. feedforward = voltage to remain at current position
    private double getFeedForward(){
        // angle measured between straight down and current position
        // converted to radians for sine function
        double angle = Units.rotationsToRadians(encoder.getAbsPosition());
        return Math.cos(angle) * WristConstants.kG;
    }
}
