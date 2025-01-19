package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    SparkMax left;
    SparkMax right;
    RelativeEncoder encoder;

    public ElevatorSubsystem(){
        left = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        right = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();

        rightConfig.apply(globalConfig).follow(left);
        rightConfig.apply(rightConfig).inverted(true);

        left.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        right.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = left.getAlternateEncoder();
    }
}
