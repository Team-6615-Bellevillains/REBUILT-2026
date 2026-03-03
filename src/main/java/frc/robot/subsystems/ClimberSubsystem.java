package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: make auton climb
public class ClimberSubsystem extends SubsystemBase {
    private SparkMax climbMotor = new SparkMax(40, MotorType.kBrushless);

    public ClimberSubsystem(){
        SparkMaxConfig config = new SparkMaxConfig();
        SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        //TODO: get actual soft-limit values
        softLimitConfig.forwardSoftLimit(0);
        softLimitConfig.reverseSoftLimit(0);
        softLimitConfig.forwardSoftLimitEnabled(true);
        softLimitConfig.reverseSoftLimitEnabled(true);
        config.softLimit.apply(softLimitConfig);
        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command climb(double speed) {
        return Commands.startEnd(() -> climbMotor.set(speed), ()->{climbMotor.stopMotor();});
    }
}
