package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax climbMotor = new SparkMax(0, MotorType.kBrushless); // TODO: put real motor id

    public Command climb(double speed) {
        return Command.run(() -> climbMotor.set(speed));
    }

    public Command stop() {
        return Command.run(() -> climbMotor.stopMotor());
    }
}
