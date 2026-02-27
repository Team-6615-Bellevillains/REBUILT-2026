package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: make auton climb
public class ClimberSubsystem extends SubsystemBase {
    private SparkMax climbMotor = new SparkMax(40, MotorType.kBrushless); // TODO: put real motor id

    public Command climb(double speed) {
        return Commands.startEnd(() -> climbMotor.set(speed), ()->{climbMotor.stopMotor();});
    }
}
