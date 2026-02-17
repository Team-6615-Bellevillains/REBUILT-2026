package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    
    private SparkMax spindexerMotor = new SparkMax(0, MotorType.kBrushless);
    private SparkMax roadMotor = new SparkMax(0, MotorType.kBrushless);
    private State state = State.OFF;

    @Override
    public void periodic() {
        if (state == State.ON){
            on();
        } else {
            off();
        }
    }

    private void on(){
        spindexerMotor.set(0.5);
        roadMotor.set(0.5);
    }

    private void off(){
        spindexerMotor.stopMotor();
        roadMotor.stopMotor();
    }

    public void setState(State state){
        this.state = state;
    }

    public enum State {
        ON,
        OFF
    }
}
