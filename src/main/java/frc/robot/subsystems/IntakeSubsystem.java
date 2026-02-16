package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    
    private State state = State.IN;
    private SparkFlex angleMotor = new SparkFlex(0, MotorType.kBrushless);
    private SparkFlex wheelMotor = new SparkFlex(0, MotorType.kBrushless);

    @Override
    public void periodic() {
        switch (state) {
            case IN:
                inPeriodic();
                break;
        
            case OUT_OFF:
                outOffPeriodic();
                break;

            case OUT_ON:
                outOnPeriodic();
                break;
        }
    }

    private void inPeriodic(){
        angleMotor.set(-0.2);
        wheelMotor.set(0);
    }

    private void outOffPeriodic(){
        angleMotor.set(0.2);
        wheelMotor.set(0);
    }

    private void outOnPeriodic(){
        angleMotor.set(0.2);
        wheelMotor.set(0.9);
    }
    
    public enum State{
        IN,
        OUT_ON,
        OUT_OFF
    }

    public Command setState(State state){
        return this.runOnce(()->{this.state = state;});
    }

}
