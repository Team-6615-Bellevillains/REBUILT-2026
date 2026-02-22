package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    
    private State state = State.IN;
    private SparkFlex angleMotor = new SparkFlex(20, MotorType.kBrushless);
    private SparkFlex wheelMotor = new SparkFlex(22, MotorType.kBrushless);
    private static final int PULL_IN_CURRENT = 60;
    private MedianFilter currentFilter = new MedianFilter(20);
    private double filteredCurrent = 0;

    public IntakeSubsystem(){
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(1);
        angleMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        filteredCurrent = currentFilter.calculate(angleMotor.getOutputCurrent());
        switch (state) {
            case PULL_IN:
                inPeriodic();
                checkPullInCurrent();
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
        SmartDashboard.putNumber("angle motor current", angleMotor.getOutputCurrent());
        SmartDashboard.putString("Intake state", state.toString());
        SmartDashboard.putNumber("filtered current", filteredCurrent);
    }

    private void inPeriodic(){
        angleMotor.set(-0.25);
        wheelMotor.set(0);
    }

    private void checkPullInCurrent(){
        if (Math.abs(filteredCurrent - PULL_IN_CURRENT) < 10){
            setState(State.IN);
        }
    }

    private void outOffPeriodic(){
        angleMotor.set(0.02);
        wheelMotor.set(0);
    }

    private void outOnPeriodic(){
        angleMotor.set(0.15);
        wheelMotor.set(0.3);
    }
    
    public enum State{
        IN,
        PULL_IN,
        OUT_ON,
        OUT_OFF
    }

    public void setState(State state){
        this.state = state;
        switch (state){
            case IN:
                setAngleCurrent(10);
                break;
            case OUT_OFF:
                setAngleCurrent(1);
                break;
            case OUT_ON:
                setAngleCurrent(5);
                break;
            case PULL_IN:
                setAngleCurrent(PULL_IN_CURRENT);
        }
    }

    public Command setStateCommand(State state){
        return this.runOnce(()->setState(state));
    }

    private void setAngleCurrent(int amps){
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(amps);
        angleMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

}
