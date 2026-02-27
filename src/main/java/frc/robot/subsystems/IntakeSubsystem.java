package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    
    private State state = State.IN;
    private SparkFlex angleMotor = new SparkFlex(20, MotorType.kBrushless);
    private SparkFlex wheelMotor = new SparkFlex(22, MotorType.kBrushless);
    private static final int PULL_IN_CURRENT = 5;
    private MedianFilter currentFilter = new MedianFilter(15);
    private double filteredCurrent = 0;
    private int nonLimitedCurrent = 5;
    private boolean wheelSpin = false;

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
                pullInPeriodic();
                checkPullInCurrent();
                break;

            case IN:
                inPeriodic();
                break;
        
            case OUT:
                outOffPeriodic();
                break;
        }
        wheelMotor.set(wheelSpin ? 0.4 : 0.0);
        updateAngleCurrent();
        SmartDashboard.putNumber("angle motor current", angleMotor.getOutputCurrent());
        SmartDashboard.putString("Intake state", state.toString());
        SmartDashboard.putNumber("filtered current", filteredCurrent);
    }

    private void inPeriodic(){
        angleMotor.set(-0.3);
    }

    private void pullInPeriodic(){
        angleMotor.set(0);
    }

    private void checkPullInCurrent(){
        if (Math.abs(filteredCurrent - PULL_IN_CURRENT) < 10){
            setState(State.IN);
        }
    }

    private void outOffPeriodic(){
        angleMotor.set(0.4);
    }
    
    public enum State{
        IN,
        PULL_IN,
        OUT
    }

    public void setState(State state){
        this.state = state;
        switch (state){
            case IN:
                setAngleCurrent(5);
                break;
            case OUT:
                setAngleCurrent(10);
                break;
            case PULL_IN:
                setAngleCurrent(PULL_IN_CURRENT);
        }
    }

    public Command setWheelsCommand(boolean on){
        return this.runOnce(()->{
            wheelSpin = on;
        });
    }

    public Command toggleInOut(){
        return this.runOnce(()->{
            switch (state) {
                case IN:
                    setState(State.OUT);
                    break;
            
                case PULL_IN:
                    setState(State.OUT);
                    break;

                case OUT:
                    setState(State.PULL_IN);
                    break;
            }
        });
    }

    public Command setStateCommand(State state){
        return this.runOnce(()->setState(state));
    }

    private void setAngleCurrent(int amps){
        nonLimitedCurrent = amps;
    }

    private void updateAngleCurrent(){
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(nonLimitedCurrent);
        angleMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

}
