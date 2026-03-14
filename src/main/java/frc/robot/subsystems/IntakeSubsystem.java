package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    
    private State state = State.IN;
    private SparkFlex angleMotor = new SparkFlex(20, MotorType.kBrushless);
    private SparkFlex wheelMotor = new SparkFlex(22, MotorType.kBrushless);
    private SparkClosedLoopController angleController = angleMotor.getClosedLoopController();
    private static final int PULL_IN_ANGLE_CURRENT = 30;
    private MedianFilter angleCurrentFIlter = new MedianFilter(25);
    private double filteredAngleCurrent = 0;
    private int nonLimitedAngleCurrent = PULL_IN_ANGLE_CURRENT;
    private boolean shouldRunWheelsInIntakeDirection = false;
    private final Supplier<ChassisSpeeds> getRobotRelativeVelocity;
    private final double IN_WHEEL_DUTY_CYCLE = -0.05;

    public IntakeSubsystem(Supplier<ChassisSpeeds> getRobotRelativeVelocity){
        SparkFlexConfig angleMotorConfig = new SparkFlexConfig();
        angleMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.smartCurrentLimit(1);
        angleMotorConfig.closedLoop.pid(0.2,0,0);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig wheelMotorConfig = new SparkFlexConfig();
        wheelMotorConfig.idleMode(IdleMode.kCoast);
        wheelMotorConfig.smartCurrentLimit(80);
        wheelMotor.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.getRobotRelativeVelocity = getRobotRelativeVelocity;

        angleMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        filteredAngleCurrent = angleCurrentFIlter.calculate(angleMotor.getOutputCurrent());
        switch (state) {
            case PULL_IN:
                pullInPeriodic();
                checkPullInCurrent();
                wheelMotor.set(IN_WHEEL_DUTY_CYCLE);
                break;

            case IN:
                inPeriodic();
                wheelMotor.set(IN_WHEEL_DUTY_CYCLE);
                break;
        
            case OUT:
                outOffPeriodic();
                wheelMotor.set(shouldRunWheelsInIntakeDirection ? getActiveWheelDutyCycle() : 0);
                break;
            
            case MID_HOLD:
                midHold();
                wheelMotor.set(shouldRunWheelsInIntakeDirection ? getActiveWheelDutyCycle() : 0);
                break;
        }
        updateAngleCurrent();
        SmartDashboard.putNumber("angle motor current", angleMotor.getOutputCurrent());
        SmartDashboard.putString("Intake state", state.toString());
        SmartDashboard.putNumber("filtered current", filteredAngleCurrent);
        SmartDashboard.putNumber("intake rpm", wheelMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("wheel varying duty cycle", getActiveWheelDutyCycle());
    }

    private void midHold(){
        setAngleSetpoint(-0.5);
    }

    private void inPeriodic(){
        angleMotor.set(0.3);
    }

    private void pullInPeriodic(){
        angleMotor.set(0.3);
    }

    private void checkPullInCurrent(){
        if (Math.abs(filteredAngleCurrent - PULL_IN_ANGLE_CURRENT) < 10){
            setState(State.IN);
        }
    }

    private void outOffPeriodic(){
        angleMotor.set(-0.6);
    }
    
    public enum State{
        IN,
        PULL_IN,
        OUT,
        MID_HOLD
    }

    public void setState(State state){
        this.state = state;
        switch (state){
            case IN:
                setAngleCurrent(7);
                updateWheelCurrent(10);
                break;
            case OUT:
                setAngleCurrent(25); //change 
                updateWheelCurrent(80);
                break;
            case PULL_IN:
                setAngleCurrent(PULL_IN_ANGLE_CURRENT);
                updateWheelCurrent(10);
            case MID_HOLD:
                setAngleCurrent(PULL_IN_ANGLE_CURRENT);
                updateWheelCurrent(80);
        }
    }

    public Command setWheelsCommand(boolean on){
        return this.runOnce(()->{
            shouldRunWheelsInIntakeDirection = on;
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

                case MID_HOLD:
                    setState(State.OUT);
                    break;

                case OUT:
                    setState(State.MID_HOLD);
                    break;
            }
        });
    }

    public Command setStateCommand(State state){
        return this.runOnce(()->setState(state));
    }

    private void setAngleCurrent(int amps){
        nonLimitedAngleCurrent = amps;
    }

    private void updateAngleCurrent(){
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(nonLimitedAngleCurrent);
        angleMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void updateWheelCurrent(int newLimit){
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(newLimit);
        wheelMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private double getActiveWheelDutyCycle(){
        ChassisSpeeds robotRelativeVelocity = getRobotRelativeVelocity.get();
        SmartDashboard.putNumber("velocity in intake direction", robotRelativeVelocity.vxMetersPerSecond);
        SmartDashboard.putNumber("intake lerp t", robotRelativeVelocity.vxMetersPerSecond/Constants.MAX_SPEED.in(MetersPerSecond));
        return MathUtil.interpolate(0.4, 1, robotRelativeVelocity.vxMetersPerSecond/Constants.MAX_SPEED.in(MetersPerSecond));
    }

    public void setAngleSetpoint(double setpoint){
        angleController.setSetpoint(setpoint, ControlType.kPosition);
    }
}
