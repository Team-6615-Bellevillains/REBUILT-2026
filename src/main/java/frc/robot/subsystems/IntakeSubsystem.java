package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.Console;
import java.util.function.Supplier;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    
    private State state = State.IN;
    private SparkFlex angleMotor = new SparkFlex(20, MotorType.kBrushless);
    private SparkFlex speedMotor = new SparkFlex(24, MotorType.kBrushless);
    private SparkFlex wheelMotor = new SparkFlex(22, MotorType.kBrushless);
    private SparkClosedLoopController angleController = angleMotor.getClosedLoopController();
    private static final int PULL_IN_ANGLE_CURRENT = 30;
    private MedianFilter angleCurrentFilter = new MedianFilter(25);
    private double filteredAngleCurrent = 0;
    private int nonLimitedAngleCurrent = PULL_IN_ANGLE_CURRENT;
    private boolean shouldRunWheelsInIntakeDirection = false;
    private final Supplier<ChassisSpeeds> getRobotRelativeVelocity;
    private final double IN_WHEEL_DUTY_CYCLE = -0.20;
    private final SparkClosedLoopController wheelController = wheelMotor.getClosedLoopController();

    public IntakeSubsystem(Supplier<ChassisSpeeds> getRobotRelativeVelocity){
        SparkFlexConfig angleMotorConfig = new SparkFlexConfig();
        angleMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.smartCurrentLimit(1);
        angleMotorConfig.closedLoop.pid(0.2,0,0, ClosedLoopSlot.kSlot0);
        angleMotorConfig.closedLoop.pid(0.15,0,0, ClosedLoopSlot.kSlot1);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig wheelMotorConfig = new SparkFlexConfig();
        wheelMotorConfig.idleMode(IdleMode.kCoast);
        wheelMotorConfig.smartCurrentLimit(80);
        wheelMotorConfig.closedLoop.pid(0.0001, 0, 0)
        .feedForward.kS(0.18).kV(0.0026);
        wheelMotorConfig.encoder.velocityConversionFactor((25.0/36.0));
        wheelMotor.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.getRobotRelativeVelocity = getRobotRelativeVelocity;

        SparkFlexConfig speedMotorConfig = new SparkFlexConfig();
        speedMotorConfig.follow(22);
        speedMotor.configure(speedMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        angleMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        filteredAngleCurrent = angleCurrentFilter.calculate(angleMotor.getOutputCurrent());
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
                if(shouldRunWheelsInIntakeDirection)
                {
                    wheelController.setSetpoint(1145.4, ControlType.kVelocity);
                } 
                else 
                {
                    wheelMotor.stopMotor();
                }
                
                break;
            
            case MID_HOLD:
                midHold();
                if(shouldRunWheelsInIntakeDirection)
                {
                    wheelController.setSetpoint(1145.4, ControlType.kVelocity);
                } 
                else 
                {
                    wheelMotor.stopMotor();
                }
                break;

            case PUSH_OUT:
                pushOutPeriodic();
                wheelMotor.set(-IN_WHEEL_DUTY_CYCLE);
                if (angleMotor.getEncoder().getPosition() < -3.2) {
                    setState(State.OUT);
                }
                break;
                
            case REVERSE:
                wheelMotor.set(-0.5);
        }
        updateAngleCurrent();
        SmartDashboard.putNumber("angle motor current", angleMotor.getOutputCurrent());
        SmartDashboard.putString("Intake state", state.toString());
        SmartDashboard.putNumber("filtered current", filteredAngleCurrent);
        SmartDashboard.putNumber("intake rpm", wheelMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("wheel varying duty cycle", getActiveWheelDutyCycle());
    }

    private void midHold(){
    }

    private void inPeriodic(){
        angleMotor.set(0.35);
    }

    private void pullInPeriodic(){
        angleMotor.set(0.3);
    }

    private void pushOutPeriodic(){
        angleMotor.set(-0.6);
    }

    private void checkPullInCurrent(){
        if (Math.abs(filteredAngleCurrent - PULL_IN_ANGLE_CURRENT) < 10){
            setState(State.IN);
        }
    }

    private void outOffPeriodic(){
    }
    
    public enum State{
        IN,
        PULL_IN,
        OUT,
        MID_HOLD,
        PUSH_OUT,
        REVERSE
    }

    public void setState(State state){
        this.state = state;
        switch (state){
            case IN:
                setAngleCurrent(16);
                updateWheelCurrent(8);
                break;
            case OUT:
                setAngleCurrent(60); //change 
                updateWheelCurrent(80);
                setAngleSetpoint(-4, ClosedLoopSlot.kSlot1);
                break; 
            case PULL_IN:
                setAngleCurrent(PULL_IN_ANGLE_CURRENT);
                updateWheelCurrent(10);
                break;
            case MID_HOLD:
                setAngleCurrent(PULL_IN_ANGLE_CURRENT);
                setAngleSetpoint(-0.5, ClosedLoopSlot.kSlot1);
                updateWheelCurrent(80);
                break;
            case PUSH_OUT:
                setAngleCurrent(35);
                updateWheelCurrent(8);
                break;
            case REVERSE:
                setAngleCurrent(PULL_IN_ANGLE_CURRENT);
                setAngleCurrent(80);
                break;
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
                    setState(State.PUSH_OUT);
                    break;
            
                case PULL_IN:
                    setState(State.PUSH_OUT);
                    break;

                case MID_HOLD:
                    setState(State.PUSH_OUT);
                    break;

                case OUT:
                    setState(State.MID_HOLD);
                    break;

                case PUSH_OUT:
                    setState(State.MID_HOLD);
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
        return MathUtil.interpolate(0.5, 0.9, robotRelativeVelocity.vxMetersPerSecond/Constants.MAX_SPEED.in(MetersPerSecond)); // Standard: 0.4
    }

    public void setAngleSetpoint(double setpoint, ClosedLoopSlot slot){
        angleController.setSetpoint(setpoint, ControlType.kPosition, slot);
    }

    public Command agitateCommand(){
        return this.toggleInOut().andThen(Commands.waitSeconds(0.5)).repeatedly();
    }

    public Command reverseCommand(){
        return this.startEnd(()->{
            this.setState(State.REVERSE);
        }, ()->{
            this.setState(State.PUSH_OUT);
        });        
    }
}
