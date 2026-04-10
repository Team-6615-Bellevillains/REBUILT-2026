package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    
    private State state = State.IN;
    private SparkFlex angleMotor = new SparkFlex(20, MotorType.kBrushless);
    private SparkFlex speedMotor = new SparkFlex(22, MotorType.kBrushless);
    private SparkFlex wheelMotor = new SparkFlex(24, MotorType.kBrushless);
    private SparkClosedLoopController angleController = angleMotor.getClosedLoopController();
    private static final int PULL_IN_ANGLE_CURRENT = 30;
    private MedianFilter angleCurrentFilter = new MedianFilter(25);
    private double filteredAngleCurrent = 0;
    private int nonLimitedAngleCurrent = PULL_IN_ANGLE_CURRENT;
    private boolean shouldRunWheelsInIntakeDirection = false;
    private final Supplier<ChassisSpeeds> getRobotRelativeVelocity;
    private final double IN_WHEEL_DUTY_CYCLE = -0.20;
    private final SparkClosedLoopController wheelController = wheelMotor.getClosedLoopController();

    private static final double WHEEL_STALL_RPM_THRESHOLD = 5.0;
    private static final double STALL_TRIGGER_DURATION = 0.25;
    private static final double STALL_REVERSE_DURATION = 0.25;

    private double stallTimer = 0;
    private double stallReverseTimer = 0;
    private boolean isStallReversing = false;

    // Fast agitate tuning
    private static final double FAST_AGITATE_UP_DUTY    =  0.6;
    private static final double FAST_AGITATE_DOWN_DUTY  = -0.6;
    private static final double FAST_AGITATE_DRIVE_TIME =  0.15;
    private static final double FAST_AGITATE_PAUSE_TIME =  0.05;

    private final Timer fastAgitateTimer = new Timer();
    private int   fastAgitatePhase = 0; // 0=drive up, 1=pause, 2=drive down, 3=pause
    private State stateBeforeFastAgitate = State.MID_HOLD;

    public IntakeSubsystem(Supplier<ChassisSpeeds> getRobotRelativeVelocity){
        SparkFlexConfig angleMotorConfig = new SparkFlexConfig();
        angleMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.smartCurrentLimit(1);
        angleMotorConfig.closedLoop.pid(0.2,0,0, ClosedLoopSlot.kSlot0);
        angleMotorConfig.closedLoop.pid(0.15,0,0, ClosedLoopSlot.kSlot1);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig wheelMotorConfig = new SparkFlexConfig();
        wheelMotorConfig.idleMode(IdleMode.kCoast);
        wheelMotorConfig.smartCurrentLimit(60);
        wheelMotorConfig.closedLoop.pid(0.0002, 0, 0)
        .feedForward.kS(0.18).kV(0.0026);
        wheelMotorConfig.encoder.velocityConversionFactor((25.0/36.0));
        wheelMotor.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.getRobotRelativeVelocity = getRobotRelativeVelocity;

        SparkFlexConfig speedMotorConfig = new SparkFlexConfig();
        speedMotorConfig.follow(24);
        speedMotorConfig.smartCurrentLimit(60);
        speedMotor.configure(speedMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        angleMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        filteredAngleCurrent = angleCurrentFilter.calculate(angleMotor.getOutputCurrent());

        checkWheelStall();

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
                if (isStallReversing) {
                    wheelMotor.set(-0.5);
                } else if (shouldRunWheelsInIntakeDirection) {
                    wheelController.setSetpoint(2200, ControlType.kVelocity);
                } else {
                    wheelMotor.stopMotor();
                }
                break;
            
            case MID_HOLD:
                midHold();
                if (isStallReversing) {
                    wheelMotor.set(-0.5);
                } else if (shouldRunWheelsInIntakeDirection) {
                    wheelController.setSetpoint(2200, ControlType.kVelocity);
                } else {
                    wheelMotor.stopMotor();
                }
                break;

            case PUSH_OUT:
                pushOutPeriodic();
                if (isStallReversing) {
                    wheelMotor.set(-0.5);
                } else if (shouldRunWheelsInIntakeDirection) {
                    wheelController.setSetpoint(2200, ControlType.kVelocity);
                } else {
                    wheelMotor.stopMotor();
                }
                if (angleMotor.getEncoder().getPosition() < -3.2) {
                    setState(State.OUT);
                }
                break;
                
            case REVERSE:
                wheelMotor.set(-0.5);
                break;

            case FAST_AGITATE:
                fastAgitatePeriodic();
                wheelMotor.stopMotor();
                break;
        }
        SmartDashboard.putNumber("angle motor current", angleMotor.getOutputCurrent());
        SmartDashboard.putString("Intake state", state.toString());
        SmartDashboard.putNumber("filtered current", filteredAngleCurrent);
        SmartDashboard.putNumber("intake rpm", wheelMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("wheel varying duty cycle", getActiveWheelDutyCycle());
        SmartDashboard.putNumber("intake leader current", wheelMotor.getOutputCurrent());
        SmartDashboard.putNumber("intake follower current", speedMotor.getOutputCurrent());
        SmartDashboard.putNumber("stall timer", stallTimer);
        SmartDashboard.putBoolean("stall reversing", isStallReversing);
        SmartDashboard.putNumber("fast agitate phase", fastAgitatePhase);
    }

    private void fastAgitatePeriodic() {
        switch (fastAgitatePhase) {
            case 0: // drive up
                angleMotor.set(FAST_AGITATE_UP_DUTY);
                if (fastAgitateTimer.hasElapsed(FAST_AGITATE_DRIVE_TIME)) {
                    fastAgitatePhase = 1;
                    fastAgitateTimer.reset();
                }
                break;
            case 1: // pause after up
                angleMotor.set(0);
                if (fastAgitateTimer.hasElapsed(FAST_AGITATE_PAUSE_TIME)) {
                    fastAgitatePhase = 2;
                    fastAgitateTimer.reset();
                }
                break;
            case 2: // drive down
                angleMotor.set(FAST_AGITATE_DOWN_DUTY);
                if (fastAgitateTimer.hasElapsed(FAST_AGITATE_DRIVE_TIME)) {
                    fastAgitatePhase = 3;
                    fastAgitateTimer.reset();
                }
                break;
            case 3: // pause after down
                angleMotor.set(0);
                if (fastAgitateTimer.hasElapsed(FAST_AGITATE_PAUSE_TIME)) {
                    fastAgitatePhase = 0;
                    fastAgitateTimer.reset();
                }
                break;
        }
    }

    private void checkWheelStall() {
        if (state == State.REVERSE || state == State.PUSH_OUT || state == State.FAST_AGITATE) return;

        boolean wheelsCommanded = shouldRunWheelsInIntakeDirection && 
            (state == State.OUT || state == State.MID_HOLD);
        if (!wheelsCommanded) {
            stallTimer = 0;
            return;
        }

        if (isStallReversing) {
            stallReverseTimer += 0.02;
            if (stallReverseTimer >= STALL_REVERSE_DURATION) {
                isStallReversing = false;
                stallReverseTimer = 0;
                stallTimer = 0;
            }
            return;
        }

        double rpm = Math.abs(wheelMotor.getEncoder().getVelocity());
        if (rpm < WHEEL_STALL_RPM_THRESHOLD) {
            stallTimer += 0.02;
            if (stallTimer >= STALL_TRIGGER_DURATION) {
                isStallReversing = true;
                stallReverseTimer = 0;
                stallTimer = 0;
            }
        } else {
            stallTimer = 0;
        }
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
        REVERSE,
        FAST_AGITATE
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
                updateWheelCurrent(60);
                break;
            case REVERSE:
                setAngleCurrent(PULL_IN_ANGLE_CURRENT);
                setAngleCurrent(80);
                break;
            case FAST_AGITATE:
                setAngleCurrent(60);
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
                    break;

                default:
                    break;
            }
        });
    }

    public Command setStateCommand(State state){
        return this.runOnce(()->setState(state));
    }

    private void setAngleCurrent(int amps){
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(amps);
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
        return this.setWheelsCommand(true).andThen((toggleInOut().andThen(Commands.waitSeconds(0.5)).repeatedly())).finallyDo(()->{shouldRunWheelsInIntakeDirection = false;});
    }

    public Command reverseCommand(){
        return this.startEnd(()->{
            this.setState(State.REVERSE);
        }, ()->{
            this.setState(State.PUSH_OUT);
        });        
    }

    public Command fastAgitateCommand() {
        return this.startEnd(
            () -> {
                stateBeforeFastAgitate = state;
                fastAgitatePhase = 0;
                fastAgitateTimer.reset();
                fastAgitateTimer.start();
                setState(State.FAST_AGITATE);
            },
            () -> setState(stateBeforeFastAgitate)
        );
    }
}