package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class IndexerSubsystem extends SubsystemBase {
    
    private final SparkMax spindexerMotor = new SparkMax(50, MotorType.kBrushless);
    private final SparkMax roadMotor = new SparkMax(52, MotorType.kBrushless);
    private final SparkClosedLoopController spinController = spindexerMotor.getClosedLoopController();
    private final SparkClosedLoopController roadController = roadMotor.getClosedLoopController();
    private State state = State.OFF;
    private final Supplier<Double> getHubDistanceFeet;

    // Burst mode tuning
    private static final double BURST_FEED_DURATION = 0.3;
    private static final double BURST_WAIT_DURATION = 0.7;
    private static final double BURST_DISTANCE_THRESHOLD_FEET = 14.0;

    // Stall detection tuning
    private static final double SPIN_STALL_RPM_THRESHOLD = 5.0;
    private static final double STALL_TRIGGER_DURATION = 0.25;
    private static final double STALL_REVERSE_DURATION = 0.25;

    private double stallTimer = 0;
    private double stallReverseTimer = 0;
    private boolean isStallReversing = false;

    private double burstTimer = 0;
    private boolean isBurstFeeding = true;

    public IndexerSubsystem(Supplier<Double> getHubDistanceFeet){
        this.getHubDistanceFeet = getHubDistanceFeet;

        SparkMaxConfig spinConfig = new SparkMaxConfig();
        SparkMaxConfig roadConfig = new SparkMaxConfig();
        
        spinConfig.closedLoop
        .pid(0, 0, 0)
        .feedForward
        .kS(0.14)
        .kA(0)
        .kV(0.00202);

        roadConfig.closedLoop
        .pid(0, 0, 0)
        .feedForward
        .kS(0.25)
        .kA(0)
        .kV(0.00213);

        spinConfig.inverted(true);
        spindexerMotor.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        roadConfig.inverted(false);
        roadMotor.configure(roadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        checkSpindexerStall();

        switch (state) {
            case OFF:
                off();
                break;
        
            case INDEX:
                if (isStallReversing) {
                    spinController.setSetpoint(-3000, ControlType.kVelocity);
                } else {
                    index();
                }
                break;
            
            case SHOOT:
                if (isStallReversing) {
                    spinController.setSetpoint(-3000, ControlType.kVelocity);
                    roadController.setSetpoint(3000, ControlType.kVelocity);
                } else if (getHubDistanceFeet.get() > BURST_DISTANCE_THRESHOLD_FEET) {
                    shootBurst();
                } else {
                    shoot();
                }
                break;
                
            case SLOW:
                if (isStallReversing) {
                    spinController.setSetpoint(-3000, ControlType.kVelocity);
                } else {
                    slow();
                }
                roadMotor.stopMotor();
                break;
            
            case REVERSE:
                reverse();
                break;
        }
    }

    private void checkSpindexerStall() {
        if (state == State.OFF || state == State.REVERSE) {
            stallTimer = 0;
            return;
        }

        if (state == State.SHOOT && !isBurstFeeding && getHubDistanceFeet.get() > BURST_DISTANCE_THRESHOLD_FEET) {
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

        double rpm = Math.abs(spindexerMotor.getEncoder().getVelocity());
        if (rpm < SPIN_STALL_RPM_THRESHOLD) {
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

    private void shootBurst() {
        burstTimer += 0.02;
        roadController.setSetpoint(3000, ControlType.kVelocity);

        if (isBurstFeeding) {
            spinController.setSetpoint(3000, ControlType.kVelocity);
            if (burstTimer >= BURST_FEED_DURATION) {
                isBurstFeeding = false;
                burstTimer = 0;
            }
        } else {
            spindexerMotor.stopMotor();
            if (burstTimer >= BURST_WAIT_DURATION) {
                isBurstFeeding = true;
                burstTimer = 0;
            }
        }
    }

    private void shoot(){
        burstTimer = 0;
        isBurstFeeding = true;
        spinController.setSetpoint(3000, ControlType.kVelocity);
        roadController.setSetpoint(3000, ControlType.kVelocity);
    }

    private void index(){
        spinController.setSetpoint(3000, ControlType.kVelocity);
        roadMotor.stopMotor();
    }

    private void off(){
        spindexerMotor.stopMotor();
        roadMotor.stopMotor();
    }

    private void slow(){
        spinController.setSetpoint(500, ControlType.kVelocity);
        roadMotor.stopMotor();
    }

    public void setState(State state){
        this.state = state;
    }

    private void reverse(){
        spinController.setSetpoint(-3000, ControlType.kVelocity);
        roadMotor.stopMotor();
    }

    public enum State {
        OFF,
        INDEX,
        SHOOT,
        SLOW,
        REVERSE
    }

    public Command indexerRunCommand(){
        return this.runEnd(()->{
            this.setState(State.SLOW);
        }, ()->{
            this.setState(State.OFF);
        });
    }

    public Command indexerReverseCommand(){
        return this.runEnd(()->{
            this.setState(State.REVERSE);
        }, ()->{
            this.setState(State.OFF);
        });
    }
}