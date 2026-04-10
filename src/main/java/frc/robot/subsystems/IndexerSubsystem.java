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
    private final Supplier<Boolean> isInAllianceZone;

    // Burst mode tuning
    private static final double BURST_FEED_DURATION = 0.1;
    private static final double BURST_WAIT_DURATION = 0.25;
    private static final double BURST_DISTANCE_THRESHOLD_FEET = 16.0;

    // Stall detection tuning
    private static final double SPIN_STALL_RPM_THRESHOLD = 250.0;
    private static final double STALL_TRIGGER_DURATION = 0.2;
    private static final double STALL_REVERSE_DURATION = 0.2;

    private double stallTimer = 0;
    private double stallReverseTimer = 0;
    private boolean isStallReversing = false;

    private double burstTimer = 0;
    private boolean isBurstFeeding = true;

    public IndexerSubsystem(Supplier<Double> getHubDistanceFeet, Supplier<Boolean> isInAllianceZone) {
        this.getHubDistanceFeet = getHubDistanceFeet;
        this.isInAllianceZone = isInAllianceZone;

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

        SmartDashboard.putString("indexer state", state.toString());
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
                } else if (isInAllianceZone.get() && getHubDistanceFeet.get() > BURST_DISTANCE_THRESHOLD_FEET) {
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

        SmartDashboard.putNumber("spindexer rpm", spindexerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("spindexer current", spindexerMotor.getOutputCurrent());
        SmartDashboard.putNumber("road rpm", roadMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("road current", roadMotor.getOutputCurrent());
        SmartDashboard.putNumber("spindexer stall timer", stallTimer);
        SmartDashboard.putBoolean("spindexer stall reversing", isStallReversing);
        SmartDashboard.putNumber("burst timer", burstTimer);
        SmartDashboard.putBoolean("burst feeding", isBurstFeeding);
        SmartDashboard.putNumber("hub distance feet", getHubDistanceFeet.get());
    }

    private void checkSpindexerStall() {
        if (state == State.OFF || state == State.REVERSE) {
            stallTimer = 0;
            return;
        }

        if (state == State.SHOOT && !isBurstFeeding && isInAllianceZone.get() && getHubDistanceFeet.get() > BURST_DISTANCE_THRESHOLD_FEET) {
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