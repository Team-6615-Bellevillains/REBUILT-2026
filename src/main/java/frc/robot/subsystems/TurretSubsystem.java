package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;

import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {

    private static final double GEAR_RATIO = 10.0;
    private static final double MIN_ANGLE = -270;
    private static final double MAX_ANGLE =  0;

    private static final double HOMING_OFFSET = -270;

    private static final double SOFT_LIMIT_BUFFER       =  5.0;
    private static final double OUTPUT_LIMIT            =  0.5;
    private static final double ANGLE_TOLERANCE         =  0.1;

    private static final double kP = 0.005;
    private static final double kI = 0.0;  
    private static final double kD = 0.0002; 

    private static final double STALL_CURRENT_THRESHOLD = 20.0;
    private static final double STALL_DEBOUNCE_TIME     = 0.1;

    private static final double HOMING_SPEED = 0.045;

    private final MedianFilter currentFilter = new MedianFilter(2*(int)((STALL_DEBOUNCE_TIME*1000)/20));


    // Hardware
    private final SparkFlex                 motor;
    private final SparkClosedLoopController closedLoop;
    private final RelativeEncoder           encoder;

    // Robot pose supplier from RobotContainer
    private final Supplier<Pose2d> robotPoseSupplier;

    // State machine
    private enum TurretState { HOMING_TO_MIN, HOMING_TO_CENTER, HOMED, TRACKING }
    private TurretState state          = TurretState.HOMING_TO_MIN;
    private double      targetAngle    = 0;
    private boolean     shootAllowed   = false;


    // Last known hub — fallback if alliance data drops mid-match
    private Translation2d lastKnownHub = Utils.getHubCenter(DriverStation.Alliance.Blue);

    // Elastic telemetry publishers under "Turret/"
    private final DoublePublisher  ntCurrentAngle;
    private final DoublePublisher  ntTargetAngle;
    private final DoublePublisher  ntMotorCurrent;
    private final DoublePublisher  ntFilteredCurret;
    private final DoublePublisher  ntDistanceToHub;
    private final BooleanPublisher ntIsHomed;
    private final BooleanPublisher ntAtTarget;
    private final BooleanPublisher ntCanShoot;
    private final StringPublisher  ntState;
    private final StringPublisher  ntActiveHub;

    private double filteredCurrent;

    public TurretSubsystem(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;

        motor = new SparkFlex(14, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        config.encoder
            .positionConversionFactor(360.0 / GEAR_RATIO)
            .velocityConversionFactor((360.0 / GEAR_RATIO) / 60.0);
        config.closedLoop
            .p(kP).i(kI).d(kD)
            .outputRange(-OUTPUT_LIMIT, OUTPUT_LIMIT)
            .feedForward
                .kS(0.26);
        config.softLimit
            .forwardSoftLimit((float)(MAX_ANGLE - SOFT_LIMIT_BUFFER))
            .reverseSoftLimit((float)(MIN_ANGLE + SOFT_LIMIT_BUFFER))
            .forwardSoftLimitEnabled(false)  // disabled until homing gives us a valid position
            .reverseSoftLimitEnabled(false);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        closedLoop = motor.getClosedLoopController();
        encoder    = motor.getEncoder();

        var table       = NetworkTableInstance.getDefault().getTable("Turret");
        ntCurrentAngle  = table.getDoubleTopic("CurrentAngle").publish();
        ntTargetAngle   = table.getDoubleTopic("TargetAngle").publish();
        ntMotorCurrent  = table.getDoubleTopic("MotorCurrent").publish();
        ntDistanceToHub = table.getDoubleTopic("DistanceToHub").publish();
        ntIsHomed       = table.getBooleanTopic("IsHomed").publish();
        ntAtTarget      = table.getBooleanTopic("AtTarget").publish();
        ntCanShoot      = table.getBooleanTopic("CanShoot").publish();
        ntState         = table.getStringTopic("State").publish();
        ntActiveHub     = table.getStringTopic("ActiveHub").publish();
        ntFilteredCurret= table.getDoubleTopic("FilteredCurret").publish();
    }

    @Override
    public void periodic() {
        switch (state) {
            case HOMING_TO_MIN:    runHomingToMin();    break;
            case HOMING_TO_CENTER: runHomingToCenter(); break;
            case HOMED:
                closedLoop.setSetpoint(-180, ControlType.kPosition);
                ifAtSetpointTurnOff();
                break;
            case TRACKING:
                closedLoop.setSetpoint(
                    MathUtil.clamp(targetAngle, MIN_ANGLE, MAX_ANGLE),
                    ControlType.kPosition);
                shootAllowed = isTargetReachable(targetAngle);
                ifAtSetpointTurnOff();
                break;
        }
        filteredCurrent = currentFilter.calculate(motor.getOutputCurrent());
        publishTelemetry();
        
    }

    public void ifAtSetpointTurnOff(){
        if (atTarget()){
            motor.set(0);
        }
    }

    // Calculates robot-relative angle to hub and commands the turret
    public void aimAt(Translation2d target) {
        if (!isHomed()) return;
        Translation2d robot = Utils.calculateTurretTranslation(robotPoseSupplier.get());
        Translation2d diff = target.minus(robot);
        Rotation2d fieldAngle = diff.getAngle();
        Rotation2d robotAngle = fieldAngle.minus(robotPoseSupplier.get().getRotation());
        double turretAngle = MathUtil.inputModulus(robotAngle.getDegrees(), 0, 360);
        setTargetAngle(-turretAngle);
    }

    public void aimAtHub() {
        aimAt(getActiveHub());
    }

    // Returns active hub, falls back to last known if FMS drops
    private Translation2d getActiveHub() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            lastKnownHub = Utils.getHubCenter(alliance.get());
        }
        return lastKnownHub;
    }

    // Homing phase 1 — crawl toward forward hard stop, zero encoder on stall
    private void runHomingToMin() {
        motor.set(-HOMING_SPEED);
        if (isStalled()) {
            motor.stopMotor();
            encoder.setPosition(0.0 + HOMING_OFFSET); // forward stop = 0° + any trim
            applySoftLimits(true);
            state = TurretState.HOMING_TO_CENTER;
        }
    }

    // Homing phase 2 — closed-loop back to center (0°), then go HOMED
    private void runHomingToCenter() {
        closedLoop.setSetpoint(-180, ControlType.kPosition);
        targetAngle=-180;
        if (Math.abs(-180-encoder.getPosition()) < 2)
            state = TurretState.HOMED;
    }

    // Enables or disables soft limits without wiping other config
    private void applySoftLimits(boolean enabled) {
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.softLimit.forwardSoftLimitEnabled(enabled).reverseSoftLimitEnabled(enabled);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Returns true once current exceeds threshold for STALL_DEBOUNCE_TIME seconds
    private boolean isStalled() {
        return filteredCurrent > STALL_CURRENT_THRESHOLD;
    }

    // Public API

    public void rehome() {
        applySoftLimits(false);
        shootAllowed = false;
        state = TurretState.HOMING_TO_MIN;
    }

    public void setTargetAngle(double degrees) {
        if (!isHomed()) return;
        targetAngle = degrees;
        state = TurretState.TRACKING;
    }

    public double  getDistanceToHub()            { return robotPoseSupplier.get().getTranslation().getDistance(getActiveHub()); }
    public boolean canShoot()                    { return isHomed() && shootAllowed; }
    public boolean isTargetReachable(double deg) { return deg >= MIN_ANGLE && deg <= MAX_ANGLE; }
    public double  getCurrentAngle()             { return encoder.getPosition(); }
    public boolean isHomed()                     { return state == TurretState.HOMED || state == TurretState.TRACKING; }
    public boolean atTarget()                    { return isHomed() && Math.abs(getCurrentAngle() - targetAngle) < ANGLE_TOLERANCE; }

    private void publishTelemetry() {
        ntCurrentAngle.set(getCurrentAngle());
        ntTargetAngle.set(targetAngle);
        ntMotorCurrent.set(motor.getOutputCurrent());
        ntDistanceToHub.set(getDistanceToHub());
        ntIsHomed.set(isHomed());
        ntAtTarget.set(atTarget());
        ntCanShoot.set(canShoot());
        ntState.set(state.toString());
        ntActiveHub.set(getActiveHub().equals(Utils.getHubCenter(DriverStation.Alliance.Blue)) ? "BLUE" : "RED");
        ntFilteredCurret.set(currentFilter.lastValue());
    }
}