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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {

    private static final double GEAR_RATIO = 10.0;
    private static final double MIN_ANGLE = -270;
    private static final double MAX_ANGLE =  0;

    // Offset this if forward direction is off after homing
    private static final double HOMING_OFFSET = -270; // TODO: Tune after first homing test

    private static final double SOFT_LIMIT_BUFFER       =  5.0;
    private static final double OUTPUT_LIMIT            =  0.5;
    private static final double ANGLE_TOLERANCE         =  0.0;

    // Enable the robot, call setTargetAngle(-45.0), and watch Elastic "Turret/CurrentAngle".
    private static final double kP = 0.005; // TODO: Tune: start at 0.01
    private static final double kI = 0.0;  // TODO: Tune: leave at 0.0 until P and D are done
    private static final double kD = 0.0002; // TODO: Tune: add small amounts to reduce oscillation

    // If homing triggers too early (before hitting the stop): increase this value
    // If homing never triggers (misses the stop): decrease this value
    private static final double STALL_CURRENT_THRESHOLD = 20.0; // TODO: Tune with Elastic
    private static final double STALL_DEBOUNCE_TIME     = 0.1;  // seconds — increase if false triggers

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

    // Stall detection debounce
    private final Timer stallTimer        = new Timer();
    private boolean     stallTimerRunning = false;

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
            .inverted(false) // TODO: Change to true if positive angles go the wrong direction
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        config.encoder
            .positionConversionFactor(360.0 / GEAR_RATIO)
            .velocityConversionFactor((360.0 / GEAR_RATIO) / 60.0);
        config.closedLoop
            .p(kP).i(kI).d(kD)
            .outputRange(-OUTPUT_LIMIT, OUTPUT_LIMIT);
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
                closedLoop.setSetpoint(0.0, ControlType.kPosition);
                break;
            case TRACKING:
                closedLoop.setSetpoint(
                    MathUtil.clamp(targetAngle, MIN_ANGLE, MAX_ANGLE),
                    ControlType.kPosition);
                shootAllowed = isTargetReachable(targetAngle);
                break;
        }
        filteredCurrent = currentFilter.calculate(motor.getOutputCurrent());
        publishTelemetry();
    }

    // Calculates robot-relative angle to hub and commands the turret
    public void aimAtHub() {
        if (!isHomed()) return;

        Pose2d robot       = robotPoseSupplier.get();
        Translation2d hub  = getActiveHub();
        Translation2d target = hub.minus(robot.getTranslation());
        Rotation2d fieldAngle  = target.getAngle();
        Rotation2d robotAngle = fieldAngle.minus(robot.getRotation());
        double turretAngle = MathUtil.inputModulus(robotAngle.getDegrees(), 0, 360);
        setTargetAngle(-turretAngle);
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
        motor.set(-HOMING_SPEED); // STEP 6: Flip HOMING_SPEED sign if going wrong direction
        if (isStalled()) {
            motor.stopMotor();
            encoder.setPosition(0.0 + HOMING_OFFSET); // forward stop = 0° + any trim
            resetStallTimer();
            applySoftLimits(true);
            state = TurretState.HOMING_TO_CENTER;
        }
    }

    // Homing phase 2 — closed-loop back to center (0°), then go HOMED
    private void runHomingToCenter() {
        closedLoop.setSetpoint(0, ControlType.kPosition);
        targetAngle=0;
        if (Math.abs(encoder.getPosition()) < 2)
            state = TurretState.HOMED;
    }

    // Enables or disables soft limits without wiping other config
    private void applySoftLimits(boolean enabled) {
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.softLimit.forwardSoftLimitEnabled(enabled).reverseSoftLimitEnabled(enabled);
        motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Returns true once current exceeds threshold for STALL_DEBOUNCE_TIME seconds
    private boolean isStalled() {
        // if (motor.getOutputCurrent() > STALL_CURRENT_THRESHOLD) {
        //     if (!stallTimerRunning) {
        //         stallTimer.reset();
        //         stallTimer.start();
        //         stallTimerRunning = true;
        //     }
        //     return stallTimer.hasElapsed(STALL_DEBOUNCE_TIME);
        // }
        // resetStallTimer();
        // return false;
        return filteredCurrent > STALL_CURRENT_THRESHOLD;
    }

    private void resetStallTimer() {
        stallTimer.stop();
        stallTimer.reset();
        stallTimerRunning = false;
    }

    // Public API

    public void rehome() {
        applySoftLimits(false);
        resetStallTimer();
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

    /*
     *  BEFORE ENABLING:
     * 
     *  1.      Make sure the gear ratio is right 
     * 
     *  2.      Make sure Utils.getHubCenter() returns the correct Translation2d
     *          for both Alliance.Blue and Alliance.Red.
     * 
     *  3.      Make sure RobotContainer passes in the drive pose supplier and
     *          calls turret.rehome() on enable.
     * 
     *  4.      Open Elastic and add widgets for:
     *          Turret/State, Turret/CurrentAngle, Turret/MotorCurrent,
     *          Turret/IsHomed, Turret/AtTarget, Turret/CanShoot
     *
     *  HOMING TESTING:
     * 
     *  5.      Enable the robot. Watch Turret/State in Elastic.
     *          It should say HOMING_TO_MIN while moving slowly.
     *          If the turret moves away from the forward stop, flip the sign
     *          on HOMING_SPEED
     *
     *  6.      Watch Turret/MotorCurrent as it hits the stop.
     *          Set STALL_CURRENT_THRESHOLD between the free current and the stall spike.
     *          If homing triggers too early: increase STALL_CURRENT_THRESHOLD.
     *          If homing never triggers: decrease STALL_CURRENT_THRESHOLD.
     *          There's even the debounce timer to mess with if needed (STALL_DEBOUNCE_TIME).
     *
     *  7.      Once homing completes (State = HOMED), physically look at the turret.
     *          It should be pointing directly forward on the robot.
     *          If it is slightly off, that means the hard stop is mounted slightly off.
     *          Set HOMING_OFFSET to nudge it.
     *
     *  DIRECTION TESTING:
     * 
     *  8.      After homing succeeds, call setTargetAngle(-45.0) from a button.
     *          The turret should rotate to the left (counterclockwise from above).
     *          If it goes right instead, change inverted(false) to inverted(true)
     *          in the constructor and re-deploy.
     *
     *  9.      Test setTargetAngle(-45.0). Turret should go left.
     *          Test setTargetAngle(0.0). Turret should return to forward.
     *
     *  PID TUNING:
     * 
     *  10.     Start with kP = 0.01, kI = 0.0, kD = 0.0.
     *          Increase kP slowly until the turret moves confidently to the target.
     *          If it oscillates, back kP off and add a small kD (e.g. 0.001).
     *          Only add kI if the turret consistently stops a few degrees short.
     *
     *  AIMING TEST:
     * 
     *  11.     Put the robot on the field. Call aimAtHub().
     *          If it points 180° off, the robot heading convention may be flipped —
     *          check how rotation is reported (CCW positive is standard).
     */
}