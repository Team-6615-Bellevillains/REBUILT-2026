package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase{

    private SparkMax shooterLeft = new SparkMax(12, MotorType.kBrushless);
    private SparkMax shooterRight = new SparkMax(10, MotorType.kBrushless);

    private AngularVelocity setPoint = RPM.of(0);

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // feedback constants
        .withClosedLoopController(0.3, 0, 0)
        .withSimClosedLoopController(0.3, 0, 0)
        // feedforward constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // telemetry
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        // gearing
        .withGearing(1)
        // motor properties; prevent overcurrenting
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withFollowers(Pair.of(shooterRight, true));

    private SmartMotorController turretController = new SparkWrapper(shooterLeft, DCMotor.getNEO(1), smcConfig);

    private FlyWheelConfig shooterConfig = new FlyWheelConfig(turretController)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(1))
        // max speed
        .withUpperSoftLimit(RPM.of(2000))
        .withTelemetry("Shooter Mech", TelemetryVerbosity.HIGH);

    private FlyWheel shooter = new FlyWheel(shooterConfig);

    public ShooterSubsystem(){
        shooter = new FlyWheel(shooterConfig);
        setPoint = RPM.of(0);
    }

    @Override
    public void periodic() {
        shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
    }

    public void RpmSetPoint(double rpm){
        AngularVelocity velocity = RPM.of(rpm);
        shooter.setMechanismVelocitySetpoint(velocity);
        setPoint = velocity;
    }

    public AngularVelocity getVelocity(){
        return shooter.getSpeed();
    }

    private BooleanSupplier atSetpoint = shooter.isNear(setPoint, RPM.of(20));
    public boolean atSetpoint(){
        return atSetpoint.getAsBoolean();
    }

    public void stop(){
        shooter.set(0);
    }
}