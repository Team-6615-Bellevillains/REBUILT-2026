package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        .withClosedLoopController(0.02, 0, 0)
        .withSimClosedLoopController(0, 0, 0)
        // feedforward constants
        .withFeedforward(new SimpleMotorFeedforward(0.1546, 0.125, 0.0))
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

    private SmartMotorController shooterController = new SparkWrapper(shooterLeft, DCMotor.getNEO(1), smcConfig);

    private FlyWheelConfig shooterConfig = new FlyWheelConfig(shooterController)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(3))
        // max speed
        .withUpperSoftLimit(RPM.of(2000))
        .withTelemetry("Shooter Mech", TelemetryVerbosity.HIGH)
        .withSoftLimit(RPM.of(-5000), RPM.of(5000));

    private FlyWheel shooter = new FlyWheel(shooterConfig);

    public ShooterSubsystem(){
        shooter = new FlyWheel(shooterConfig);
        setPoint = RPM.of(0);
        SmartDashboard.putNumber("rpm to run", 0);
    }

    @Override
    public void periodic() {
        shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
    }

    public void setPoint(AngularVelocity velocity){
        shooter.setMechanismVelocitySetpoint(velocity);
        setPoint = velocity;
    }

    public AngularVelocity getVelocity(){
        return shooter.getSpeed();
    }

    public BooleanSupplier nearVelocity(AngularVelocity velocity){
        return shooter.isNear(velocity, RPM.of(120));
    }

    public void stop(){
        setPoint = RPM.of(0);
        setPoint(setPoint);
    }

    public Command stopCommand(){
        return shooter.set(0);
    }

    public Command liveRPMCommand(){
        return this.run(()->{
            setPoint(RPM.of(SmartDashboard.getNumber("rpm to run", 0)));
        });
    }

    public Command sysId() {return shooter.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));}
}