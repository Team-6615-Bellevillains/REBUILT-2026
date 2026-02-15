package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase{

    // ========================================
    // ========== Motors and Control ==========
    // ========================================

    private SparkMax spindexerMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Update CAN ID
    private SparkMax roadMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Update CAN ID

    private final RelativeEncoder spinCoder = spindexerMotor.getEncoder();
    private final RelativeEncoder roadCoder = roadMotor.getEncoder();

    private final PIDController spinPID = new PIDController(0.002, 0.0, 0.00005); // TODO: Tune!
    private final SimpleMotorFeedforward spinFF =
            new SimpleMotorFeedforward(0.07201442207491023, 0.002081727425253755); // TODO: Tune!

    private final PIDController roadPID = new PIDController(0.002, 0.0, 0.00005); // TODO: Tune!
    private final SimpleMotorFeedforward roadFF =
            new SimpleMotorFeedforward(0.07201442207491023, 0.002081727425253755); // TODO: Tune!

    private double targetSpinRPM = 0.0;
    private double targetRoadRPM = 0.0;

    public SpindexerSubsystem() {
        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.idleMode(IdleMode.kCoast);
        spindexerMotor.configure(config1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.idleMode(IdleMode.kCoast);
        roadMotor.configure(config2, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    // =======================================
    // ========== Spindexer Methods ==========
    // =======================================

    public void runSpindexerAtRPM(double rpm) {
        targetSpinRPM = rpm;

        double currentRPM = spinCoder.getVelocity();
        double pidOutput = spinPID.calculate(currentRPM, targetSpinRPM);
        double ffOutput = spinFF.calculate(targetSpinRPM);
        double voltage = pidOutput + ffOutput;

        spindexerMotor.setVoltage(voltage);
    }

    public void stopSpindexer() {
        spindexerMotor.stopMotor();
        spinPID.reset();
        targetSpinRPM = 0;
    }

    public Command spinSpindexer(double rpm) {
        return this.run(() -> runSpindexerAtRPM(rpm))
                .finallyDo(this::stopSpindexer);
    }

    public double getSpindexerRPM() {
        return spinCoder.getVelocity();
    }

    public boolean SpindexerAtSetpoint() {
        return Math.abs(getSpindexerRPM() - targetSpinRPM) < 75; // tolerance band
    }

    // ==================================
    // ========== Road Methods ==========
    // ==================================

    public void runRoadAtRPM(double rpm) {
        targetRoadRPM = rpm;

        double currentRPM = roadCoder.getVelocity();
        double pidOutput = roadPID.calculate(currentRPM, targetRoadRPM);
        double ffOutput = roadFF.calculate(targetRoadRPM);
        double voltage = pidOutput + ffOutput;

        roadMotor.setVoltage(voltage);
    }

    public void stopRoad() {
        roadMotor.stopMotor();
        roadPID.reset();
        targetRoadRPM = 0;
    }

    public Command spinRoad(double rpm) {
        return this.run(() -> runRoadAtRPM(rpm))
                .finallyDo(this::stopRoad);
    }

    public double getRoadRPM() {
        return roadCoder.getVelocity();
    }

    public boolean RoadAtSetpoint() {
        return Math.abs(getRoadRPM() - targetRoadRPM) < 75;
    }
}
