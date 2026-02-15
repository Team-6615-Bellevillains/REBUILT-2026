package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    // ========================================
    // ========== Motors and Control ==========
    // ========================================

    private SparkMax leftMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Update CAN ID
    private SparkMax rightMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Update CAN ID
    
    private final RelativeEncoder encoder = leftMotor.getEncoder();

    private final PIDController pid = new PIDController(0.002, 0.0, 0.00005); // TODO: Tune!
    private final SimpleMotorFeedforward ff =
            new SimpleMotorFeedforward(0.07201442207491023, 0.002081727425253755); // TODO: Tune!

    private double targetRPM = 0.0;

    public ShooterSubsystem() {
        // First motor config
        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.idleMode(IdleMode.kCoast);
        leftMotor.configure(config1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Second motor config — REVERSED
        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.idleMode(IdleMode.kCoast);
        config2.inverted(true);
        rightMotor.configure(config2, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    // =====================================
    // ========== Shooter Methods ==========
    // =====================================

    public void runAtRPM(double rpm) {
        targetRPM = rpm;

        double currentRPM = encoder.getVelocity();
        double pidOutput = pid.calculate(currentRPM, targetRPM);
        double ffOutput = ff.calculate(targetRPM);

        double voltage = pidOutput + ffOutput;

        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        pid.reset();
        targetRPM = 0;
    }

    public Command spinShooter(double rpm) {
        return this.run(() -> runAtRPM(rpm))
                .finallyDo(this::stop);
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    public boolean atSetpoint() {
        return Math.abs(getRPM() - targetRPM) < 75;
    }
}
