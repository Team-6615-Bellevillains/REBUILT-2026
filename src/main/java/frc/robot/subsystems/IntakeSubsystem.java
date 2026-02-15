package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//put code for the intake in here.
public class IntakeSubsystem extends SubsystemBase {

    // ========================================
    // ========== Motors and Control ==========
    // ========================================

    private SparkMax intakeMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Update CAN ID
    private final RelativeEncoder intakeCoder = intakeMotor.getEncoder();

    private final PIDController intakePID = new PIDController(0.002, 0.0, 0.00005); // TODO: Tune!
    private final SimpleMotorFeedforward intakeFF =
            new SimpleMotorFeedforward(0.07201442207491023, 0.002081727425253755); // TODO: Tune!

    private double targetRPM = 0.0;

    public IntakeSubsystem() {
        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.idleMode(IdleMode.kCoast);
        //config1.inverted(true); // Test and uncomment if needed
        intakeMotor.configure(config1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ====================================
    // ========== Intake Methods ==========
    // ====================================

    public void runIntakeAtRPM(double rpm) {
        targetRPM = rpm;

        double currentRPM = intakeCoder.getVelocity();
        double pidOutput = intakePID.calculate(currentRPM, targetRPM);
        double ffOutput = intakeFF.calculate(targetRPM);

        double voltage = pidOutput + ffOutput;

        intakeMotor.setVoltage(voltage);
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
        intakePID.reset();
        targetRPM = 0;
    }

    public Command spinIntake(double rpm) {
        return this.run(() -> runIntakeAtRPM(rpm))
                .finallyDo(this::stopIntake);
    }

    public double getIntakeRPM() {
        return intakeCoder.getVelocity();
    }

    public boolean atSetpoint() {
        return Math.abs(getIntakeRPM() - targetRPM) < 75; // tolerance band
    }
}
