package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LinkageSubsystem.LinkageState;

public class LinkageSubsystem extends SubsystemBase {
    
    private SparkMax linkageMotor = new SparkMax(0, MotorType.kBrushless); // TODO: Update CAN ID

    public LinkageSubsystem() {
        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.idleMode(IdleMode.kCoast);
        //config2.inverted(true); // Test and uncomment if needed
        linkageMotor.configure(config2, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ===================================================
    // ========== Linkage Methods and Constants ==========
    // ===================================================

    private static final double LINKAGE_UP_POWER = 0.6;     // Strong upward force
    private static final double LINKAGE_DOWN_POWER = -0.15; // Gentle gravity-assisted down
    private static final double CURRENT_LOCK_THRESHOLD = 35.0; // Amps — tune this
    private static final double LOCK_TIME_SEC = 0.15;       // Time current must stay high
    private static final double LINKAGE_HOLD_GROUND_POWER = 0.05;
    private static final double LINKAGE_HOLD_AIR_POWER    = 0.08;

    public enum LinkageState {
        GROUND,
        AIR
    }

    private LinkageState linkageState = LinkageState.GROUND;
    private double currentSpikeStartTime = -1.0;

    private void setLinkagePower(double power) {
        linkageMotor.set(power);
    }

    private void stopLinkage() {
        linkageMotor.stopMotor();
        currentSpikeStartTime = -1.0;
    }

    private boolean isLinkageLocked() {
        double current = linkageMotor.getOutputCurrent();

        if (current >= CURRENT_LOCK_THRESHOLD) {
            if (currentSpikeStartTime < 0) {
                currentSpikeStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            }

            return (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - currentSpikeStartTime)
                    >= LOCK_TIME_SEC;
        } else {
            currentSpikeStartTime = -1.0;
            return false;
        }
    }

    public Command moveLinkageUp() {
    return this.run(() -> setLinkagePower(LINKAGE_UP_POWER))
        .until(this::isLinkageLocked)
        .finallyDo(() -> {
            linkageState = LinkageState.AIR;
        });
}

    public Command moveLinkageDown() {
        return this.run(() -> setLinkagePower(LINKAGE_DOWN_POWER))
            .until(this::isLinkageLocked)
            .finallyDo(() -> {
                stopLinkage();
                linkageState = LinkageState.GROUND;
            });
    }

    public Command toggleLinkage() {
        return this.defer(() ->
            linkageState == LinkageState.GROUND
                ? moveLinkageUp()
                : moveLinkageDown()
        );
    }

    public void holdLinkage() {
        switch (linkageState) {
            case GROUND:
                setLinkagePower(LINKAGE_HOLD_GROUND_POWER);
                break;

            case AIR:
                setLinkagePower(LINKAGE_HOLD_AIR_POWER);
                break;
        }
    }
}
