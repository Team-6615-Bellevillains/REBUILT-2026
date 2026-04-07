package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LoggerSubsystem extends SubsystemBase {
    private final CommandXboxController driveController;
    private final CommandXboxController opController;
    
    public LoggerSubsystem(CommandXboxController driveController, CommandXboxController opController) {
        this.driveController = driveController;
        this.opController = opController;
    }

    @Override
    public void periodic() {
    }
}
