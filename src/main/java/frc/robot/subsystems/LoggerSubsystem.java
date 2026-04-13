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
        SmartDashboard.putNumber("driver-controller/driver left x", driveController.getLeftX());
        SmartDashboard.putNumber("driver-controller/driver left y", driveController.getLeftY());
        SmartDashboard.putNumber("driver-controller/driver right x", driveController.getRightX());
        SmartDashboard.putNumber("driver-controller/driver right y", driveController.getRightY());
        SmartDashboard.putNumber("operator-controller/operator left x", opController.getLeftX());
        SmartDashboard.putNumber("operator-controller/operator left y", opController.getLeftY());
        SmartDashboard.putNumber("operator-controller/operator right x", opController.getRightX());
        SmartDashboard.putNumber("operator-controller/operator right y", opController.getRightY());
        SmartDashboard.putNumber("timestamp", Timer.getFPGATimestamp());
    }
}
