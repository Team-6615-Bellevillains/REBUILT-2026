package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LoggerSubsystem extends SubsystemBase {
    private final CommandXboxController driveController;
    private final CommandXboxController opController;
    private static Field2d field = new Field2d();
    
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
        SmartDashboard.putData("Poses", field);
        SmartDashboard.putNumber("timestamp", Timer.getFPGATimestamp());
    }

    public static void addFieldPose(Pose2d pose, String name){
        field.getObject(name).setPose(pose);
    }

    public static void setRobotPose(Pose2d robotPose){
        field.setRobotPose(robotPose);
    }
}
