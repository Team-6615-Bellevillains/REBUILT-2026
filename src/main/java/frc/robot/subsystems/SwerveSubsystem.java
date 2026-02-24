package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase{
    
    double maximumSpeed = Units.feetToMeters(10);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    private SwerveDrive drive;
    String limelight3g = "limelight-threeg";
    private Pigeon2 gyro = new Pigeon2(0);
 
    public SwerveSubsystem(){
        try {
            drive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (Exception e) {
            //robot.explode();
            throw new RuntimeException("swerve config file missing");
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    @Override
    public void periodic() {
        Pose2d currentPose = getPose();
        SmartDashboard.putNumber("rotation fed to limelight", currentPose.getRotation().getDegrees());
        LimelightHelpers.SetRobotOrientation(limelight3g, currentPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight3g);
        if(mt2 != null && !(Math.abs(gyro.getAngularVelocityYWorld().getValueAsDouble())>360 || mt2.tagCount == 0)){
            drive.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 9999999));
            drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative){
        drive.drive(translation, rotation, fieldRelative, false);
    }

    public SwerveDrive getSwerveDrive(){
        return drive;
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
        return run(() -> {
            drive.driveFieldOriented(velocity.get());
        });
    }

    public Pose2d getPose(){
        return drive.getPose();
    }
}
