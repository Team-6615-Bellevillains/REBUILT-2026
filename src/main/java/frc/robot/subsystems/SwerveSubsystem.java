package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase{
    
    double maximumSpeed = Units.feetToMeters(15);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    private SwerveDrive drive;
    String limelight3g = "limelight-threeg";
    private Pigeon2 gyro = new Pigeon2(0);
    private Field2d field = new Field2d();

    public SwerveSubsystem(){
        try {
            drive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (Exception e) {
            //robot.explode();
            throw new RuntimeException("swerve config file missing");
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        LimelightHelpers.setCameraPose_RobotSpace(limelight3g, Inches.of(13.125).in(Meters), 
        -Inches.of(3.875).in(Meters), Inches.of(8.625).in(Meters), 0, 15, 0);
        drive.zeroGyro();
        gyro.setYaw(Degrees.of(0));

        RobotConfig config;
        try {
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          throw new RuntimeException("PathPlanner config file missing");
        }

        AutoBuilder.configure(this::getPose, this::resetPose, this::getRobotVelocity, (speeds, feedforwards)-> {drive.setChassisSpeeds(speeds);},
        new PPHolonomicDriveController(new PIDConstants(2.0, 0.0,0.0), new PIDConstants(2.0,0.0,0.0)), config,
        ()->{            
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        }, this);

    } 

    public void resetPose(Pose2d initialPose){
        drive.resetOdometry(initialPose);
    }

    public ChassisSpeeds getRobotVelocity(){
        return drive.getRobotVelocity();
    }

    @Override
    public void periodic() {
        Pose2d currentPose = getPose();
        SmartDashboard.putNumber("rotation fed to limelight", gyro.getYaw().getValue().in(Degrees));
        LimelightHelpers.SetRobotOrientation(limelight3g, gyro.getYaw().getValue().in(Degrees), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight3g);
        if(mt2 != null && !(Math.abs(gyro.getAngularVelocityYWorld().getValueAsDouble())>360 || mt2.tagCount == 0)){
            drive.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 9999999));
            drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
        field.setRobotPose(currentPose);
        SmartDashboard.putData("field", field);
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

    public Command resetGyroCommand(){
        return this.runOnce(()->{
            drive.zeroGyro();
            gyro.setYaw(Degrees.of(0));
        });
    }

    public Pose2d getPose(){
        return drive.getPose();
    }
}
