package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Utils;
import frc.robot.utils.AccelerationLimiter;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase{
    
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    private SwerveDrive drive;
    String limelight3g = "limelight-threeg";
    String limelight4 = "limelight-four";
    private Pigeon2 gyro = new Pigeon2(0);
    private Field2d field = new Field2d();
    private Field2d limelight3gField = new Field2d();
    private Field2d limelight4Field = new Field2d();
    private AccelerationLimiter accelLimiter = new AccelerationLimiter(FeetPerSecondPerSecond.of(15), DegreesPerSecondPerSecond.of(360));

    public SwerveSubsystem(){
        try {
            drive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.MAX_SPEED.in(MetersPerSecond));
        } catch (Exception e) {
            //robot.explode();
            throw new RuntimeException("swerve config file missing");
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.INFO;
        
        //LL4 setup
        LimelightHelpers.setCameraPose_RobotSpace(
            limelight4, 
            -Inches.of(13.125).in(Meters), 
            Inches.of(3.875).in(Meters), 
            Inches.of(8.625).in(Meters), 
            0, 
            15, 
            180
        );

        //LL3G setup
        LimelightHelpers.setCameraPose_RobotSpace(
            limelight3g, 
            -Inches.of(2.9).in(Meters), 
            Inches.of(12.977).in(Meters), 
            Inches.of(8.65).in(Meters), 
            0, 
            20, 
            270
        );

        drive.zeroGyro();
        gyro.setYaw(Degrees.of(0));

        LimelightHelpers.SetIMUMode(limelight4, 0);

    } 

    public void initPathPlanner() {
        RobotConfig config;
        try {
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          //pathplanner.explode();
          throw new RuntimeException("PathPlanner config file missing: " + e.getMessage());
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
        SmartDashboard.putNumber("localization/rotation fed to limelight", currentPose.getRotation().getDegrees());
        LimelightHelpers.SetRobotOrientation(limelight4, currentPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(limelight3g, currentPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate limelight4Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight4);
        LimelightHelpers.PoseEstimate limelight3gPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight3g);


        field.getObject("limelight 4 pose").setPose(limelight4Pose.pose);
        field.getObject("limelight 3g pose").setPose(limelight3gPose.pose);

        if(limelight4Pose != null && !(Math.abs(gyro.getAngularVelocityYWorld().getValueAsDouble())>360 || limelight4Pose.tagCount == 0)){
            drive.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 9999999));
            drive.addVisionMeasurement(limelight4Pose.pose, limelight4Pose.timestampSeconds);
        }
        if(limelight3gPose != null && !(Math.abs(gyro.getAngularVelocityYWorld().getValueAsDouble())>360 || limelight3gPose.tagCount == 0)){
            drive.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 9999999));
            drive.addVisionMeasurement(limelight3gPose.pose, limelight3gPose.timestampSeconds);
        }
        field.setRobotPose(currentPose);
        SmartDashboard.putData("localization/field", field);
        Translation2d hubPosition = Utils.getHubCenter(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
        double hubDistance = currentPose.getTranslation().getDistance(hubPosition);
        SmartDashboard.putNumber("localization/feet to hub", Units.metersToFeet(hubDistance));
        SmartDashboard.putNumber("swerve/measured speed, in meters/s", Math.hypot(drive.getFieldVelocity().vxMetersPerSecond, drive.getFieldVelocity().vyMetersPerSecond));
    }

    public SwerveDrive getSwerveDrive(){
        return drive;
    }

    public Command driveCommand(SwerveInputStream swerveInput, BooleanSupplier turbo, BooleanSupplier accelLimit){
        return this.run(() -> {

            SwerveInputStream adjustedSwerve;
            if (turbo.getAsBoolean() && !accelLimit.getAsBoolean()) 
                adjustedSwerve = swerveInput.scaleTranslation(1.5/Constants.MAX_SPEED.in(MetersPerSecond));
            else if (accelLimit.getAsBoolean()) 
                adjustedSwerve = swerveInput.scaleTranslation(1.5/Constants.MAX_SPEED.in(MetersPerSecond));
            else 
                adjustedSwerve = swerveInput.scaleTranslation(0.9);

            ChassisSpeeds velocity = adjustedSwerve.get();
            if (accelLimit.getAsBoolean()) 
                velocity = accelLimiter.calculate(velocity);
            else 
                accelLimiter.update(velocity);

            drive.driveFieldOriented(velocity);
        });
    }

    public Command lockPoseCommand(){
        return this.run(()->{
            drive.lockPose();
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

    public ChassisSpeeds getFieldRelativeVelocity(){
        return drive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotRelativeVelocity(){
        return drive.getRobotVelocity();
    }

}
