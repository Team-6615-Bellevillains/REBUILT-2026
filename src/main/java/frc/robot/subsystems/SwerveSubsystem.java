package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;


public class SwerveSubsystem extends SubsystemBase{
    
    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    private SwerveDrive drive;
 
    public SwerveSubsystem(){
        try {
            new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (Exception e) {
            //robot.explode();
            throw new RuntimeException("swerve config file missing");
        }
    }

    @Override
    public void periodic() {
        
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
}
