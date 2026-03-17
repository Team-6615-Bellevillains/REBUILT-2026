package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootOnTheMoveCommand extends Command{

    ShooterSubsystem shooter;
    TurretSubsystem turret;
    Supplier<Pose2d> getPose;
    Supplier<ChassisSpeeds> getVelocity;

    private static final double phaseDelay = 0.0;

    public ShootOnTheMoveCommand(Supplier<Pose2d> getPose, Supplier<ChassisSpeeds> getVelocity, ShooterSubsystem shooter, TurretSubsystem turret){
        this.addRequirements(shooter, turret);
        this.turret = turret;
        this.shooter = shooter;
        this.getPose = getPose;
        this.getVelocity = getVelocity;
    }

    @Override
    public void execute() {
        Pose2d estimatedPose = getPose.get();
        ChassisSpeeds robotRelativeVelocity = getVelocity.get();
        estimatedPose.exp(new Twist2d(
            robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
            robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
            robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));
        
    }
}
