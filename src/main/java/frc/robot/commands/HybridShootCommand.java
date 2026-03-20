package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class HybridShootCommand extends Command {

    private final SwerveSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final IndexerSubsystem indexer;
    private final Supplier<Translation2d> targetSupplier;

    private final ShootOnTheMoveCommandRevisedAdjusted sotmCommand;

    private boolean shouldShoot;

    public HybridShootCommand(
            SwerveSubsystem drivetrain,
            TurretSubsystem turret,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            Supplier<Translation2d> targetSupplier) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.shooter = shooter;
        this.indexer = indexer;
        this.targetSupplier = targetSupplier;

        this.sotmCommand = new ShootOnTheMoveCommandRevisedAdjusted(
                drivetrain, turret, shooter, indexer, targetSupplier);

        addRequirements(turret, shooter, indexer);
    }

    @Override
    public void initialize() {
        sotmCommand.initialize();
        shouldShoot = false;
    }

    @Override
    public void execute() {
        if (!shouldShoot){
            shouldShoot = shooter.atSetPoint();
        }
        Pose2d pose = drivetrain.getPose();
        boolean inAllianceZone = Utils.isInAllianceZone(pose);

        SmartDashboard.putBoolean("HybridShoot/inAllianceZone", inAllianceZone);
        SmartDashboard.putString("HybridShoot/mode", inAllianceZone ? "STATIONARY_HUB" : "SOTM_SNOWBLOW");

        if (inAllianceZone) {
            runStationaryHubShot(pose);
        } else {
            sotmCommand.execute();
        }
    }

    private void runStationaryHubShot(Pose2d pose) {
        Translation2d hubPosition = Utils.getHubCenter(
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));

        Translation2d turretPosition = Utils.calculateTurretTranslation(pose);
        Distance distanceToHub = Meters.of(turretPosition.getDistance(hubPosition));

        shooter.setPoint(shooter.getRPMFromDistance(distanceToHub));
        turret.aimAtHub();

        SmartDashboard.putNumber("HybridShoot/distanceToHub", distanceToHub.in(Meters));

        if (shouldShoot && turret.canShoot() && turret.atTarget()) {
            indexer.setState(IndexerSubsystem.State.SHOOT);
        } else {
            indexer.setState(IndexerSubsystem.State.OFF);
        }
    }

    @Override
    public void end(boolean interrupted) {
        sotmCommand.end(interrupted);
        indexer.setState(IndexerSubsystem.State.OFF);
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}