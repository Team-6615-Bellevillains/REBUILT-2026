package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SnowblowCommand extends Command {

    private static final double FIELD_HALF_X = 4.035;

    private static final double Equalizer = 0.676725; // Middle between hub and wall
    private static final double POS_X  = 8.070 - Equalizer;
    private static final double NEGATIVE_X = 0 + Equalizer;

    private static final double SNOWBLOW_RPM = 3000;  // TODO: tune as needed

    private final Supplier<Pose2d> poseSupplier;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final TurretSubsystem  turret;

    public SnowblowCommand(Supplier<Pose2d> poseSupplier, ShooterSubsystem shooter,
                           IndexerSubsystem indexer, TurretSubsystem turret) {
        this.poseSupplier = poseSupplier;
        this.shooter = shooter;
        this.indexer = indexer;
        this.turret = turret;
        this.addRequirements(shooter, indexer, turret);
    }

    @Override
    public void initialize() {
        indexer.setState(IndexerSubsystem.State.OFF);
    }

    @Override
    public void execute() {
        double robotX = poseSupplier.get().getTranslation().getX();
        double diff = robotX - FIELD_HALF_X;

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d hub = Utils.getHubCenter(alliance);

        Translation2d snowblowTarget = diff < 0
            ? new Translation2d(POS_X, hub.getY())  // robot on blue side, aim to blue middles
            : new Translation2d(NEGATIVE_X, hub.getY()); // robot on red side, aim to red middles

        Pose2d robotPose = poseSupplier.get();
        Translation2d turretPosition = Utils.calculateTurretTranslation(robotPose);
        turret.aimAt(snowblowTarget);
        shooter.setPoint(shooter.getRPMFromDistance(Meters.of(turretPosition.getDistance(snowblowTarget))));

        if (shooter.atSetPoint() && turret.atTarget()) {
            indexer.setState(IndexerSubsystem.State.SHOOT);
        } else {
            indexer.setState(IndexerSubsystem.State.OFF);
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setState(IndexerSubsystem.State.OFF);
        shooter.stop();
    }
}