package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SnowblowCommand extends Command {

    private static final double FIELD_HALF_Y = 4.035;

    private static final double Equalizer = 4.035/2; // Middle between hub and wall
    private static final double POS_Y  = 8.070 - Equalizer;
    private static final double NEGATIVE_Y = 0 + Equalizer;

    private final Supplier<Pose2d> poseSupplier;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final TurretSubsystem  turret;

    private final Timer m_timer = new Timer();

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
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double robotY = poseSupplier.get().getTranslation().getY();
        double diff = robotY - FIELD_HALF_Y;

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d hub = Utils.getHubCenter(alliance);

        Translation2d snowblowTarget = diff < 0
            ? new Translation2d(hub.getX(), NEGATIVE_Y)  // blue side robot → aim toward blue side
            : new Translation2d(hub.getX(), POS_Y);    // red side robot  → aim toward red side

        Pose2d robotPose = poseSupplier.get();
        Translation2d turretPosition = Utils.calculateTurretTranslation(robotPose);
        turret.aimAtSnowblowing(snowblowTarget);
        shooter.setPoint(shooter.getRPMFromDistance(Meters.of(turretPosition.getDistance(snowblowTarget))));

        if (turret.atSnowblowingTarget() & m_timer.get() > 0.5) {
            indexer.setState(IndexerSubsystem.State.SHOOT);
        } 
        //else {
        //    indexer.setState(IndexerSubsystem.State.OFF);
        //}
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setState(IndexerSubsystem.State.OFF);
        shooter.stop();
    }
}