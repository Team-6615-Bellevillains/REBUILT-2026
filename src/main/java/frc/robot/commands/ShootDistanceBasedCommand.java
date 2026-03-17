package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootDistanceBasedCommand extends Command {

    private final Supplier<Pose2d> poseSupplier;
    private final BooleanSupplier turretAimed;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;

    public ShootDistanceBasedCommand(Supplier<Pose2d> poseSupplier, ShooterSubsystem shooter, IndexerSubsystem indexer, BooleanSupplier turretAimed){
        this.addRequirements(shooter, indexer);
        this.turretAimed = turretAimed;
        this.poseSupplier = poseSupplier;
        this.shooter = shooter;
        this.indexer = indexer; 
    }

    @Override
    public void initialize() {
        indexerOff();
    }

    @Override
    public void execute() {
        Translation2d hubPosition = Utils.getHubCenter(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));

        Pose2d robotPose = poseSupplier.get();
        Translation2d turretPosition = Utils.calculateTurretTranslation(robotPose);
        Distance distanceToHub = Meters.of(turretPosition.getDistance(hubPosition));
        shooter.setPoint(shooter.getRPMFromDistance(distanceToHub));
        if (shooter.atSetPoint() && turretAimed.getAsBoolean()) indexerOn();
        //else indexerOff();
    }

    @Override
    public void end(boolean interrupted) {
        indexerOff();
        shooter.stop();
    }

    private void indexerOff(){
        indexer.setState(IndexerSubsystem.State.OFF);
    }
    
    private void indexerOn(){
        indexer.setState(IndexerSubsystem.State.SHOOT);
    }
}
