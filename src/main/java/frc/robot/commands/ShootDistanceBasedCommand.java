package frc.robot.commands;

import static edu.wpi.first.units.Units.Minutes;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootDistanceBasedCommand extends Command {

    private final Supplier<Pose2d> poseSupplier;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final InterpolatingDoubleTreeMap distancesToRPM;
    private AngularVelocity velocity;

    public ShootDistanceBasedCommand(Supplier<Pose2d> poseSupplier, ShooterSubsystem shooter, IndexerSubsystem indexer){
        this.addRequirements(shooter, indexer);
        this.poseSupplier = poseSupplier;
        this.shooter = shooter;
        this.indexer = indexer; 
        //TODO: add real values
        distancesToRPM = new InterpolatingDoubleTreeMap(){{
            this.put(0.3, 3000d);
            this.put(0.5, 3500d);
        }};
    }

    @Override
    public void initialize() {
        indexerOff();
    }

    //TODO: add alliance switching
    private static final Translation2d hubPosition = new Translation2d(4.626, 4.037);
    @Override
    public void execute() {
        Pose2d currentPose = poseSupplier.get();
        double distanceToHub = currentPose.getTranslation().getDistance(hubPosition);
        shooter.setPoint(Rotations.per(Minutes).of(distancesToRPM.get(distanceToHub)));
        if (!shooter.atSetPoint()){
            indexerOff();
            return;
        }
        indexerOn();
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
