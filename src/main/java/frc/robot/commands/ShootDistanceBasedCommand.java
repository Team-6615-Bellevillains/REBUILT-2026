package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.InterpolatingDistanceAngularVelocityTreeMap;

public class ShootDistanceBasedCommand extends Command {

    private final Supplier<Pose2d> poseSupplier;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final InterpolatingDistanceAngularVelocityTreeMap distanceToFlywheelVelocityInterpolator;
    private AngularVelocity velocity;

    public ShootDistanceBasedCommand(Supplier<Pose2d> poseSupplier, ShooterSubsystem shooter, IndexerSubsystem indexer){
        this.addRequirements(shooter, indexer);
        this.poseSupplier = poseSupplier;
        this.shooter = shooter;
        this.indexer = indexer; 
        //TODO: add real values
        distanceToFlywheelVelocityInterpolator = new InterpolatingDistanceAngularVelocityTreeMap(){{
            this.addDatapoint(Meters.of(0.3), Rotations.per(Minute).of(3000));
            this.addDatapoint(Meters.of(0.5), Rotations.per(Minute).of(3500));
        }};
    }

    @Override
    public void initialize() {
        indexerOff();
    }

    @Override
    public void execute() {
        Translation2d hubPosition = Utils.getHubCenter(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));

        Pose2d currentPose = poseSupplier.get();
        Distance distanceToHub = Meters.of(currentPose.getTranslation().getDistance(hubPosition));
        shooter.setPoint(distanceToFlywheelVelocityInterpolator.getInterpolatedValue(distanceToHub));
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
