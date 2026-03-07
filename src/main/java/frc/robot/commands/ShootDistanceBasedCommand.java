package frc.robot.commands;

import static edu.wpi.first.units.Units.*;


import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.MeasureInterpolatingTreeMap;

public class ShootDistanceBasedCommand extends Command {

    private final Supplier<Pose2d> poseSupplier;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final MeasureInterpolatingTreeMap<DistanceUnit, AngularVelocityUnit> distanceToFlywheelVelocityInterpolator;

    public ShootDistanceBasedCommand(Supplier<Pose2d> poseSupplier, ShooterSubsystem shooter, IndexerSubsystem indexer){
        this.addRequirements(shooter, indexer);
        this.poseSupplier = poseSupplier;
        this.shooter = shooter;
        this.indexer = indexer; 
        //TODO: add real values
        distanceToFlywheelVelocityInterpolator = new MeasureInterpolatingTreeMap<DistanceUnit, AngularVelocityUnit>(){{
            this.addPoint(Feet.of(8.2), Rotations.per(Minute).of(2700));
            // this.addDatapoint(Feet.of(9.6), Rotations.per(Minute).of(2800));
            // this.addDatapoint(Feet.of(11), Rotations.per(Minute).of(3000));
            // this.addDatapoint(Feet.of(13), Rotations.per(Minute).of(3500));
            // this.addDatapoint(Feet.of(14.9), Rotations.per(Minute).of(3600));
            this.addPoint(Feet.of(17.7), Rotations.per(Minute).of(4000));
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
        shooter.setPoint(RPM.of(distanceToFlywheelVelocityInterpolator.getValue(distanceToHub).in(RPM)));
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
