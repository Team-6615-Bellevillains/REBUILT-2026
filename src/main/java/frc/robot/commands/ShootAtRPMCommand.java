package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAtRPMCommand extends Command{
    
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final AngularVelocity velocity;
    private final BooleanSupplier atSetPoint;

    public ShootAtRPMCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, AngularVelocity velocity){
        this.shooter = shooter;
        this.indexer = indexer;
        this.velocity = velocity;
        this.atSetPoint = shooter.nearVelocity(velocity);
        this.addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        indexerOff();
        shooter.setPoint(velocity);
    }

    @Override
    public void execute() {
        if (!atSetPoint.getAsBoolean()){
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
