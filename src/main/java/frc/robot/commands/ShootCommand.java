package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final double shooterRPM;

    public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, double shooterRPM){
        this.shooter = shooter;
        this.indexer = indexer;
        this.shooterRPM = shooterRPM;
    }

    @Override
    public void initialize() {
        indexerOff();
        shooter.RpmSetPoint(shooterRPM);
    }

    @Override
    public void execute() {
        if (!shooter.atSetpoint()){
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
        indexer.setState(IndexerSubsystem.State.ON);
    }
}
