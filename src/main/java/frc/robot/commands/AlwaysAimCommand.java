package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AlwaysAimCommand extends Command {

    private final TurretSubsystem turret;
    private final SwerveSubsystem swerve;

    public AlwaysAimCommand(SwerveSubsystem swerve, TurretSubsystem turret) {
        this.turret = turret;
        this.swerve = swerve;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        Translation2d rawTarget = Utils.calculateShotTarget(swerve.getPose());
        Translation2d lookaheadTarget = ShootOnTheMoveCommand.calculateLookaheadTarget(swerve, rawTarget);
        turret.aimAt(lookaheadTarget);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}