package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class ManualAimCommand extends Command {

    private static final double MIN_ANGLE = 130;
    private static final double MAX_ANGLE = 230;

    private final TurretSubsystem turret;
    private final DoubleSupplier aimAxis;

    private double currentAngle = 0;

    public ManualAimCommand(TurretSubsystem turret, DoubleSupplier aimAxis){
        this.turret = turret;
        this.aimAxis = aimAxis;
    }

    @Override
    public void initialize() {
        currentAngle = 0;
    }

    @Override
    public void execute() {
        currentAngle = MathUtil.clamp(currentAngle + aimAxis.getAsDouble(), MIN_ANGLE, MAX_ANGLE);
        SmartDashboard.putNumber("manual-aim/current-angle", currentAngle);
        turret.setTargetAngle(currentAngle);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
