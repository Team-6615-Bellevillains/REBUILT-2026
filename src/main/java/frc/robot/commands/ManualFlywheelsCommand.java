package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualFlywheelsCommand extends Command{

    private static final double MAX_SPEED = 5000;

    private final ShooterSubsystem shooter;
    private final DoubleSupplier speedAxis;

    private double currentSpeed = 0;

    public ManualFlywheelsCommand(ShooterSubsystem shooter, DoubleSupplier speedAxis){
        this.shooter = shooter;
        this.speedAxis = speedAxis;
    }

    @Override
    public void initialize() {
        
    
    }

    @Override
    public void execute() {
        currentSpeed = MathUtil.clamp(currentSpeed + 10 * speedAxis.getAsDouble(), 0, MAX_SPEED);
        shooter.setPoint(RPM.of(currentSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
