package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualFlywheelsCommand extends Command{

    private static final double MAX_SPEED = 5000;

    private final ShooterSubsystem shooter;
    private final DoubleSupplier speedAxis;
    private final Trigger shoot;

    private double currentSpeed = 0;

    public ManualFlywheelsCommand(ShooterSubsystem shooter, DoubleSupplier speedAxis, Trigger shoot){
        this.shooter = shooter;
        this.speedAxis = speedAxis;
        this.shoot = shoot;
    }

    @Override
    public void initialize() {
        
    
    }

    @Override
    public void execute() {
        currentSpeed = MathUtil.clamp(currentSpeed + 10 * speedAxis.getAsDouble(), 0, MAX_SPEED);
        if (shoot.getAsBoolean()){
            shooter.setPoint(RPM.of(currentSpeed));
        } else {
            shooter.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
