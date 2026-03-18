package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;

public final class Constants {

    public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(15);
    public static final Translation2d TURRET_OFFSET = new Translation2d(-0.1714, -0.1714);
    public static final double AllianceZoneX = 180;
    public static final double AllianceZoneY = 316.64;
}
