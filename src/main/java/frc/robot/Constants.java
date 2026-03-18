package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;

public final class Constants {

    public static final LinearVelocity MAX_SPEED = FeetPerSecond.of(15);
    public static final Translation2d TURRET_OFFSET = new Translation2d(-0.1714, -0.1714);

    // Knowing where you are on the field
    public static final double BLUE_ALLIANCE_ZONE_MAX_X = 4.03;
    public static final double RED_ALLIANCE_ZONE_MIN_X  = 12.51;

    // Snowblow targets
    public static final double FIELD_HALF_Y     = 4.035;
    public static final double SNOWBLOW_POS_Y   = 8.070 - (4.035 / 2.0); // ~6.053
    public static final double SNOWBLOW_NEG_Y   = 4.035 / 2.0;            // ~2.018
}
