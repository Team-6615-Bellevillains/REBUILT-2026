package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Utils {
    public static Translation2d getHubCenter(DriverStation.Alliance alliance) {
        switch (alliance) {
            case Red:
                return new Translation2d(11.90, 4.02);
            case Blue:
                return new Translation2d(4.61, 4.02);
            default:
                // Impossible case
                return Translation2d.kZero;
        }
    }
}
