package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    public static Translation2d calculateTurretTranslation(Pose2d pose){
        return pose.getTranslation().plus(TURRET_OFFSET.rotateBy(pose.getRotation()));
    }

    public static Translation2d calculateShotTarget(Pose2d pose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double x = pose.getTranslation().getX();
        boolean inOwnZone = (alliance == Alliance.Blue && x < Constants.BLUE_ALLIANCE_ZONE_MAX_X)
                        || (alliance == Alliance.Red  && x > Constants.RED_ALLIANCE_ZONE_MIN_X);
        if (inOwnZone) return Utils.getHubCenter(alliance);
        double robotY = pose.getTranslation().getY();
        double hubX = Utils.getHubCenter(alliance).getX();
        if (alliance == Alliance.Blue) {
            hubX -= 3; // Offset for snowblowing
        } else if (alliance == Alliance.Red) {
            hubX += 3; // Offset for snowblowing
        }
        return (robotY < Constants.FIELD_HALF_Y)
            ? new Translation2d(hubX, Constants.SNOWBLOW_NEG_Y)
            : new Translation2d(hubX, Constants.SNOWBLOW_POS_Y);
    }

    public static boolean isInAllianceZone(Pose2d pose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double x = pose.getTranslation().getX();
        return (alliance == Alliance.Blue && x < BLUE_ALLIANCE_ZONE_MAX_X)
            || (alliance == Alliance.Red  && x > RED_ALLIANCE_ZONE_MIN_X);
    }
}
