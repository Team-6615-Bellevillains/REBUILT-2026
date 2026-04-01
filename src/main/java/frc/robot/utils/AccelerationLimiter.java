package frc.robot.utils;

import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;

public class AccelerationLimiter {

    private LinearAcceleration linearAccelerationLimit;

    private SlewRateLimiter omegaLimiter;

    private ChassisSpeeds lastChassisSpeeds;
    private double lastTimeSeconds;

    public AccelerationLimiter(LinearAcceleration linearAccelerationLimit, AngularAcceleration angularAccelerationLimit){
        this.linearAccelerationLimit = linearAccelerationLimit;
        omegaLimiter = new SlewRateLimiter(angularAccelerationLimit.in(RadiansPerSecondPerSecond));
    }

    public ChassisSpeeds calculate(ChassisSpeeds input){
        double currentTime = MathSharedStore.getTimestamp();
        double linearAcceleration = linearAccelerationLimit.in(Units.MetersPerSecondPerSecond);
        Translation2d lastTranslation = new Translation2d(lastChassisSpeeds.vxMetersPerSecond, lastChassisSpeeds.vyMetersPerSecond);
        Translation2d inputTranslation = new Translation2d(input.vxMetersPerSecond, input.vyMetersPerSecond);
        Translation2d limitedAccel = MathUtil.slewRateLimit(lastTranslation, inputTranslation, currentTime-lastTimeSeconds, linearAcceleration);
        double limitedAngularAccel = omegaLimiter.calculate(input.omegaRadiansPerSecond);
        ChassisSpeeds calculatedChassisSpeeds =  new ChassisSpeeds(limitedAccel.getX(), limitedAccel.getY(), limitedAngularAccel);
        lastTimeSeconds = currentTime;
        lastChassisSpeeds = calculatedChassisSpeeds;
        return calculatedChassisSpeeds;
    }

    public void update(ChassisSpeeds input){
        lastChassisSpeeds = input;
        lastTimeSeconds = MathSharedStore.getTimestamp();
    }


}
