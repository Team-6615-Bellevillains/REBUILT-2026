// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Modified for use with YAGSL by Team 6615

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.parser.SwerveDriveConfiguration;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public class SpinForDiameterCharacterization extends Command {
  // -- Characterization Config --
  private static final AngularVelocity DEFAULT_ROTATION_SPEED = RotationsPerSecond.of(0.5);
  private static final String ROTATION_SPEED_TOPIC_NAME = "SpinForDiameterCharacterization/rotationSpeed(Rotations per Second)";

  // Take 3 seconds to spin up to 1 rotation/second
  private static final AngularAcceleration ROTATION_ACCELERATION_LIMIT = RotationsPerSecond.one().div(Seconds.of(3));


  // -- Swerve Modules Input --
  private final SwerveSubsystem swerve;
  private final Per<DistanceUnit, AngleUnit> actualDistanceTraveledPerRobotRotation;
  private SwerveModulePosition[] initialModulePositions;
  private final Distance expectedWheelDiameter;

  // -- Gyro Input --
  private Rotation2d lastGyroRotation = Rotation2d.kZero;

  // -- State --
  private final SlewRateLimiter rotationAccelerationLimiter = new SlewRateLimiter(ROTATION_ACCELERATION_LIMIT.in(RotationsPerSecondPerSecond));
  private Angle gyroRotationSinceStart = Radians.zero();
  private Distance actualWheelDiameter = Inches.zero();

  public SpinForDiameterCharacterization(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    SwerveDriveConfiguration swerveDriveConfiguration = swerve.getSwerveDrive().swerveDriveConfiguration;

    this.expectedWheelDiameter = Inches.of(swerveDriveConfiguration.physicalCharacteristics.conversionFactor.drive.diameter);

    Distance driveBaseRadius = Meters.of(swerveDriveConfiguration.moduleLocationsMeters[0].getDistance(Translation2d.kZero));
    this.actualDistanceTraveledPerRobotRotation = driveBaseRadius.div(Radians.one());
  }

  @Override
  public void initialize() {
    // Reset inputs and state
    lastGyroRotation = swerve.getPose().getRotation();
    gyroRotationSinceStart = Radians.zero();

    initialModulePositions = swerve.getSwerveDrive().getModulePositions();

    rotationAccelerationLimiter.reset(0);

    SmartDashboard.putNumber(ROTATION_SPEED_TOPIC_NAME, getTargetRotationSpeed().in(RotationsPerSecond));
  }

  private Distance calculateActualDistanceTraveled() {
    // Use the gyro as ground truth for how much distance we've actually traveled
    gyroRotationSinceStart = gyroRotationSinceStart.plus(swerve.getPose().getRotation().minus(lastGyroRotation).getMeasure());

    Distance actualDistanceTraveled = Meters.of(Math.abs(actualDistanceTraveledPerRobotRotation.times(gyroRotationSinceStart).baseUnitMagnitude()));
    SmartDashboard.putNumber("SpinForDiameterCharacterization/actualDistanceTraveled", actualDistanceTraveled.in(Inches));

    lastGyroRotation = swerve.getPose().getRotation();

    return actualDistanceTraveled;
  }

  private Distance calculateExpectedDistanceTraveled() {
    // We expect to have traveled whatever distance the swerve wheels are currently reportingb
    Distance averageWheelDistance = Inches.zero();

    SwerveModulePosition[] modulePositions = swerve.getSwerveDrive().getModulePositions();
    for (int i = 0; i < 4; i++) {
      averageWheelDistance = averageWheelDistance.plus(Meters.of(
        Math.abs(modulePositions[i].distanceMeters - initialModulePositions[i].distanceMeters)
      ));
    }

    averageWheelDistance = averageWheelDistance.div(Value.of(4));
    SmartDashboard.putNumber("SpinForDiameterCharacterization/averageWheelDistance", averageWheelDistance.in(Inches));

    return averageWheelDistance;
  }

  private AngularVelocity getTargetRotationSpeed() {
    return RotationsPerSecond.of(
      SmartDashboard.getNumber(ROTATION_SPEED_TOPIC_NAME, DEFAULT_ROTATION_SPEED.in(RotationsPerSecond))
    );
  }

  @Override
  public void execute() {
    // Rotate the robot in-place (with acceleration limiter)
    swerve.getSwerveDrive().setChassisSpeeds(
      new ChassisSpeeds(
        0,
        0,
        RotationsPerSecond.of(
          rotationAccelerationLimiter.calculate(getTargetRotationSpeed().in(RotationsPerSecond))
        ).in(RadiansPerSecond)
      )
    );

    Distance actualDistanceTraveled = calculateActualDistanceTraveled();
    Distance expectedDistanceTraveled = calculateExpectedDistanceTraveled();

    // Prevent divide-by-zero
    if (expectedDistanceTraveled.in(Inches) > 1e-6) {
      // (actual distance)/(expected distance) * (expected diameter) = actual diameter
      actualWheelDiameter = actualDistanceTraveled.div(expectedDistanceTraveled).times(expectedWheelDiameter);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.getSwerveDrive().setChassisSpeeds(new ChassisSpeeds());
    
    double result;

    if (gyroRotationSinceStart.abs(Rotations) <= 1) {
      // Not enough data to be reliable
      result = Double.NaN;
    } else {
      result = actualWheelDiameter.in(Inches);
    }

    SmartDashboard.putNumber("SpinForDiameterCharacterization/actualWheelDiameter (Inches)", result); 
  }
}