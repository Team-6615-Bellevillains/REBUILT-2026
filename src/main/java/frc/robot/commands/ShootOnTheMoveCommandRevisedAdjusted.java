package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// taken from 2181's code, adapted for our code

public class ShootOnTheMoveCommandRevisedAdjusted extends Command {
  private final SwerveSubsystem drivetrain;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final IndexerSubsystem indexer;
  private final Supplier<Translation2d> targetSupplier;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / /*loop secs*/0.01));

  private Rotation2d lastTurretAngle;
  private Rotation2d turretAngle;
  private double turretVelocity;
  private AngularVelocity lastShootSpeed;

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03; // should be .13?

    //TODO: populate with real values
    launchFlywheelSpeedMap.put(2.4384, 2725d);
    launchFlywheelSpeedMap.put(3.048, 3000d);
    launchFlywheelSpeedMap.put(3.6576, 3225d);
    launchFlywheelSpeedMap.put(4.2672, 3450d);
    launchFlywheelSpeedMap.put(4.8768, 3675d);
    launchFlywheelSpeedMap.put(5.4864, 4250d);

    timeOfFlightMap.put(2.4384, 0.98);
    timeOfFlightMap.put(3.048, 1.16);
    timeOfFlightMap.put(3.6576, 1.265);
    timeOfFlightMap.put(4.2672, 1.4);
    timeOfFlightMap.put(4.8768, 1.42);
    timeOfFlightMap.put(5.4864, 1.47);

  }

  public ShootOnTheMoveCommandRevisedAdjusted(
      SwerveSubsystem drivetrain, TurretSubsystem turret, ShooterSubsystem shooter, IndexerSubsystem indexer, Supplier<Translation2d> targetSupplier) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.shooter = shooter;
    this.indexer = indexer;
    this.targetSupplier = targetSupplier;
  }

  @Override
  public void initialize() {
    super.initialize();
    lastTurretAngle = Rotation2d.fromRotations(turret.getCurrentAngleDegrees()/360);
    lastShootSpeed = shooter.getVelocity();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = drivetrain.getPose();
    ChassisSpeeds robotRelativeVelocity = drivetrain.getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from turret to target
    Pose2d turretPosition = new Pose2d(Utils.calculateTurretTranslation(estimatedPose), estimatedPose.getRotation());

    // Designate desired target

    Translation2d target = targetSupplier.get();

    double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = drivetrain.getFieldRelativeVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX = robotVelocity.vxMetersPerSecond + robotVelocity.omegaRadiansPerSecond
        * (Constants.TURRET_OFFSET.getY() * Math.cos(robotAngle) - Constants.TURRET_OFFSET.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (Constants.TURRET_OFFSET.getX() * Math.cos(robotAngle)
                    - Constants.TURRET_OFFSET.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle().minus(estimatedPose.getRotation());
    if (lastTurretAngle == null) lastTurretAngle = turretAngle;
    turretVelocity =
        turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRadians() / 0.01);
    lastTurretAngle = turretAngle;
    latestParameters =
        new LaunchingParameters(
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngle,
            turretVelocity,
            launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    lastShootSpeed = RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));
    // SmartDashboard.putNumber("distance to turret", lookaheadTurretToTargetDistance);
    shooter.setPoint(RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance)));
    turret.aimAtFromTurretPosition(target, lookaheadPose);
    

    SmartDashboard.putNumber("Distance to Target", lookaheadTurretToTargetDistance);
    indexer.setState(IndexerSubsystem.State.SHOOT);
  }

  @Override
  public void end(boolean interupted) {
    indexer.setState(IndexerSubsystem.State.OFF);
  }
}
