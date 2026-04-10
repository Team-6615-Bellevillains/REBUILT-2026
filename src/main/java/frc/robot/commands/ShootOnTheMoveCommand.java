package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// taken from 2181's code, adapted for our code

public class ShootOnTheMoveCommand extends Command {
  private final SwerveSubsystem drivetrain;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;
  private final IndexerSubsystem indexer;
  private final Supplier<Translation2d> targetSupplier;


  private Rotation2d lastTurretAngle;
  private Rotation2d turretAngle;

  private static double phaseDelay;
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private final Timer timer = new Timer();

  static {
    phaseDelay = 0;

    timeOfFlightMap.put(2.4384, 0.98);
    timeOfFlightMap.put(3.048, 1.16);
    timeOfFlightMap.put(3.6576, 1.265);
    timeOfFlightMap.put(4.2672, 1.4);
    timeOfFlightMap.put(4.8768, 1.42);
    timeOfFlightMap.put(5.4864, 1.47);
  }

  public ShootOnTheMoveCommand(
      SwerveSubsystem drivetrain, TurretSubsystem turret, ShooterSubsystem shooter, IndexerSubsystem indexer, Supplier<Translation2d> targetSupplier) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.shooter = shooter;
    this.indexer = indexer;
    this.targetSupplier = targetSupplier;
    this.addRequirements(turret, shooter, indexer);
  }

  @Override
  public void initialize() {
    super.initialize();
    lastTurretAngle = Rotation2d.fromRotations(turret.getCurrentAngleDegrees()/360);
    timer.reset();
    timer.start();
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
    double turretVelocityX = robotVelocity.vxMetersPerSecond + 
      robotVelocity.omegaRadiansPerSecond * (Constants.TURRET_OFFSET.getY() * Math.cos(robotAngle) - Constants.TURRET_OFFSET.getX() * Math.sin(robotAngle));
    double turretVelocityY = robotVelocity.vyMetersPerSecond +
      robotVelocity.omegaRadiansPerSecond * (Constants.TURRET_OFFSET.getX() * Math.cos(robotAngle) - Constants.TURRET_OFFSET.getY() * Math.sin(robotAngle));

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

    // SmartDashboard.putNumber("distance to turret", lookaheadTurretToTargetDistance);
    shooter.setPoint(shooter.getRPMFromDistance(Meters.of(lookaheadTurretToTargetDistance)));
    turret.aimAtFromTurretPosition(target, lookaheadPose);
    

    SmartDashboard.putNumber("Distance to Target", lookaheadTurretToTargetDistance);

    if (turret.canShoot() & timer.get() > 0.5) {
      indexer.setState(IndexerSubsystem.State.SHOOT);
    } else {
      indexer.setState(IndexerSubsystem.State.OFF);
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexer.setState(IndexerSubsystem.State.OFF);
  }

  public static Translation2d calculateLookaheadTarget( // For autoaiming
        SwerveSubsystem drivetrain, Translation2d target) {
    Pose2d estimatedPose = drivetrain.getPose();
    ChassisSpeeds robotRelativeVelocity = drivetrain.getRobotVelocity();
    estimatedPose = estimatedPose.exp(new Twist2d(
        robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
        robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
        robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Pose2d turretPosition = new Pose2d(
        Utils.calculateTurretTranslation(estimatedPose), estimatedPose.getRotation());

    ChassisSpeeds robotVelocity = drivetrain.getFieldRelativeVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX = robotVelocity.vxMetersPerSecond + robotVelocity.omegaRadiansPerSecond
        * (Constants.TURRET_OFFSET.getY() * Math.cos(robotAngle)
        -  Constants.TURRET_OFFSET.getX() * Math.sin(robotAngle));
    double turretVelocityY = robotVelocity.vyMetersPerSecond + robotVelocity.omegaRadiansPerSecond
        * (Constants.TURRET_OFFSET.getX() * Math.cos(robotAngle)
        -  Constants.TURRET_OFFSET.getY() * Math.sin(robotAngle));

    Pose2d lookaheadPose = turretPosition;
    for (int i = 0; i < 20; i++) {
        double dist = target.getDistance(lookaheadPose.getTranslation());
        double tof = timeOfFlightMap.get(dist);
        lookaheadPose = new Pose2d(
            turretPosition.getTranslation().plus(
                new Translation2d(turretVelocityX * tof, turretVelocityY * tof)),
            turretPosition.getRotation());
    }

    Translation2d lookaheadOffset = lookaheadPose.getTranslation().minus(turretPosition.getTranslation());
      return target.minus(lookaheadOffset);
  }
}
