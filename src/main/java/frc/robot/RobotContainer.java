// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootAtRPMCommand;
import frc.robot.commands.ShootDistanceBasedCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  // controllers for the driver and operator, respectively.
  // driver is the one who controls the drivetrain and moves the robot.
  // operator controls mechanisms on the robot.
  // certain mechanisms may be assigned to the driver if needed.
  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);

  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem(swerveSubsystem::getRobotRelativeVelocity);
  TurretSubsystem turretSubsystem = new TurretSubsystem(swerveSubsystem::getPose);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                              () -> driverController.getLeftY() * -1,
                                                              () -> driverController.getLeftX() * -1)
                                                          .withControllerRotationAxis(driverController::getRightX)
                                                          .scaleTranslation(0.8)
                                                          .allianceRelativeControl(true);

  private final SendableChooser<Command> autoChooser;
  


  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
    turretSubsystem.rehome();

    configureBindings();
  }

  private void configureBindings() {

    // swerve config
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(driveAngularVelocity, driverController.povUp()));

    // Intake Controls

    operatorController.b().onTrue(intakeSubsystem.toggleInOut());
    operatorController.leftBumper().onTrue(intakeSubsystem.setWheelsCommand(true));
    operatorController.leftBumper().onFalse(intakeSubsystem.setWheelsCommand(false));

    // Shooter and Spindexer Controls

    operatorController.rightTrigger().whileTrue(new ShootDistanceBasedCommand(swerveSubsystem::getPose, shooterSubsystem, indexerSubsystem));
    operatorController.rightTrigger().whileFalse(shooterSubsystem.stopCommand());
    //operatorController.leftBumper().whileTrue(indexerSubsystem.indexerRunCommand());
    operatorController.povUp().whileTrue(indexerSubsystem.indexerReverseCommand());
    
    // Climber Controls
    // D-Pad Up and Down: Climb up and down
    driverController.leftBumper().whileTrue(climberSubsystem.climb(0.3));
    driverController.rightBumper().whileTrue(climberSubsystem.climb(-0.3));
    driverController.a().onTrue(swerveSubsystem.resetGyroCommand());
    driverController.x().whileTrue(swerveSubsystem.lockPoseCommand());

    SmartDashboard.putNumber("Starting Angle (Degrees)", 0);
    driverController.y().onTrue(Commands.runOnce(() -> {
      double newAngleDegrees = SmartDashboard.getNumber("Starting Angle (Degrees)", 0);
      Translation2d currentTranslation = swerveSubsystem.getPose().getTranslation();
      swerveSubsystem.resetPose(new Pose2d(
        currentTranslation,
        Rotation2d.fromDegrees(newAngleDegrees)
      ));
    }));

    //operatorController.b().whileTrue(shooterSubsystem.sysId());
    //operatorController.b().onTrue(shooterSubsystem.liveRPMCommand());

    NamedCommands.registerCommand("shootfor10s", Commands.deadline(Commands.waitSeconds(10), new ShootAtRPMCommand(shooterSubsystem, indexerSubsystem, RPM.of(3000))));


    operatorController.rightBumper().whileTrue(Commands.run(() -> turretSubsystem.aimAtHub(), turretSubsystem));
    operatorController.rightBumper().onFalse(Commands.runOnce(() -> turretSubsystem.setTargetAngle(0.0), turretSubsystem));
    // TURRET SETUP
    // TurretSubsystem turretSubsystem = new TurretSubsystem(swerveSubsystem::getPose);
    // Add to constructor: turretSubsystem.rehome();

    // TURRET TESTING BINDINGS

    // HOMING TEST: Manually trigger a rehome
    // driverController.back().onTrue(Commands.runOnce(() -> turretSubsystem.rehome()));

    // DIRECTION TEST: Command specific angles to check positive/negative directions
    // operatorController.povLeft().onTrue(Commands.runOnce(() -> turretSubsystem.setTargetAngle(-45.0)));   // should go left
    // operatorController.povDown().onTrue(Commands.runOnce(() -> turretSubsystem.setTargetAngle(0.0)));     // should return to forward

    // AIM TEST: Hold button to continuously aim at hub, release to return to forward
    // operatorController.rightBumper().whileTrue(Commands.run(() -> turretSubsystem.aimAtHub(), turretSubsystem));
    // operatorController.rightBumper().onFalse(Commands.runOnce(() -> turretSubsystem.setTargetAngle(0.0), turretSubsystem));

    // SHOOT GATE TEST: Only fires if turret can actually reach the hub
    // operatorController.rightTrigger().whileTrue(
    //     Commands.run(() -> {
    //         if (turretSubsystem.canShoot() && turretSubsystem.atTarget()) {
    //             // replace with shoot command
    //         }
    //     })
    // );
  }

  public Command getAutonomousCommand() {
    return Commands.deadline(Commands.waitSeconds(10), new ShootAtRPMCommand(shooterSubsystem, indexerSubsystem, RPM.of(3000)));
    //return autoChooser.getSelected();
  }
}
