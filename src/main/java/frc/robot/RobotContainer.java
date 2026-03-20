
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HybridShootCommand;
import frc.robot.commands.ShootAtRPMCommand;
import frc.robot.commands.ShootDistanceBasedCommand;
import frc.robot.commands.ShootOnTheMoveCommandRevisedAdjusted;
import frc.robot.commands.ShootOnTheMoveDebugCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeSubsystem.State;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LoggerSubsystem;

public class RobotContainer {

  // controllers for the driver and operator, respectively.
  // driver is the one who controls the drivetrain and moves the robot.
  // operator controls mechanisms on the robot.
  // certain mechanisms may be assigned to the driver if needed.
  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);

  SwerveSubsystem  swerveSubsystem  = new SwerveSubsystem();
  ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  IntakeSubsystem  intakeSubsystem  = new IntakeSubsystem(swerveSubsystem::getRobotRelativeVelocity);
  TurretSubsystem  turretSubsystem  = new TurretSubsystem(swerveSubsystem::getPose);
  LedSubsystem     ledSubsystem     = new LedSubsystem();
  LoggerSubsystem  loggerSubsystem  = new LoggerSubsystem(driverController, operatorController);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                              () -> driverController.getLeftY() * -1,
                                                              () -> driverController.getLeftX() * -1)
                                                          .withControllerRotationAxis(()-> -1 * driverController.getRightX())
                                                          .scaleTranslation(0.9)
                                                          .allianceRelativeControl(true);

  private final SendableChooser<Command> autoChooser;
  


  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    turretSubsystem.rehome();

    registerNamedCommands();
    swerveSubsystem.initPathPlanner();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // Swerve
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(driveAngularVelocity, driverController.povUp()));

    // Driver Controls
    driverController.a().onTrue(swerveSubsystem.resetGyroCommand());
    driverController.x().whileTrue(swerveSubsystem.lockPoseCommand());
    driverController.leftBumper().whileTrue(climberSubsystem.climb(1));
    driverController.rightBumper().whileTrue(climberSubsystem.climb(-1));
    // driverController.y().onTrue(Commands.runOnce(() -> {
    //     double newAngleDegrees = SmartDashboard.getNumber("Starting Angle (Degrees)", 0);
    //     Translation2d currentTranslation = swerveSubsystem.getPose().getTranslation();
    //     swerveSubsystem.resetPose(new Pose2d(currentTranslation, Rotation2d.fromDegrees(newAngleDegrees)));
    // }));
    SmartDashboard.putNumber("Starting Angle (Degrees)", 0);

    // Operator - Intake
    operatorController.b().onTrue(intakeSubsystem.toggleInOut());
    operatorController.povDown().onTrue(intakeSubsystem.setStateCommand(IntakeSubsystem.State.PULL_IN));
    operatorController.leftBumper().onTrue(intakeSubsystem.setWheelsCommand(true));
    operatorController.leftBumper().onFalse(intakeSubsystem.setWheelsCommand(false));

    // Operator - Shooter
    operatorController.rightBumper().whileTrue(
      new HybridShootCommand(
          swerveSubsystem, turretSubsystem, shooterSubsystem, indexerSubsystem,
          () -> Utils.calculateShotTarget(swerveSubsystem.getPose())
      )
    );
    operatorController.rightBumper().onFalse(shooterSubsystem.stopCommand());

    turretSubsystem.setDefaultCommand(Commands.run(() -> { // AUTOAIMING
      Translation2d lookahead = ShootOnTheMoveCommandRevisedAdjusted.calculateLookaheadTarget(
        swerveSubsystem, 
        Utils.calculateShotTarget(swerveSubsystem.getPose())
      );
      turretSubsystem.aimAtFromTurretPosition(lookahead, swerveSubsystem.getPose());
    }, turretSubsystem));
    
    // Operator - Indexer
    operatorController.povUp().whileTrue(indexerSubsystem.indexerReverseCommand());

    // Named Commands
    NamedCommands.registerCommand("shootfor10s", Commands.deadline(Commands.waitSeconds(10), new ShootAtRPMCommand(shooterSubsystem, indexerSubsystem, RPM.of(3000))));

    loggerSubsystem.setDefaultCommand(new ShootOnTheMoveDebugCommand(swerveSubsystem, turretSubsystem,shooterSubsystem, indexerSubsystem, loggerSubsystem, () -> Utils.calculateShotTarget(swerveSubsystem.getPose())));
  }

  private void registerNamedCommands(){
    // Turret Commands
    //  - Aim
    NamedCommands.registerCommand("aim", Commands.run(() -> turretSubsystem.aimAtHub(), turretSubsystem));

    //  - Shoot with Fixed Turret Angle
    NamedCommands.registerCommand("shootfor3s", Commands.deadline(Commands.waitSeconds(3), new ShootDistanceBasedCommand(swerveSubsystem::getPose, shooterSubsystem, indexerSubsystem, turretSubsystem::atTarget)));
    NamedCommands.registerCommand("shootfor5s", Commands.deadline(Commands.waitSeconds(5), new ShootDistanceBasedCommand(swerveSubsystem::getPose, shooterSubsystem, indexerSubsystem, turretSubsystem::atTarget)));
    NamedCommands.registerCommand("shootfor7s", Commands.deadline(Commands.waitSeconds(5), new ShootDistanceBasedCommand(swerveSubsystem::getPose, shooterSubsystem, indexerSubsystem, turretSubsystem::atTarget)));
    
    //  - Shoot on the Move (includes aiming)
    NamedCommands.registerCommand("shoot continuous", new ShootOnTheMoveCommandRevisedAdjusted(
          swerveSubsystem, turretSubsystem, shooterSubsystem, indexerSubsystem,
          () -> Utils.calculateShotTarget(swerveSubsystem.getPose())
      ));

    // Intake Commands
    NamedCommands.registerCommand("intake down", intakeSubsystem.setStateCommand(State.OUT));
    NamedCommands.registerCommand("intake up", intakeSubsystem.setStateCommand(State.MID_HOLD));
    NamedCommands.registerCommand("intake run", intakeSubsystem.setWheelsCommand(true));
    NamedCommands.registerCommand("intake off", intakeSubsystem.setWheelsCommand(false));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
