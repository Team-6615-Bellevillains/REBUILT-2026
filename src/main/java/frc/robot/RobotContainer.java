// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootAtRPMCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  // controllers for the driver and operator, respectively.
  // driver is the one who controls the drivetrain and moves the robot.
  // operator controls mechanisms on the robot.
  // certain mechanisms may be assigned to the driver if needed.
  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);

  ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

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

    operatorController.y().whileTrue(new ShootAtRPMCommand(shooterSubsystem, indexerSubsystem, RPM.of(3500)));
    operatorController.x().whileTrue(new ShootAtRPMCommand(shooterSubsystem, indexerSubsystem, RPM.of(3000)));
    operatorController.a().whileTrue(new ShootAtRPMCommand(shooterSubsystem, indexerSubsystem, RPM.of(2500)));
    operatorController.y().or(operatorController.x()).or(operatorController.a()).whileFalse(shooterSubsystem.stopCommand());
    operatorController.leftBumper().whileTrue(indexerSubsystem.indexerRunCommand());
    operatorController.povUp().whileTrue(indexerSubsystem.indexerReverseCommand());
    
    // Climber Controls
    // D-Pad Up and Down: Climb up and down
    driverController.leftBumper().whileTrue(climberSubsystem.climb(0.3));
    driverController.rightBumper().whileTrue(climberSubsystem.climb(-0.3));
    driverController.a().onTrue(swerveSubsystem.resetGyroCommand());

    //operatorController.b().whileTrue(shooterSubsystem.sysId());
    //operatorController.b().onTrue(shooterSubsystem.liveRPMCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.deadline(Commands.waitSeconds(10), new ShootAtRPMCommand(shooterSubsystem, indexerSubsystem, RPM.of(3000)));
    //return autoChooser.getSelected();
  }
}
