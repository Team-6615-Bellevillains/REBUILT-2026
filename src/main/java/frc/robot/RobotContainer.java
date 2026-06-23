// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualAimCommand;
import frc.robot.commands.ManualFlywheelsCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
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

  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  IntakeSubsystem  intakeSubsystem  = new IntakeSubsystem();
  TurretSubsystem  turretSubsystem  = new TurretSubsystem();
  LedSubsystem     ledSubsystem     = new LedSubsystem();
  LoggerSubsystem  loggerSubsystem  = new LoggerSubsystem(driverController, operatorController);

  private final SendableChooser<Command> autoChooser;
  


  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    turretSubsystem.rehome();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    // Operator - Intake
    operatorController.b().onTrue(intakeSubsystem.toggleInOut());
    operatorController.a().whileTrue(intakeSubsystem.fastAgitateCommand());
    operatorController.povDown().onTrue(intakeSubsystem.setStateCommand(IntakeSubsystem.State.PULL_IN));
    operatorController.leftBumper().onTrue(intakeSubsystem.setWheelsCommand(true));
    operatorController.leftBumper().onFalse(intakeSubsystem.setWheelsCommand(false));
    
    // Operator - Indexer
    operatorController.povUp().whileTrue(indexerSubsystem.indexerReverseCommand());

    operatorController.povLeft().whileTrue(intakeSubsystem.agitateCommand());
    operatorController.povRight().whileTrue(intakeSubsystem.reverseCommand());
    
    shooterSubsystem.setDefaultCommand(new ManualFlywheelsCommand(shooterSubsystem, operatorController::getLeftX, operatorController.rightBumper()));
    turretSubsystem.setDefaultCommand(new ManualAimCommand(turretSubsystem, operatorController::getRightY));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}