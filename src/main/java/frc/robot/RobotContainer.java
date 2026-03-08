// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import swervelib.SwerveInputStream;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  // controllers for the driver and operator, respectively.
  // driver is the one who controls the drivetrain and moves the robot.
  // operator controls mechanisms on the robot.
  // certain mechanisms may be assigned to the driver if needed.
  //CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);

  //SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  //IntakeSubsystem intakeSubsystem = new IntakeSubsystem(swerveSubsystem::getRobotRelativeVelocity);

  // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
  //                                                             () -> driverController.getLeftY() * -1,
  //                                                             () -> driverController.getLeftX() * -1)
  //                                                         .withControllerRotationAxis(driverController::getRightX)
  //                                                         .scaleTranslation(0.8)
  //                                                         .allianceRelativeControl(true);

  //private final SendableChooser<Command> autoChooser;
  


  public RobotContainer() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData(autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    operatorController.rightBumper().onTrue(shooterSubsystem.increaseRPMCommand(RPM.of(100)));
    operatorController.leftBumper().onTrue(shooterSubsystem.decreaseRPMCommand(RPM.of(100)));
    operatorController.povUp().onTrue(shooterSubsystem.increaseRPMCommand(RPM.of(25)));
    operatorController.povDown().onTrue(shooterSubsystem.decreaseRPMCommand(RPM.of(25)));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.stopCommand());
    operatorController.a().whileTrue(shooterSubsystem.emptyCommand());
    operatorController.b().whileTrue(indexerSubsystem.indexWhileTrueCommand(shooterSubsystem::atSetPoint));

    NamedCommands.registerCommand("shootfor10s", Commands.deadline(Commands.waitSeconds(10), new ShootAtRPMCommand(shooterSubsystem, indexerSubsystem, RPM.of(3000))));
  }

  public Command getAutonomousCommand() {
    return Commands.print("");//Commands.deadline(Commands.waitSeconds(10), new ShootAtRPMCommand(shooterSubsystem, indexerSubsystem, RPM.of(3000)));
    //return autoChooser.getSelected();
  }
}
