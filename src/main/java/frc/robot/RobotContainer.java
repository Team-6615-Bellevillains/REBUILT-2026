// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

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
import frc.robot.subsystems.IntakeSubsystem.State;

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


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // swerve config
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driveAngularVelocity));

    // Intake Controls

    operatorController.a().onTrue(intakeSubsystem.setStateCommand(State.OUT_ON));
    operatorController.a().onFalse(intakeSubsystem.setStateCommand(State.PULL_IN));

    // Shooter and Spindexer Controls

    operatorController.rightBumper().whileTrue(new ShootAtRPMCommand(shooterSubsystem, indexerSubsystem, RPM.of(3500)));
    operatorController.rightBumper().whileFalse(shooterSubsystem.stopCommand());
    operatorController.leftBumper().whileTrue(indexerSubsystem.indexerRunCommand());
    
    // Climber Controls
    // D-Pad Up and Down: Climb up and down
    operatorController.povUp().whileTrue(climberSubsystem.climb(0.1));
    operatorController.povDown().whileTrue(climberSubsystem.climb(-0.1));

    //operatorController.b().whileTrue(shooterSubsystem.sysId());
    //operatorController.b().onTrue(shooterSubsystem.liveRPMCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
