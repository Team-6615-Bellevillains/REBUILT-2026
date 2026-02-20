// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootCommand;
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
  //IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

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

    //operatorController.x().onTrue(intakeSubsystem.setStateCommand(State.IN));
    //operatorController.y().onTrue(intakeSubsystem.setStateCommand(State.OUT_OFF));
    //operatorController.b().onTrue(intakeSubsystem.setStateCommand(State.OUT_ON));

    // Shooter and Spindexer Controls

    operatorController.rightBumper().whileTrue(new ShootCommand(shooterSubsystem, indexerSubsystem, 3500));
    operatorController.leftBumper().whileTrue(indexerSubsystem.indexerRunCommand());
    
    // Climber Controls
    // D-Pad Up and Down: Climb up and down
    operatorController.povUp().onTrue(climberSubsystem.climb(0.5));
    operatorController.povDown().onTrue(climberSubsystem.climb(-0.5));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
