// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // controllers for the driver and operator, respectively.
  // driver is the one who controls the drivetrain and moves the robot.
  // operator controls mechanisms on the robot.
  // certain mechanisms may be assigned to the driver if needed.
  CommandXboxController driverController = new CommandXboxController(0);
  CommandXboxController operatorController = new CommandXboxController(1);

  ClimberSubsystem climberSubsystem;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  SwerveSubsystem swerveSubsystem;

  // initialization for the whole robot.
  public RobotContainer() {
    climberSubsystem = new ClimberSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    swerveSubsystem = new SwerveSubsystem();
    configureBindings();
  }

  // where button mappings are set up.
  private void configureBindings() {

    // =====================================
    // ========== Intake Controls ==========
    // =====================================

    // Both Triggers: Intake in and out
    operatorController.leftTrigger().and(operatorController.rightTrigger()).onTrue(LinkageSubsystem.toggleLinkage());
    linkageSubsystem.setDefaultCommand(linkageSubsystem.holdLinkage());

    // Both Bumpers: Toggle intake spin
    operatorController.leftBumper().and(operatorController.rightBumper()).toggleOnTrue(intakeSubsystem.spinIntake(3000));

    // ====================================================
    // ========== Shooter and Spindexer Controls ==========
    // ====================================================

    operatorController.x()
      .onTrue(Command.run(() -> shooterSubsystem.stop()));
    operatorController.a()
      .onTrue(shooterSubsystem.spinShooter(2500));
    operatorController.b()
        .onTrue(shooterSubsystem.spinShooter(3500));
    operatorController.y()
        .onTrue(shooterSubsystem.spinShooter(4500));

    // ======================================
    // ========== Climber Controls ==========
    // ======================================

    operatorController.povUp().onTrue(climberSubsystem.climb(0.5));
    operatorController.povDown().onTrue(climberSubsystem.climb(-0.5));
    operatorController.povUp().or(operatorController.povDown()).onFalse(climberSubsystem.stop());
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
