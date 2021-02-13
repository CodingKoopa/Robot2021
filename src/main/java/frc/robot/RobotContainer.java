// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import io.github.oblarg.oblog.annotations.Log;

import frc.robot.Constants.DriverStation;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.driverinput.F310Controller;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** Contains all of the robot's subsystems, commands, and button mappings. */
public class RobotContainer {
  // Subsystems

  @Log private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  @Log private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  @Log private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  @Log private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  // Commands

  // Driver Input

  private final F310Controller m_controllerDrive =
      new F310Controller(DriverStation.kPortControllerDrive);
  private final F310Controller m_controllerManip =
      new F310Controller(DriverStation.kPortControllerManip);

  // Initializes the default commands and command bindings.
  public RobotContainer() {
    // Configure default commands.

    m_drivetrainSubsystem.setDefaultCommand(
        new RunCommand(
                () ->
                    m_drivetrainSubsystem.arcadeDrive(
                        m_controllerDrive.getY(GenericHID.Hand.kLeft),
                        m_controllerDrive.getX(GenericHID.Hand.kRight)),
                m_drivetrainSubsystem)
            .withName("Arcade Drive"));

    // Configure button to command bindings.
    configureButtonBindings();
  }

  /** Configures button to command bindings. */
  private void configureButtonBindings() {
    // Configure drive speed multipliers controls.

    Command resetDriveSpeedCommand =
        new InstantCommand(
            () -> m_drivetrainSubsystem.setDriveSpeed(DrivetrainConstants.kSpeedNormal),
            m_drivetrainSubsystem);
    m_controllerDrive
        .axisLt
        .whenPressed(
            () -> m_drivetrainSubsystem.setDriveSpeed(DrivetrainConstants.kSpeedSlow),
            m_drivetrainSubsystem)
        .whenReleased(resetDriveSpeedCommand);
    m_controllerDrive
        .axisRt
        .whenPressed(
            () -> m_drivetrainSubsystem.setDriveSpeed(DrivetrainConstants.kSpeedFast),
            m_drivetrainSubsystem)
        .whenReleased(resetDriveSpeedCommand);

    // Configure ball intake controls.

    Command resetBeltCommand = new InstantCommand(m_intakeSubsystem::stopBelt, m_intakeSubsystem);
    m_controllerManip
        .buttonLb
        .whenPressed(
            () -> m_intakeSubsystem.startBelt(-IntakeConstants.kSpeedBelt), m_intakeSubsystem)
        .whenReleased(resetBeltCommand);
    m_controllerManip
        .buttonRb
        .whenPressed(
            () -> m_intakeSubsystem.startBelt(IntakeConstants.kSpeedBelt), m_intakeSubsystem)
        .whenReleased(resetBeltCommand);

    m_controllerManip
        .axisLt
        .whenPressed(
            () -> m_intakeSubsystem.startIntake(IntakeConstants.kSpeedIntake), m_intakeSubsystem)
        .whenReleased(m_intakeSubsystem::stopIntake, m_intakeSubsystem);

    // Configure ball shooting controls.

    m_controllerManip.buttonLs.whenPressed(m_shooterSubsystem::toggleSolenoid, m_shooterSubsystem);
    m_controllerManip
        .axisRt
        .whenPressed(
            () -> m_shooterSubsystem.startShooting(ShooterConstants.kSpeedShooter),
            m_shooterSubsystem)
        .whenReleased(m_shooterSubsystem::stopShooting, m_shooterSubsystem);
  }

  /**
   * Passes an the autonomous command to the Robot class.
   *
   * @return The command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    return new PrintCommand("Autonomous is not implemented yet!");
  }
}
