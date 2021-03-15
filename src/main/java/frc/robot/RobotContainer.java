// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import io.github.oblarg.oblog.annotations.Log;

import frc.robot.Constants.DriverStation;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoInitCommand;
import frc.robot.commands.DisabledInitCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.commands.GalacticSearchCommand;
import frc.robot.commands.TeleopInitCommand;
import frc.robot.commands.TurnToAngleCommand;
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

  @Log
  private final VisionSubsystem m_visionSubsystem =
      new VisionSubsystem(m_drivetrainSubsystem::getPose);

  // Commands

  private final DisabledInitCommand m_disabledInitCommand =
      new DisabledInitCommand(m_drivetrainSubsystem);
  private final AutoInitCommand m_autoInitCommand =
      new AutoInitCommand(m_drivetrainSubsystem, m_shooterSubsystem);
  private final TeleopInitCommand m_teleopInitCommand =
      new TeleopInitCommand(m_drivetrainSubsystem, m_shooterSubsystem);
  // Follows the example trajectory from the FRC Docs Trajectory Tutorial.
  private final DriveTrajectoryCommand m_exampleAutoCommand =
      new DriveTrajectoryCommand(
          m_drivetrainSubsystem,
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)));
  // Galactic Search
  private final GalacticSearchCommand m_galacticSearchCommand =
      new GalacticSearchCommand(m_drivetrainSubsystem, m_visionSubsystem);

  // Driver Input

  private final F310Controller m_controllerDrive =
      new F310Controller(DriverStation.kPortControllerDrive);
  private final F310Controller m_controllerManip =
      new F310Controller(DriverStation.kPortControllerManip);

  // Autonmous Chooser
  @Log(
      name = "Autonomous Chooser",
      width = 2,
      height = 1,
      rowIndex = 0,
      columnIndex = 10,
      tabName = "Driver View")
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  // Initializes the default commands and command bindings.
  public RobotContainer() {
    // Configure the automous chooser.
    m_autoChooser.addOption("Example Autonomous Trajectory", m_exampleAutoCommand);
    m_autoChooser.setDefaultOption("Galactic Search", m_galacticSearchCommand);
    m_autoChooser.addOption("flip flip", new TurnToAngleCommand(m_drivetrainSubsystem, 180));
    m_autoChooser.addOption("back back", new DriveStraightCommand(m_drivetrainSubsystem));

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

    m_controllerManip
        .axisLeftY
        .whenPressed(
            () ->
                m_intakeSubsystem.startBelt(
                    Math.signum(m_controllerManip.getY(GenericHID.Hand.kLeft))
                        * IntakeConstants.kSpeedBelt),
            m_intakeSubsystem)
        .whenReleased(m_intakeSubsystem::stopBelt, m_intakeSubsystem);

    m_controllerManip
        .axisRightY
        .whenPressed(
            () ->
                m_intakeSubsystem.startIntake(
                    Math.signum(m_controllerManip.getY(GenericHID.Hand.kRight))
                        * IntakeConstants.kSpeedIntake),
            m_intakeSubsystem)
        .whenReleased(m_intakeSubsystem::stopIntake, m_intakeSubsystem);

    // Configure ball shooting controls.

    m_controllerManip.buttonLs.whenPressed(m_shooterSubsystem::toggleSolenoid, m_shooterSubsystem);
    m_controllerManip
        .axisLt
        .whenPressed(
            () -> m_shooterSubsystem.startShooting(ShooterConstants.kSpeedSlow), m_shooterSubsystem)
        .whenReleased(m_shooterSubsystem::stopShooting, m_shooterSubsystem);
    m_controllerManip
        .axisRt
        .whenPressed(
            () -> m_shooterSubsystem.startShooting(ShooterConstants.kSpeedNormal),
            m_shooterSubsystem)
        .whenReleased(m_shooterSubsystem::stopShooting, m_shooterSubsystem);
  }

  /**
   * Passes the disabled init command to the Robot class.
   *
   * @return The command to run.
   */
  public Command getDisabledInitCommand() {
    return m_disabledInitCommand;
  }

  /**
   * Passes the autonomous init command to the Robot class.
   *
   * @return The command to run.
   */
  public Command getAutoInitCommand() {
    // Get the current autonomous command. This may be obtained from a SendableChooser.
    Command autoCommand = m_autoChooser.getSelected();

    // WPILibJ keeps track of every command that gets added to a group, adding them to a blacklist.
    // If you try scheduling, or creating another group with a blacklisted command, an exception is
    // thrown. Since we know that, in Robot.java, there won't be more than one init method running
    // at once, we can safely assume that this won't result in any weirdness with different
    // instances of these getting interleaved.
    CommandGroupBase.clearGroupedCommand(m_autoInitCommand);
    CommandGroupBase.clearGroupedCommand(autoCommand);

    return m_autoInitCommand.andThen(autoCommand);
  }

  /**
   * Passes the teleop init command to the Robot class.
   *
   * @return The command to run.
   */
  public Command getTeleopInitCommand() {
    return m_teleopInitCommand;
  }
}
