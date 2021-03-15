// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

public class DriveToTargetCommand extends CommandBase {
  // Drivetrain Subsystem
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  // Vision Subsystem
  private final VisionSubsystem m_visionSubsystem;

  // PID Controllers
  // These are named according to the Robot Coordinate System:
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system

  // PID controller for the drivetrain movement along the x-axis.
  private final PIDController m_controllerX =
      new PIDController(DrivetrainConstants.Feedback.kP, 0, 0);
  // PID controller for the drivetrain's theta angle.
  private final PIDController m_controllerTheta =
      new PIDController(DrivetrainConstants.Feedback.kPAngular, 0, 0);

  private final double m_goalRange;

  private boolean m_isFinished = false;

  /** Initializes the command. */
  public DriveToTargetCommand(
      DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem, double goalRange) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_goalRange = goalRange;

    addRequirements(drivetrainSubsystem);
  }

  /** This method is run periodically while the command is scheduled. */
  @Override
  public void execute() {
    PhotonTrackedTarget target = m_visionSubsystem.getBestTarget();
    if (target != null) {
      SmartDashboard.putNumber("Area", target.getArea());

      double range =
          PhotonUtils.calculateDistanceToTargetMeters(
              VisionConstants.kCamHeightOffGroundLow,
              0.0,
              Units.degreesToRadians(VisionConstants.kCamPitchLow),
              target.getPitch());

      double forwardSpeed = m_controllerX.calculate(range, m_goalRange);
      double rotationSpeed = m_controllerTheta.calculate(target.getYaw(), 0);

      m_drivetrainSubsystem.arcadeDrive(forwardSpeed, rotationSpeed);

    } else {
      m_drivetrainSubsystem.arcadeDrive(0, 0.25);
      m_isFinished = false;
    }
  }

  /** This method is run when the command ends. */
  @Override
  public void end(boolean interrupted) {
    if (interrupted) m_drivetrainSubsystem.tankDriveVolts(0, 0);
  }

  /**
   * Whether the command has finished. Once a command finishes, the scheduler will call its end()
   * method and un-schedule it.
   *
   * @return whether the command has finished.
   */
  @Override
  public boolean isFinished() {
    return m_controllerX.atSetpoint() && m_controllerTheta.atSetpoint();
  }
}
