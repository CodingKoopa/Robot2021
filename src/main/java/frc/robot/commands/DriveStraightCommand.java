// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveStraightCommand extends CommandBase {
  // Drivetrain Subsystem
  private final DrivetrainSubsystem m_drivetrainSubsystem;

  // PID Controllers
  // These are named according to the Robot Coordinate System:
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system

  // PID controller for the drivetrain's theta angle.
  private final PIDController m_controllerTheta =
      new PIDController(DrivetrainConstants.Feedback.kPAngular, 0, 0);

  private double m_setpoint = 0;

  /** Initializes the command. */
  public DriveStraightCommand(DrivetrainSubsystem drivetrainSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(drivetrainSubsystem);
  }

  /** This method is run when the command is initially scheduled. */
  @Override
  public void initialize() {
    m_setpoint = m_drivetrainSubsystem.getHeading();
  }

  /** This method is run periodically while the command is scheduled. */
  @Override
  public void execute() {
    double rotationSpeed =
        m_controllerTheta.calculate(m_drivetrainSubsystem.getHeading(), m_setpoint);

    m_drivetrainSubsystem.arcadeDrive(-0.3, rotationSpeed);
  }

  /** This method is run when the command ends. */
  @Override
  public void end(boolean interrupted) {
    if (interrupted) m_drivetrainSubsystem.tankDriveVolts(0, 0);
  }
}
