// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class TurnToAngleCommand extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnToAngleCommand(DrivetrainSubsystem drivetrainSubsystem, double targetAngleDegrees) {
    super(
        new ProfiledPIDController(
            DrivetrainConstants.Feedback.kPAngular,
            0,
            0,
            new TrapezoidProfile.Constraints(
                DrivetrainConstants.kMaxTurnVelocity,
                DrivetrainConstants.kMaxTrajectoryAcceleration)),
        // Close loop on heading
        drivetrainSubsystem::getHeading,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        (output, setpoint) -> drivetrainSubsystem.arcadeDrive(0, output),
        // Require the drive
        drivetrainSubsystem);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    // TODO: constants?
    getController().setTolerance(5, 10);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
