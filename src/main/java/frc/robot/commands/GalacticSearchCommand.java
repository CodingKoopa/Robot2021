// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GalacticSearchCommand extends SequentialCommandGroup {
  /** Initializes the command. */
  public GalacticSearchCommand(
      DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
        // Reset the robot to its starting position.
        new InstantCommand(
            () ->
                drivetrainSubsystem.resetOdometry(
                    new Pose2d(
                        new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(90)),
                        new Rotation2d()))),
        // Drive to a target.
        new DriveToTargetCommand(drivetrainSubsystem, visionSubsystem, 3));
  }
}
