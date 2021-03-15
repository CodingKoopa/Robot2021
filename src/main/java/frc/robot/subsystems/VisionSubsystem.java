// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

/**
 * Represents the vision subsystem.
 *
 * <p>The vision subsystem encapsulates managing cameras attached to the roboRIO, and interfacing
 * with coprocessors to provide object tracking data from the other camera.
 */
public class VisionSubsystem extends SubsystemBase implements Loggable {
  @Log.CameraStream(
      name = "Camera",
      width = 7,
      height = 6,
      rowIndex = 0,
      columnIndex = 0,
      tabName = "Driver View")
  // USB Camera Class
  UsbCamera m_camera;

  // PhotonLib Camera Class
  private PhotonCamera m_photonCamera = new PhotonCamera(VisionConstants.kCamName);

  // PhotonLib Simulation
  // Shooter Low
  private final SimVisionSystem m_photonSimLow =
      new SimVisionSystem(
          VisionConstants.kCamName,
          VisionConstants.kCamDiagonalFOV,
          VisionConstants.kCamPitchLow,
          VisionConstants.kTransCamToRobot,
          VisionConstants.kCamHeightOffGroundLow,
          VisionConstants.kCamMaxLEDRange,
          VisionConstants.kCamResolutionWidth,
          VisionConstants.kCamResolutionHeight,
          VisionConstants.kMinTargetArea);
  // Shooter High
  private final SimVisionSystem m_photonSimHigh =
      new SimVisionSystem(
          VisionConstants.kCamName,
          VisionConstants.kCamDiagonalFOV,
          VisionConstants.kCamPitchHigh,
          VisionConstants.kTransCamToRobot,
          VisionConstants.kCamHeightOffGroundHigh,
          VisionConstants.kCamMaxLEDRange,
          VisionConstants.kCamResolutionWidth,
          VisionConstants.kCamResolutionHeight,
          VisionConstants.kMinTargetArea);

  // Pose Supplier (Used for simulation)
  private final Supplier<Pose2d> m_poseSupplier;

  // Current best tracked target.
  private PhotonTrackedTarget m_bestTarget = null;

  @Log(name = "Has Targets")
  private boolean m_hasTargets = false;

  // Low-Pass Filter
  LinearFilter m_lowPassFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  // High-Pass Filter
  LinearFilter m_highPassFilter = LinearFilter.highPass(0.1, 0.02);

  /** Initializes the vision subsystem. */
  public VisionSubsystem(Supplier<Pose2d> poseSupplier) {
    m_poseSupplier = poseSupplier;

    // If we're runnning on the real robot, enable the camera.
    if (RobotBase.isReal()) {
      m_camera = CameraServer.getInstance().startAutomaticCapture();
      m_camera.setResolution(960, 720);
    } else {
      SimVisionTarget tgt =
          new SimVisionTarget(
              new Pose2d(
                  new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(90)),
                  Rotation2d.fromDegrees(180)),
              0,
              Units.inchesToMeters(7),
              Units.inchesToMeters(7));
      for (SimVisionSystem photonSim : List.of(m_photonSimLow, m_photonSimHigh)) {
        photonSim.addSimVisionTarget(tgt);
      }
    }
  }

  /** This method is run periodically. */
  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      m_photonSimHigh.processFrame(m_poseSupplier.get());
    }

    // Get the latest pipeline result.
    PhotonPipelineResult pipelineResult = m_photonCamera.getLatestResult();

    // Update the telemetry.
    m_hasTargets = pipelineResult.hasTargets();

    // Update the best target.
    if (m_hasTargets) m_bestTarget = pipelineResult.getBestTarget();
    else m_bestTarget = null;
  }

  /**
   * Returns the latest pipeline result.
   *
   * @return The latest pipeline result.
   */
  public PhotonTrackedTarget getBestTarget() {
    return m_bestTarget;
  }
}
