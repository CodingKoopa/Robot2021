// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.RoboRIO;

/**
 * Represents the drivetrain subsystem.
 *
 * <p>The drivetrain encapsulates the wheels the robot moves around with, the motors that power
 * them, and the encoders that measure their movement. This subsystem also encapsulates the
 * gyrometer, because it is tightly integrated with the robot driving.
 */
public class DrivetrainSubsystem extends SubsystemBase implements Loggable {
  // Motor Controllers

  private final WPI_TalonSRX m_motorFrontLeft =
      new WPI_TalonSRX(RoboRIO.CAN.kPortMotorDriveFrontLeft);
  private final WPI_TalonSRX m_motorFrontRight =
      new WPI_TalonSRX(RoboRIO.CAN.kPortMotorDriveFrontRight);
  private final WPI_VictorSPX m_motorBackLeft =
      new WPI_VictorSPX(RoboRIO.CAN.kPortMotorDriveBackLeft);
  private final WPI_VictorSPX m_motorBackRight =
      new WPI_VictorSPX(RoboRIO.CAN.kPortMotorDriveBackRight);

  // Quadrature Encoders (Measures relative distance)

  @Log(name = "Left Encoder (Relative)", width = 2, height = 1, rowIndex = 0, columnIndex = 3)
  private final Encoder m_encoderRelativeLeft =
      new Encoder(
          RoboRIO.DIO.kPortsEncoderDriveLeft[0],
          RoboRIO.DIO.kPortsEncoderDriveLeft[1],
          RoboRIO.DIO.kPortsEncoderDriveLeft[2]);

  @Log(name = "Right Encoder (Relative)", width = 2, height = 1, rowIndex = 0, columnIndex = 5)
  // This encoder must be reversed so that values will be consistent with the left drive encoder.
  private final Encoder m_encoderRelativeRight =
      new Encoder(
          RoboRIO.DIO.kPortsEncoderDriveRight[0],
          RoboRIO.DIO.kPortsEncoderDriveRight[1],
          RoboRIO.DIO.kPortsEncoderDriveRight[2],
          true);

  // Duty Cycle Encoders (Measures absolute distance)

  @Log(name = "Left Encoder (Absolute)", width = 2, height = 2, rowIndex = 1, columnIndex = 3)
  private final DutyCycleEncoder m_encoderAbsoluteLeft =
      new DutyCycleEncoder(RoboRIO.DIO.kPortsEncoderDriveLeft[3]);

  @Log(name = "Right Encoder (Absolute)", width = 2, height = 2, rowIndex = 1, columnIndex = 5)
  private final DutyCycleEncoder m_encoderAbsoluteRight =
      new DutyCycleEncoder(RoboRIO.DIO.kPortsEncoderDriveRight[3]);

  // Gyroscope
  @Log(name = "Gyroscope", width = 2, height = 2, rowIndex = 0, columnIndex = 7)
  private final Gyro m_gyro = new ADXRS450_Gyro();

  @Log(name = "Drive", width = 2, height = 3, rowIndex = 0, columnIndex = 0)
  @Log(
      name = "Drive",
      width = 2,
      height = 3,
      rowIndex = 0,
      columnIndex = 7,
      tabName = "Driver View")
  // Drive Class
  private final DifferentialDrive m_differentialDrive =
      new DifferentialDrive(m_motorFrontLeft, m_motorFrontRight);

  /** Initializes the drivetrain subsystem. */
  public DrivetrainSubsystem() {
    // Make the back motors follow the front motors.
    m_motorBackLeft.follow(m_motorFrontLeft);
    m_motorBackRight.follow(m_motorFrontRight);
  }

  /**
   * Drives the robot using arcade drive.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *     positive.
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    m_differentialDrive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * Sets the speed that the robot drives at.
   *
   * @param speed The value will be used as the scaling factor [0..1.0].
   */
  public void setDriveSpeed(double speed) {
    m_differentialDrive.setMaxOutput(speed);
  }
}
