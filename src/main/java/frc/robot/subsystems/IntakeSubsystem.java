// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.RoboRIO;

/**
 * Represents the intake subsystem.
 *
 * <p>The intake subsystem encapsulates the motors used to feed balls into the robot, any sensors
 * used to keep track of them, and the motors used to transport the balls through the robot.
 */
public class IntakeSubsystem extends SubsystemBase implements Loggable {
  // Motor Controllers

  @Log(name = "Intake Motor", width = 2, height = 1, rowIndex = 0, columnIndex = 0)
  @Log(
      name = "Intake Motor",
      width = 3,
      height = 1,
      rowIndex = 3,
      columnIndex = 7,
      tabName = "Driver View")
  private final WPI_TalonSRX m_motorIntake = new WPI_TalonSRX(RoboRIO.CAN.kPortMotorIntake);

  @Log(name = "Belt Motor", width = 2, height = 1, rowIndex = 1, columnIndex = 0)
  @Log(
      name = "Belt Motor",
      width = 3,
      height = 1,
      rowIndex = 4,
      columnIndex = 7,
      tabName = "Driver View")
  private final WPI_VictorSPX m_motorBelt = new WPI_VictorSPX(RoboRIO.CAN.kPortMotorBelt);

  /**
   * Starts the ball intake motor to take in balls.
   *
   * @param speed The speed to set the motor to.
   */
  public void startIntake(double speed) {
    m_motorIntake.set(speed);
  }

  /** Stops the ball intake motor. */
  public void stopIntake() {
    m_motorIntake.set(0);
  }

  /**
   * Starts the belt motor to move the balls forwards or backwards.
   *
   * @param speed The speed to set the motor to.
   */
  public void startBelt(double speed) {
    m_motorBelt.set(speed);
  }

  /** Stops the belt motor. */
  public void stopBelt() {
    m_motorBelt.set(0);
  }
}
