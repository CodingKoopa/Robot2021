// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.RoboRIO;

/**
 * Represents the shooter subsystem.
 *
 * <p>The shooter subsystem encapsulates the motors used to shoot balls out of the cannon.
 */
public class ShooterSubsystem extends SubsystemBase implements Loggable {
  // Motor Controllers

  @Log(name = "Shooter Motor", width = 2, height = 1, rowIndex = 0, columnIndex = 0)
  @Log(
      name = "Shooter Motor",
      width = 3,
      height = 1,
      rowIndex = 5,
      columnIndex = 7,
      tabName = "Driver View")
  private final WPI_VictorSPX m_motorShooterLeft =
      new WPI_VictorSPX(RoboRIO.CAN.kPortMotorLauncherLeft);

  private final WPI_TalonSRX m_motorShooterRight =
      new WPI_TalonSRX(RoboRIO.CAN.kPortMotorLauncherRight);

  // Solenoids

  // We can't log this as-is because it is added to Shuffleboard as a Sendable which is an actuator.
  // Actuator widgets can be used to set the value of the hardware they control, when LiveWindow
  // is enabled - so, in test mode. As a safety feature, whenever we change to a mode other than
  // test mode, every actuator is reset to a "safe" value in order to ensure that there won't be
  // a bad situation created by values that were set in test mode. Unfortunately, even though we
  // don't use test mode, these safety checks will still run when changing modes.
  //
  // Ultimately, the result of this is that, if we add the double solenoid as a Sendable, it will
  // be subject to safety checks, and, when entering teleoperated mode, get set to the kOff state.
  // This will prevent the toggle method from working properly. Rather then logging the solenoid
  // object, we log getSolenoidValue().
  private final DoubleSolenoid m_doubleSolenoidShooter =
      new DoubleSolenoid(
          RoboRIO.CAN.kPortsDoubleSolenoidLauncherCannon[0],
          RoboRIO.CAN.kPortsDoubleSolenoidLauncherCannon[1]);

  /** Initializes the shooter subsystem. */
  public ShooterSubsystem() {
    // Make the right motor follow the left.
    m_motorShooterRight.follow(m_motorShooterLeft);

    // Initialize the position of the double solenoid.
    m_doubleSolenoidShooter.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Starts the shooter motor to shoot balls.
   *
   * @param speed The speed to set the motor to.
   */
  public void startShooting(double speed) {
    m_motorShooterLeft.set(speed);
  }

  /** Stops the shooter motor. */
  public void stopShooting() {
    m_motorShooterLeft.set(0);
  }

  /** Toggles the double solenoids to raise or retract the shooter. */
  public void toggleSolenoid() {
    m_doubleSolenoidShooter.toggle();
  }

  /**
   * Gets the value of the double solenoid as a string, rather than the Value enum.
   *
   * @return The name of the enum value corresponding to the solenoid state.
   */
  @Log(name = "Shooter Solenoid", width = 2, height = 1, rowIndex = 1, columnIndex = 0)
  private String getSolenoidValue() {
    return m_doubleSolenoidShooter.get().name();
  }
}
