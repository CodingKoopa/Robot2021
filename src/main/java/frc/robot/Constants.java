// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Constants for ports on the robot and driver station, and for each subsystem. */
public final class Constants {
  /**
   * Constants for ports that are on the roboRIO.
   *
   * <p>The SPI port to which the Gyroscope is connected is not included here because that constant
   * is provided by WPILib.
   */
  public static final class RoboRIO {
    /** Constants for the Control Area Network/CAN bus. */
    public static final class CAN {
      /** The port of the front left drive motor. */
      public static final int kPortMotorDriveFrontLeft = 1;
      /** The port of the back left drive motor. */
      public static final int kPortMotorDriveBackLeft = 2;
      /** The port of the front right drive motor. */
      public static final int kPortMotorDriveFrontRight = 3;
      /** The port of the back right drive motor. */
      public static final int kPortMotorDriveBackRight = 4;
      /** The ports of the launcher cannon double solenoid. */
      public static final int[] kPortsDoubleSolenoidLauncherCannon = {5, 6};
      /** The port of the intake motor. */
      public static final int kPortMotorIntake = 7;
      /** The port of the left launcher motor. */
      public static final int kPortMotorLauncherLeft = 8;
      /** The port of the conveyor belt. */
      public static final int kPortMotorBelt = 10;
      /**
       * The port of the right launcher motor. On the electronics board, this is #8b, but this
       * couldn't be replicated due to software limitaitons.
       */
      public static final int kPortMotorLauncherRight = 11;
    }

    /** Constants for the Digital Input/Output ports. */
    public static final class DIO {
      /** The ports of the left drive encoder. */
      public static final int[] kPortsEncoderDriveLeft = {0, 1, 2, 3};
      /** The ports of the right drive encoder. */
      public static final int[] kPortsEncoderDriveRight = {4, 5, 6, 7};
      /** The port of the photoelectric sensor at the bottom of the storage, where balls enter. */
      public static final int kPortPhotoelectricSensorEnter = 8;
      /** The port of the photoelectric sensor at the top of the storage, where balls leave. */
      public static final int kPortPhotoelectricSensorExit = 9;
    }

    /** Constants for the Analog Input ports. */
    public static final class AI {
      /** The port of the ultrasonic sensor. */
      public static final int kPortUltrasonicSensorPort = 0;
    }
  }

  /** Constants for ports that are on the driver station. */
  public static final class DriverStation {
    /** The port of the driver controller. */
    public static final int kPortControllerDrive = 0;
    /** The port of the manipulator controller. */
    public static final int kPortControllerManip = 1;
  }

  /** Constants for the drivetrain subsystem. */
  public static final class DrivetrainConstants {
    // TODO: Check these speeds - especially kSpeedFast.
    /** The slow speed to drive at. */
    public static final double kSpeedSlow = 0.5;
    /** The normal speed to drive at. */
    public static final double kSpeedNormal = 0.85;
    /** The fast speed to drive at. */
    public static final double kSpeedFast = 1.0;
  }

  /** Constants for the intake subsystem. */
  public static final class IntakeConstants {
    /** The speed to intake balls at. */
    public static final double kSpeedIntake = 0.95;
    /** The speed to advance balls through the belt at. */
    public static final double kSpeedBelt = 1.0;
  }

  /** Constants for the Shooter subsystem. */
  public static final class ShooterConstants {
    /** The speed to shoot balls at. */
    public static final double kSpeedShooter = 1.0;
  }
}
