// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ShooterConstants {
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 0;
  }
  public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kFeederID = 21;
    public static final int kLauncherID = 22;

    // Current limit for launcher and feed wheels
    public static final int kLauncherCurrentLimit = 100;
    public static final int kFeedCurrentLimit = 100;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kLauncherSpeed = 100;
    public static final double kLaunchFeederSpeed = 100;
    public static final double kIntakeLauncherSpeed = -6;
    public static final double kIntakeFeederSpeed = -.6;

    public static final double kLauncherDelay = 2;
  }
}

