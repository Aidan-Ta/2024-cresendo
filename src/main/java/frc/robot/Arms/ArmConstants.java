/* 
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.Arms;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;

/**
 * All constants for a swerve module.
 */
public class ArmConstants {

    public enum ArmClosedLoopOutputType {
        Voltage,
        /** Requires Pro */
        TorqueCurrentFOC,
    }
    /** CAN ID of the drive motor. */
    public int DriveMotorId = 0;

    /** Gear ratio between the drive motor and the wheel. */
    public double DriveMotorGearRatio = 0;
    /** Radius of the driving wheel in inches. */
    public double WheelRadius = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the X axis of the robot.
     */
    public double LocationX = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the Y axis of the robot.
     */
    public double LocationY = 0;

    /**
     * The drive motor closed-loop gains.
     * <p>
     * When using closed-loop control, the drive motor uses the control output
     * type specified by {@link #DriveMotorClosedLoopOutput} and any closed-loop
     * {@link SwerveModule.DriveRequestType}.
     */
    public Slot0Configs DriveMotorGains = new Slot0Configs();

    /** The closed-loop output type to use for the drive motors. */
    public ArmClosedLoopOutputType DriveMotorClosedLoopOutput = ArmClosedLoopOutputType.Voltage;

    /** The maximum amount of stator current the drive motors can apply without slippage. */
    public double SlipCurrent = 400;

    /** True if the driving motor is reversed. */
    public boolean DriveMotorInverted = false;

    /**
     * When using open-loop drive control, this specifies the speed at which the robot travels
     * when driven with 12 volts, in meters per second. This is used to approximate the output
     * for a desired velocity. If using closed loop control, this value is ignored.
     */
    public double SpeedAt12VoltsMps = 0;


    /** Sim-specific constants **/
    /** Simulated drive inertia in kilogram meters squared. */
    public double DriveInertia = 0.001;
    /** Simulated drive voltage required to overcome friction. */
    public double DriveFrictionVoltage = 0.25;

    /**
     * Sets the CAN ID of the drive motor.
     *
     * @param id CAN ID of the drive motor
     * @return this object
     */
    public ArmConstants withDriveMotorId(int id) {
        this.DriveMotorId = id;
        return this;
    }
    /**
     * Sets the gear ratio between the drive motor and the wheel.
     *
     * @param ratio Gear ratio between the drive motor and the wheel
     * @return this object
     */
    public ArmConstants withDriveMotorGearRatio(double ratio) {
        this.DriveMotorGearRatio = ratio;
        return this;
    }

    /**
     * Sets the radius of the driving wheel in inches.
     *
     * @param radius Radius of the driving wheel in inches
     * @return this object
     */
    public ArmConstants withWheelRadius(double radius) {
        this.WheelRadius = radius;
        return this;
    }

    /**
     * Sets the location of this module's wheels relative to the physical center of the robot in
     * meters along the X axis of the robot.
     *
     * @param locationXMeters Location of this module's wheels
     * @return this object
     */
    public ArmConstants withLocationX(double locationXMeters) {
        this.LocationX = locationXMeters;
        return this;
    }

    /**
     * Sets the location of this module's wheels relative to the physical center of the robot in
     * meters along the Y axis of the robot.
     *
     * @param locationYMeters Location of this module's wheels
     * @return this object
     */
    public ArmConstants withLocationY(double locationYMeters) {
        this.LocationY = locationYMeters;
        return this;
    }

    /**
     * Sets the drive motor closed-loop gains.
     * <p>
     * When using closed-loop control, the drive motor uses the control output
     * type specified by {@link #DriveMotorClosedLoopOutput} and any closed-loop
     * {@link SwerveModule.DriveRequestType}.
     *
     * @param gains Drive motor closed-loop gains
     * @return this object
     */
    public ArmConstants withDriveMotorGains(Slot0Configs gains) {
        this.DriveMotorGains = gains;
        return this;
    }

    /**
     * Sets closed-loop output type to use for the drive motors.
     *
     * @param outputType Closed-loop output type to use for the drive motors
     * @return this object
     */
    public ArmConstants withDriveMotorClosedLoopOutput(ArmClosedLoopOutputType outputType) {
        this.DriveMotorClosedLoopOutput = outputType;
        return this;
    }

    /**
     * Sets the maximum amount of stator current the drive motors can
     * apply without slippage.
     *
     * @param slipCurrent Maximum amount of stator current
     * @return this object
     */
    public ArmConstants withSlipCurrent(double slipCurrent) {
        this.SlipCurrent = slipCurrent;
        return this;
    }

    /**
     * Sets whether the driving motor is reversed.
     *
     * @param driveMotorInverted True if the driving motor is reversed
     * @return this object
     */
    public ArmConstants withDriveMotorInverted(boolean driveMotorInverted) {
        this.DriveMotorInverted = driveMotorInverted;
        return this;
    }

    /**
     * When using open-loop drive control, this specifies the speed at which the robot travels
     * when driven with 12 volts, in meters per second. This is used to approximate the output
     * for a desired velocity. If using closed loop control, this value is ignored.
     *
     * @param speedAt12VoltsMps Speed at which the robot travels when driven with
     *                          12 volts, in meters per second
     * @return this object
     */
    public ArmConstants withSpeedAt12VoltsMps(double speedAt12VoltsMps) {
        this.SpeedAt12VoltsMps = speedAt12VoltsMps;
        return this;
    }

    /**
     * Sets the simulated drive inertia in kilogram meters squared.
     *
     * @param driveInertia Drive inertia in kilogram meters squared
     * @return this object
     */
    public ArmConstants withDriveInertia(double driveInertia) {
        this.DriveInertia = driveInertia;
        return this;
    }

    /**
     * Sets the simulated drive voltage required to overcome friction.
     *
     * @param voltage Drive voltage required to overcome friction
     * @return this object
     */
    public ArmConstants withDriveFrictionVoltage(double voltage) {
        this.DriveFrictionVoltage = voltage;
        return this;
    }
}