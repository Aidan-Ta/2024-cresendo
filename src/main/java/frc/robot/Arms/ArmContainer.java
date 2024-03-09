/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.Arms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Arms.ArmConstants;

/**
 * Swerve Module class that encapsulates a swerve module powered by CTR
 * Electronics devices.
 * <p>
 * This class handles the hardware devices and configures them for
 * swerve module operation using the Phoenix 6 API.
 * <p>
 * This class constructs hardware devices internally, so the user
 * only specifies the constants (IDs, PID gains, gear ratios, etc).
 * Getters for these hardware devices are available.
 */
public class ArmContainer {
    /**
     * All possible control requests for the module drive motor.
     */
    public enum DriveRequestType {
        /**
         * Control the drive motor using an open-loop voltage request.
         */
        OpenLoopVoltage,
        /**
         * Control the drive motor using a velocity closed-loop request.
         * The control output type is determined by {@link ArmConstants#DriveMotorClosedLoopOutput}
         */
        Velocity,
    }

    private final TalonFX m_driveMotor;

    private final StatusSignal<Double> m_drivePosition;
    private final StatusSignal<Double> m_driveVelocity;
    private final BaseStatusSignal[] m_signals;
    private final double m_driveRotationsPerMeter;

    private final double m_speedAt12VoltsMps;

    /* drive motor controls */
    private final VoltageOut m_voltageOpenLoopSetter = new VoltageOut(0);
    private final VelocityVoltage m_velocityVoltageSetter = new VelocityVoltage(0);
    /* Velocity Torque current neutral should always be coast, as neutral corresponds to 0-current or maintain velocity, not 0-velocity */
    private final VelocityTorqueCurrentFOC m_velocityTorqueSetter = new VelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);

    private final ArmConstants.ArmClosedLoopOutputType m_driveClosedLoopOutput;

    /**
     * Construct a ArmContainer with the specified constants.
     *
     * @param constants   Constants used to construct the module
     * @param canbusName  The name of the CAN bus this module is on
     */
    public ArmContainer(ArmConstants constants, String canbusName) {
        m_driveMotor = new TalonFX(constants.DriveMotorId, canbusName);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonConfigs.Slot0 = constants.DriveMotorGains;
        talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        talonConfigs.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        talonConfigs.MotorOutput.Inverted = constants.DriveMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        StatusCode response = m_driveMotor.getConfigurator().apply(talonConfigs);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + m_driveMotor.getDeviceID() + " failed config with error " + response.toString());
        }

        /* Undo changes for torqueCurrent */
        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();
        /* And to current limits */
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();

        talonConfigs.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for swerve modules

        m_drivePosition = m_driveMotor.getPosition().clone();
        m_driveVelocity = m_driveMotor.getVelocity().clone();

        m_signals = new BaseStatusSignal[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;

        m_velocityTorqueSetter.UpdateFreqHz = 0;
        m_velocityVoltageSetter.UpdateFreqHz = 0;
        m_voltageOpenLoopSetter.UpdateFreqHz = 0;

        /* Set the drive motor closed-loop output type */
        m_driveClosedLoopOutput = constants.DriveMotorClosedLoopOutput;

        /* Get the expected speed when applying 12 volts */
        m_speedAt12VoltsMps = constants.SpeedAt12VoltsMps;
    }

}
