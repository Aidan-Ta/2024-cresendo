package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// old stuff below
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
/* import com.ctre.phoenix6.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.sensors.SensorTimeBase; */

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig; // CANCoderConfiguration is now CANcoderConfiguration

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleCurrentLimits = new CurrentLimitsConfigs();
        angleCurrentLimits.withSupplyCurrentLimitEnable(
            Constants.Swerve.angleEnableCurrentLimit);
        // This might need to be .withStatorCurrentLimit
        angleCurrentLimits.withSupplyCurrentLimit(
            Constants.Swerve.angleContinuousCurrentLimit);
        angleCurrentLimits.withSupplyCurrentThreshold(
            Constants.Swerve.anglePeakCurrentLimit);
        angleCurrentLimits.withSupplyTimeThreshold(
            Constants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.Slot0.kV = Constants.Swerve.angleKV;
        swerveAngleFXConfig.CurrentLimits = angleCurrentLimits;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs();
        driveCurrentLimits.withSupplyCurrentLimitEnable(
            Constants.Swerve.driveEnableCurrentLimit);
        // This might need to be .withStatorCurrentLimit
        driveCurrentLimits.withSupplyCurrentLimit(
            Constants.Swerve.driveContinuousCurrentLimit);
        driveCurrentLimits.withSupplyCurrentThreshold(
            Constants.Swerve.drivePeakCurrentLimit);
        driveCurrentLimits.withSupplyTimeThreshold(
            Constants.Swerve.drivePeakCurrentDuration);        

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kS = Constants.Swerve.driveKS;       
        swerveDriveFXConfig.CurrentLimits = driveCurrentLimits;
        /* Swerve Open Loop Ramp Configuration */
        OpenLoopRampsConfigs openLoopRampConfig = new OpenLoopRampsConfigs();
        openLoopRampConfig.withDutyCycleOpenLoopRampPeriod(Constants.Swerve.OpenLoopRamps);
        swerveDriveFXConfig.OpenLoopRamps = openLoopRampConfig;

        /* Swerve Closed Loop Ramp Configuration */
        ClosedLoopRampsConfigs closedLoopRampConfig = new ClosedLoopRampsConfigs();
        closedLoopRampConfig.withDutyCycleClosedLoopRampPeriod(Constants.Swerve.ClosedLoopRamps);
        swerveDriveFXConfig.ClosedLoopRamps = closedLoopRampConfig;
        
        /* Swerve CANCoder Configuration */
        //swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;


        
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        
    }
}

