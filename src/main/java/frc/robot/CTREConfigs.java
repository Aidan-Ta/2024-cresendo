package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration; 
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
        TalonFXConfiguration angleSupplyLimit = new TalonFXConfiguration();
        

        CurrentLimitsConfigs currentLimitsVariables = new CurrentLimitsConfigs()  
        currentLimitsVariables.withSupplyCurrentLimitEnable(
            Constants.Swerve.angleEnableCurrentLimit);
        currentLimitsVariables.withStatorCurrentLimit(
            Constants.Swerve.angleContinuousCurrentLimit);
            currentLimitsVariables.withSupplyCurrentThreshold(
                Constants.Swerve.anglePeakCurrentLimit);
                currentLimitsVariables.withSupplyTimeThreshold(
            Constants.Swerve.anglePeakCurrentDuration);

        angleSupplyLimit.withCurrentLimits(currentLimitsVariables)
        
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.Slot0.kS = Constants.Swerve.angleKF;
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        TalonFXConfiguration driveSupplyLimit = new CurrentLimits(
            Constants.Swerve.driveEnableCurrentLimit, 
            Constants.Swerve.driveContinuousCurrentLimit, 
            Constants.Swerve.drivePeakCurrentLimit, 
            Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kS = Constants.Swerve.driveKS;  // was kF      
        swerveDriveFXConfig.CurrentLimits = SupplyCurrentLimit;
        swerveDriveFXConfig.OpenLoopRamps = Constants.Swerve.OpenLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps = Constants.Swerve.ClosedLoopRamps;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.AbsoluteSensorRangeValue = AbsoluteSensorRangeValue.Unsigned_0To1_to_360;
       // swerveCanCoderConfig.SensorDirectionValue = Constants.Swerve.canCoderInvert;
       // swerveCanCoderConfig.SensorInitializationStrategy= SensorInitializationStrategy.BootToAbsolutePosition; 
       // initalaztion is not needed anymore
        //swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        
    }
}

