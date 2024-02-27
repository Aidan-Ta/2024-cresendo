package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

//import com.kauailabs.navx.frc.AHRS; 

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    /* Motor Start Position */
    public static final double motorStartPosition = 0;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            final DutyCycleOut m_request = new DutyCycleOut(0);
            mDriveMotor.setControl(m_request.withOutput(percentOutput));
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            VelocityDutyCycle m_velocity = new VelocityDutyCycle(0);
            m_velocity.withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond));
            mDriveMotor.setControl(m_velocity.withVelocity(velocity));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        /* 
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        boolean useLastAngle = Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01);
       // System.out.println("desired state speed=" + desiredState.speedMetersPerSecond + " desired state angle=" + desiredState.angle);
       /*  System.out.println("current angle ="+angle); // for troubleshooting swerve drive
        System.out.println("current last angle ="+lastAngle); */ 
       // System.out.println("CANcoder deg =" + CANcoderToDegrees.positionCounts);
        final PositionVoltage m_position = new PositionVoltage(0);

     


                                                         // was prev CANcoderToDegrees
     mAngleMotor.setControl(m_position.withPosition(desiredState.angle.getRotations()));
        // mAngleMotor.setControl(m_position.withPosition(Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio)));
       // lastAngle = angle;
    }

    private Rotation2d getAngle(){
      //Rotation2d angle = Rotation2d.fromDegrees(Conversions.CANcoderToDegrees(mAngleMotor.getPosition().refresh().getValue(), Constants.Swerve.angleGearRatio));
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor.getPosition().refresh().getValue());
    //    System.out.println("get angle =" + angle);
        return angle;
    }

    public Rotation2d getCanCoder(){
      return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().refresh().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.CANcoderToDegrees(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(new TalonFXConfiguration());
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.getConfigurator().apply(new TalonFXConfiguration());
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setPosition(motorStartPosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
                              //mDriveMotor
        Conversions.falconToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}