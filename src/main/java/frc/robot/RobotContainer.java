// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Speed  */

package frc.robot;

import frc.robot.Shooter.ShooterContainer;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.Shooter.ShooterConstants;
import frc.robot.Shooter.ShooterConstants.LauncherConstants;

import static frc.robot.Shooter.ShooterConstants.LauncherConstants.kLaunchFeederSpeed;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants; 
import com.ctre.phoenix6.hardware.TalonFX;

//import frc.robot.Arms.ArmContainer;

public class RobotContainer {
  private final Joystick Rdriver = new Joystick(1);
  private final Joystick Ldriver = new Joystick(0); 
  private final TalonFX lClimber = new TalonFX(31);
  private final TalonFX rClimber = new TalonFX(30);

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate =(1.25/3.0) * Math.PI; // 62% of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandXboxController lJoystick = new CommandXboxController(0); // My joystick
  //private final CommandXboxController Joystick_2 = new CommandXboxController(1); // creates another instance of joystick for when two are plugged in.
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private Command runAuto1 = drivetrain.getAutoPath("middle path");
  private Command runAuto2 = drivetrain.getAutoPath("right path");
  private Command runAuto3 = drivetrain.getAutoPath("left path");

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.25) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private static final double kDSpeedDiv =  1.3; // Value for controlling controller drive axis (top stick) sensitivity

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(()  ->  {
           return drive.withVelocityX(-lJoystick.getLeftY() * MaxSpeed/kDSpeedDiv) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-lJoystick.getLeftX() * MaxSpeed/kDSpeedDiv) // Drive left with negative X (left)
            .withRotationalRate(-lJoystick.getRightX() * MaxAngularRate); // Swerve with bottom stick
            
         } // Drive counterclockwise with negative X (left)
        ));

    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
     //lJoystick.b().whileTrue(drivetrain
       // .applyRequest(() -> point.withModuleDirection(new Rotation2d(-lJoystick.getLeftY(), -lJoystick.getLeftX()))));
      // lJoystick.x().whileTrue(drivetrain.applyRequest(() -> drive.withRotationalRate(MaxAngularRate)));
      // lJoystick.y().whileTrue(drivetrain.applyRequest(() -> drive.withRotationalRate(MaxAngularRate * -1)));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    /* Thanks to Team  Bread for helping us create Climb code */

    lJoystick.rightTrigger().onTrue(
      new InstantCommand(() -> {
        rClimber.set(0.5);
      })
    );

    lJoystick.rightTrigger().onFalse(
      new InstantCommand(() -> {
        rClimber.set(0);
      })
    );

    lJoystick.rightBumper().onTrue(
      new InstantCommand(() -> {
        rClimber.set(-0.5);
      })
    );

    lJoystick.rightBumper().onFalse(
      new InstantCommand(() -> {
        rClimber.set(0);
      })
    );

    lJoystick.leftBumper().onFalse(
      new InstantCommand(() -> {
        lClimber.set(0);
      })
    );

    lJoystick.leftBumper().onTrue(
      new InstantCommand(() -> {
        lClimber.set(-0.5);
      })
    );

    lJoystick.leftTrigger().onTrue(
      new InstantCommand(() -> {
        lClimber.set(0.5);
      })
    );

    lJoystick.leftTrigger().onFalse(
      new InstantCommand(() -> {
        lClimber.set(0);
      })
    );
  }


  public RobotContainer() {
    configureBindings();
  }
 
  public Command getAutonomousCommand() {
    //eturn new exampleAuto(s_Swerve);
    return new InstantCommand();
  }
}
