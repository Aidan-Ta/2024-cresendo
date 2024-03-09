// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants; 

//import frc.robot.Arms.ArmContainer;

public class RobotContainer {
  private final Joystick Rdriver = new Joystick(1);
  private final Joystick Ldriver = new Joystick(0); 

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.25 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController lJoystick = new CommandXboxController(0); // My joystick
  //private final CommandXboxController Joystick_2 = new CommandXboxController(1); // creates another instance of joystick for when two are plugged in.
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.50) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public static final double kDSpeedDiv =  1.0; // Value for controlling controller sensitivity
  public static final double kAAngleDiv =  0.25; // Value for controlling controller sensitivity

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(()  ->  {
          System.out.println("joystick.getRightX=" + -lJoystick.getRightX() );
           return drive.withVelocityX(-lJoystick.getLeftY() * MaxSpeed/kDSpeedDiv) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-lJoystick.getLeftX() * MaxSpeed/kDSpeedDiv) // Drive left with negative X (left)
            .withRotationalRate(-lJoystick.getRightX()/kAAngleDiv * MaxAngularRate);
            
         } // Drive counterclockwise with negative X (left)
        ));

    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
     //lJoystick.b().whileTrue(drivetrain
       // .applyRequest(() -> point.withModuleDirection(new Rotation2d(-lJoystick.getLeftY(), -lJoystick.getLeftX()))));
      lJoystick.x().whileTrue(drivetrain.applyRequest(() -> drive.withRotationalRate(MaxAngularRate)));
      lJoystick.y().whileTrue(drivetrain.applyRequest(() -> drive.withRotationalRate(MaxAngularRate * -1)));

    // reset the field-centric heading on left bumper press
    //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }


  public RobotContainer() {
    configureBindings();
  }
 
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
    //return Commands.print("No autonomous command configured");
  }
}
