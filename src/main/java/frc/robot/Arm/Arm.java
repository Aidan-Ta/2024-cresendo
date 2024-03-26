/*  
package frc.robot.Arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.Utils;

public class Arm {

    // use 1 xbox controller
    //public final CommandXboxController  = new CommandXboxController(0);

    private final TalonFX lClimber = new TalonFX(31); // left climber 
    private final TalonFX rClimber = new TalonFX(30); // right climber

    //
    
    public void ClimbWithFalcon() {

    lJoystick.rightTrigger().onTrue( // move right motor clockwise on right trigger
      new InstantCommand(() -> {
        rClimber.set(0.5);
      })
    );

    lJoystick.rightTrigger().onFalse( // stop when not in use
      new InstantCommand(() -> {
        rClimber.set(0);
      })
    );

    lJoystick.rightBumper().onTrue( // move right motor counter-clockwise on right bumper
      new InstantCommand(() -> {
        rClimber.set(-0.5);
      })
    );

    lJoystick.rightBumper().onFalse( // stop when not in use
      new InstantCommand(() -> {
        rClimber.set(0);
      })
    );

    lJoystick.leftTrigger().onTrue( // move left motor clockwise on right trigger
      new InstantCommand(() -> {
        lClimber.set(0.5);
      })
    );

    lJoystick.leftTrigger().onFalse( // stop motor when not in use
      new InstantCommand(() -> {
        lClimber.set(0);
      })
    );

    lJoystick.leftBumper().onTrue( // move left motor counter-clockwise on right bumper
      new InstantCommand(() -> {
        lClimber.set(-0.5);
      })
    );

    lJoystick.leftBumper().onFalse( // stop motor when not in use
      new InstantCommand(() -> {
        lClimber.set(0);
      })
    );

    }



}

*/