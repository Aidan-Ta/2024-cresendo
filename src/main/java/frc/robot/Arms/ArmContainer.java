/* 
package frc.robot.Arms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Shooter.ShooterConstants.LauncherConstants;
import frc.robot.Shooter.ShooterConstants.OperatorConstants;
import frc.robot.Shooter.Autos;

public class ArmContainer {

   

    private final Arms arms = new Arms();

    
    private final CommandXboxController m_armController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

}

public ArmContainer() {
    configureBindings();
}

private void configureBindings() {
    m_armController
    .b()
    .whileTrue(
        new Climb(arms));
}
*/