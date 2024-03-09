/* 
package frc.robot.Arms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Arms.ArmConstants;
import frc.robot.Arms.ArmConstants.ArmOperatorConstants;

public class ArmContainer {

   

    private final Arm arms = new Arm();

    
    private final CommandXboxController m_armController =
      new CommandXboxController(ArmOperatorConstants.ArmControllerPort);

}

public ArmContainer() {
    configureBindings();
}

private void configureBindings() {
    m_armController
    .()
    .whileTrue(
        new Climb(arms));
}
*/
