package frc.robot.operatorInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LightingSubsystem;

public class OperatorInput extends SubsystemBase {

  private GameController driverController = new GameController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /**
   * Use this method to define your trigger->command mappings.
   */
  public void configureButtonBindings(LightingSubsystem lightingSubsystem, ExampleSubsystem exampleSubsystem) {
    // Schedule `ExampleCommand` when `A' button is pressed.
    new Trigger(() -> driverController.getAButtonPressed())
        .onTrue(new ExampleCommand(exampleSubsystem));

  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Driver Gamecontroller", driverController.toString());
  }
}
