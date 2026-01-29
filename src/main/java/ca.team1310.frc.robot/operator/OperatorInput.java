package ca.team1310.frc.robot.operator;

import ca.team1310.frc.robot.Constants;
import ca.team1310.frc.robot.commands.ExampleCommand;
import ca.team1310.frc.robot.subsystems.ExampleSubsystem;
import ca.team1310.frc.robot.subsystems.LightingSubsystem;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** The DriverController exposes all driver functions */
public class OperatorInput extends SubsystemBase {

  private final XboxController driverController;
  private final ExampleSubsystem exampleSubsystem;
  private final LightingSubsystem lightingSubsystem;

  private Command autonomousCommand = new InstantCommand();

  private final SendableChooser<Constants.AutoConstants.AutoPattern> autoPatternChooser =
      new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.Delay> delayChooser =
      new SendableChooser<>();

  /**
   * Construct an OperatorInput class that is fed by a DriverController and an OperatorController.
   *
   * @param driverControllerPort on the driver station which the driver joystick is plugged into
   */
  public OperatorInput(
      int driverControllerPort,
      double deadband,
      ExampleSubsystem exampleSubsystem,
      LightingSubsystem lightingSubsystem) {
    driverController = new GameController(driverControllerPort, deadband);
    this.exampleSubsystem = exampleSubsystem;
    this.lightingSubsystem = lightingSubsystem;
  }

  /**
   * Configure the button bindings for all operator commands
   *
   * <p>NOTE: This routine requires all subsystems to be passed in
   *
   * <p>NOTE: This routine must only be called once from the RobotContainer
   */
  public void configureButtonBindings(ExampleSubsystem exampleSubsystem) {

    // Example button binding for example command
    new Trigger(() -> driverController.getYButton())
        .onTrue(new ExampleCommand(exampleSubsystem, lightingSubsystem));
  }

  /*
   * Cancel Command support
   * Do not end the command while the button is pressed
   */
  public boolean isCancel() {
    return (driverController.getStartButton() && !driverController.getBackButton());
  }

  /*
   * Default Drive Command Buttons
   */
  public XboxController getRawDriverController() {
    return driverController;
  }

  /*
   * The following routines are used by the default commands for each subsystem
   *
   * They allow the default commands to get user input to manually move the
   * robot elements.
   */

  public double getDriverControllerAxis(Stick stick, Axis axis) {
    switch (stick) {
      case LEFT:
        switch (axis) {
          case X:
            return driverController.getLeftX();
          case Y:
            return driverController.getLeftY();
        }
        break;
      case RIGHT:
        switch (axis) {
          case X:
            return driverController.getRightX();
        }
        break;
    }

    return 0;
  }

  @Override
  public void periodic() {
    if (Constants.TelemetryConfig.oi) {
      SmartDashboard.putString("Driver Controller", driverController.toString());
    }
  }

  public enum Stick {
    LEFT,
    RIGHT
  }

  public enum Axis {
    X,
    Y
  }
}
