package frc.robot.operatorInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.swerve.SetGyroCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class OperatorInput extends SubsystemBase {

  private GameController driverController =
      new GameController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** Use this method to define your trigger->command mappings. */
  public void configureButtonBindings(

          SwerveSubsystem swerve, LightingSubsystem lightingSubsystem, ExampleSubsystem exampleSubsystem, ShooterSubsystem shooterSubsystem, LimelightVisionSubsystem vision) {
    // Schedule `ExampleCommand` when `A' button is pressed.
    new Trigger(() -> isZeroGyro())
            .onTrue(new SetGyroCommand(swerve, 0));
    new Trigger(() -> driverController.getAButtonPressed())
        .onTrue(new ExampleCommand(exampleSubsystem, lightingSubsystem));
    new Trigger(() -> driverController.getBButtonPressed())
        .onTrue(new ShooterCommand(shooterSubsystem, vision, lightingSubsystem, this, swerve));

    new Trigger(this::isCancel).whileTrue(new CancelCommand(this, swerve, shooterSubsystem));
  }

  public boolean isCancel() {
    return (driverController.getStartButton() && !driverController.getBackButton());
  }
  public boolean isZeroGyro() {
    return driverController.getBackButton();
  }

  public boolean getRotate180Val() {
    return driverController.getAButton();
  }

  public boolean isFastMode() {
    return driverController.getRightBumperButton();
  }
  public boolean isSlowMode() {
    return driverController.getLeftBumperButton();
  }

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

  public enum Stick {
    LEFT,
    RIGHT
  }

  public enum Axis {
    X,
    Y
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Driver Gamecontroller", driverController.toString());
  }

  public GameController getDriverController() {
    return driverController;
  }
}
