package frc.robot.operatorInput;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.auto.SimpleCenterAutoCommand;
import frc.robot.commands.auto.ExitZoneAutoCommand;
import frc.robot.commands.auto.OpportunisticOutpostAutoCommand;
import frc.robot.commands.swerve.FaceHubCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class OperatorInput extends SubsystemBase {

  private GameController driverController =
      new GameController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final SwerveSubsystem swerve;
  private final LimelightVisionSubsystem vision;

  private final SendableChooser<Constants.AutoConstants.AutoPattern> autoPatternChooser =
          new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.Delay> delayChooser =
          new SendableChooser<>();

  public OperatorInput(SwerveSubsystem swerve, LimelightVisionSubsystem vision) {
    this.swerve = swerve;
    this.vision = vision;
  }

  /** Use this method to define your trigger->command mappings. */
  public void configureButtonBindings(
      ExampleSubsystem exampleSubsystem) {
    // Schedule `ExampleCommand` when `A' button is pressed.
    new Trigger(() -> driverController.getBButtonPressed())
        .onTrue(new FaceHubCommand(swerve));

    new Trigger(this::isCancel).whileTrue(new CancelCommand(this, swerve));
    new Trigger(this::isZeroGyro).onTrue(new SetAllianceGyroCommand(swerve, 0));
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

  public boolean getFaceHub() {
    return driverController.getYButton();
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

  public void initAutoSelectors() {

    SmartDashboard.putData("1310/auto/Auto Selector", autoPatternChooser);

    autoPatternChooser.setDefaultOption(
            "Do Nothing", Constants.AutoConstants.AutoPattern.DO_NOTHING);
    autoPatternChooser.addOption("Exit Zone", Constants.AutoConstants.AutoPattern.EXIT_ZONE);
    autoPatternChooser.addOption("Simple Center", Constants.AutoConstants.AutoPattern.SIMPLE_CENTER);
    autoPatternChooser.addOption("Opportunistic Outpost", Constants.AutoConstants.AutoPattern.OPPORTUNISTIC_OUTPOST);

    SmartDashboard.putData("1310/auto/Delay Selector", delayChooser);

    delayChooser.setDefaultOption("No Delay", Constants.AutoConstants.Delay.NO_DELAY);
    delayChooser.addOption("1/2 Seconds", Constants.AutoConstants.Delay.WAIT_0_5_SECOND);
    delayChooser.addOption("1 Second", Constants.AutoConstants.Delay.WAIT_1_SECOND);
    delayChooser.addOption("1 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_1_5_SECONDS);
    delayChooser.addOption("2 Seconds", Constants.AutoConstants.Delay.WAIT_2_SECONDS);
    delayChooser.addOption("2 1/2 Seconds", Constants.AutoConstants.Delay.WAIT_2_5_SECONDS);
    delayChooser.addOption("3 Seconds", Constants.AutoConstants.Delay.WAIT_3_SECONDS);
    delayChooser.addOption("5 Seconds", Constants.AutoConstants.Delay.WAIT_5_SECONDS);
  }

  public Command getAutonomousCommand() {
    double delay =
            switch (delayChooser.getSelected()) {
              case WAIT_0_5_SECOND -> 0.5;
              case WAIT_1_SECOND -> 1;
              case WAIT_1_5_SECONDS -> 1.5;
              case WAIT_2_SECONDS -> 2;
              case WAIT_2_5_SECONDS -> 2.5;
              case WAIT_3_SECONDS -> 3;
              case WAIT_5_SECONDS -> 5;
              default -> 0;
            };

    return switch (autoPatternChooser.getSelected()) {
      case EXIT_ZONE -> new ExitZoneAutoCommand(swerve, delay);
      case SIMPLE_CENTER -> new SimpleCenterAutoCommand(swerve, vision);
      case OPPORTUNISTIC_OUTPOST -> new OpportunisticOutpostAutoCommand(swerve, vision);

      default -> new InstantCommand();
    };
  }
}
