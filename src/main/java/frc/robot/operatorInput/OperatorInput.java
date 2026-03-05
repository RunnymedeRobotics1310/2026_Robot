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
import frc.robot.commands.auto.ExitZoneAutoCommand;
import frc.robot.commands.auto.OpportunisticOutpostAutoCommand;
import frc.robot.commands.auto.ShootCenterAutoCommand;
import frc.robot.commands.auto.SimpleCenterAutoCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.TuneShooterCommand;
import frc.robot.commands.swerve.DriveToTowerCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;

public class OperatorInput extends SubsystemBase {

  private final GameController driverController =
      new GameController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final GameController operatorController =
      new GameController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  private final SwerveSubsystem swerve;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private final LimelightVisionSubsystem vision;

  private final SendableChooser<Constants.AutoConstants.AutoPattern> autoPatternChooser =
      new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.Delay> delayChooser =
      new SendableChooser<>();

  public OperatorInput(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      LimelightVisionSubsystem vision) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.intake = intake;
    this.vision = vision;
  }

  /** Use this method to define your trigger->command mappings. */
  public void configureButtonBindings(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      LimelightVisionSubsystem vision) {

    new Trigger(this::isZeroGyro).onTrue(new SetAllianceGyroCommand(swerve, 0));
    new Trigger(this::isCancel).whileTrue(new CancelCommand(this, swerve, shooter, intake));

    /* DRIVER CONTROLS */

    // Shoot from anywhere
    new Trigger(this::shootFromAnywhere).whileTrue(new ShooterCommand(shooter, swerve));

    // Auto align to climb
    new Trigger(driverController::getAButton)
        .onTrue(new DriveToTowerCommand(swerve, vision, false));

    // not included here:
    //   intake - left trigger
    //   drive
    //     both joysticks - move
    //     both bumpers - fast/slow
    //   tune shooter controls
    //     POV up/down - adjust shooter speed
    //     POV left - enable kicker
    //     POV right - enable hood adjust
    //     rightY - set hood angle

    /* OPERATOR CONTROLS */

    // Shoot from set range - ends when button is released, or after 100 seconds
    //    new Trigger(this::isCloseShoot)
    //            .whileTrue(new LazyShooterCommand(shooter, 3000, 0, 100));

    // not included here:
    //   manual climb
    //   reverse kicker
    //   reverse intake
    //   stop shooter

    new Trigger(driverController::getXButton)
        .onTrue(new TuneShooterCommand(shooter, this, swerve, intake));
  }

  public boolean isCancel() {
    return (driverController.getStartButton() && !driverController.getBackButton());
  }

  public boolean isZeroGyro() {
    return driverController.getBackButton();
  }

  public boolean getRotate180Val() {
    return false;
  }

  public boolean isFastMode() {
    return driverController.getRightBumperButton();
  }

  public boolean isSlowMode() {
    return driverController.getLeftBumperButton();
  }

  public boolean shootFromAnywhere() {
    return driverController.getRightTriggerAxis() > 0.5;
  }

  public boolean getFaceHub() {
    return shootFromAnywhere();
  }

  public boolean isIntakeDoingStuff() {
    return driverController.getLeftTriggerAxis() > 0.5;
  }

  public boolean isCloseShoot() {
    return operatorController.getRightTriggerAxis() > 0.5;
  }

  public boolean isStopFlywheel() {
    return operatorController.getYButton();
  }

  public boolean isReverseKicker() {
    return operatorController.getBButton();
  }

  public boolean isReverseIntake() {
    return operatorController.getAButton();
  }

  public double getDriverControllerAxis(Stick stick, Axis axis) {
    return switch (stick) {
      case LEFT ->
          switch (axis) {
            case X -> driverController.getLeftX();
            case Y -> driverController.getLeftY();
          };
      case RIGHT ->
          switch (axis) {
            case X -> driverController.getRightX();
            case Y -> driverController.getRightY();
          };
    };
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

  public void initAutoSelectors() {

    SmartDashboard.putData("1310/auto/Auto Selector", autoPatternChooser);

    autoPatternChooser.setDefaultOption(
        "Do Nothing", Constants.AutoConstants.AutoPattern.DO_NOTHING);
    autoPatternChooser.addOption("Exit Zone", Constants.AutoConstants.AutoPattern.EXIT_ZONE);
    autoPatternChooser.addOption(
        "Simple Center", Constants.AutoConstants.AutoPattern.SIMPLE_CENTER);
    autoPatternChooser.addOption(
        "Opportunistic Outpost", Constants.AutoConstants.AutoPattern.OPPORTUNISTIC_OUTPOST);
    autoPatternChooser.addOption("Shoot Center", Constants.AutoConstants.AutoPattern.SHOOT_CENTER);

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
      case SIMPLE_CENTER -> new SimpleCenterAutoCommand(swerve, shooter, vision);
      case OPPORTUNISTIC_OUTPOST -> new OpportunisticOutpostAutoCommand(swerve, shooter, vision);
      case SHOOT_CENTER -> new ShootCenterAutoCommand(swerve, intake, delay);

      default -> new InstantCommand();
    };
  }
}
