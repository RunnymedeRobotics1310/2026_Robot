package frc.robot.operatorInput;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.auto.*;
import frc.robot.commands.auto.ExitZoneAutoCommand;
import frc.robot.commands.auto.OpportunisticOutpostAutoCommand;
import frc.robot.commands.auto.SimpleCenterAutoCommand;
import frc.robot.commands.auto.config.AutoCommandFactory;
import frc.robot.commands.auto.config.AutoConfig;
import frc.robot.commands.auto.config.AutoConfigParser;
import frc.robot.commands.hopper.LazyShooterCommand;
import frc.robot.commands.swerve.DriveToTowerCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightVisionSubsystem;
import java.util.List;

public class OperatorInput extends SubsystemBase {

  private final GameController driverController =
      new GameController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final GameController operatorController =
      new GameController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  private final SwerveSubsystem swerve;
  private final HopperSubsystem hopper;
  private final LimelightVisionSubsystem vision;
  private final ClimbSubsystem climb;

  private final SendableChooser<Constants.AutoConstants.AutoPattern> autoPatternChooser =
      new SendableChooser<>();
  private final SendableChooser<Constants.AutoConstants.Delay> delayChooser =
      new SendableChooser<>();
  private SendableChooser<String> customAutoChooser = new SendableChooser<>();

  private final AutoCommandFactory autoCommandFactory;

  public OperatorInput(
      SwerveSubsystem swerve,
      HopperSubsystem hopper,
      LimelightVisionSubsystem vision,
      ClimbSubsystem climb,
      AutoCommandFactory autoCommandFactory) {
    this.swerve = swerve;
    this.hopper = hopper;
    this.vision = vision;
    this.climb = climb;
    this.autoCommandFactory = autoCommandFactory;
  }

  /** Use this method to define your trigger->command mappings. */
  public void configureButtonBindings(
      SwerveSubsystem swerve, HopperSubsystem hopper, LimelightVisionSubsystem vision) {

    new Trigger(this::isZeroGyro)
        .onTrue(
            //            new SequentialCommandGroup(
            //                new SetAllianceGyroCommand(swerve, 0),
            //                new WaitCommand(0.1),
            //                new InstantCommand(
            //                    () -> {
            //                      if (vision.isVisionPoseValid()) {
            //                        swerve.resetOdometry(vision.getVisionPose());
            //                      }
            //                    }))
            new SetAllianceGyroCommand(swerve, 0));

    new Trigger(this::isCancel).whileTrue(new CancelCommand(this, swerve, hopper, climb));

    /* DRIVER CONTROLS */

    // Auto align to climb
    new Trigger(driverController::getXButton)
        .onTrue(new DriveToTowerCommand(swerve, vision, climb, false));

    new Trigger(driverController::getBButton)
        .onTrue(new DriveToTowerCommand(swerve, vision, climb, true));

    // not included here:
    //   shoot - right trigger
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
    new Trigger(this::isCloseShoot).whileTrue(new LazyShooterCommand(hopper, 3900, 0, 100));

    new Trigger(this::putClimbUp).onTrue(new AutoClimbCommand(climb, hopper, true));
    new Trigger(this::putClimbDown).onTrue(new AutoClimbCommand(climb, hopper, false));

    // not included here:
    //   manual climb
    //   reverse kicker
    //   reverse intake
    //   stop shooter

    //    new Trigger(driverController::getXButton).onTrue(new TuneShooterCommand(hopper, this,
    // swerve));
  }

  public boolean isCancel() {
    return ((driverController.getStartButton() && !driverController.getBackButton()
        || operatorController.getStartButton()));
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

  // ----- OPERATOR CONTROLLS -----
  public boolean isShift() {
    return operatorController.getLeftBumperButton();
  }

  public boolean isReverseKicker() {
    return !isShift() && operatorController.getRightBumperButton();
  }

  public boolean isRunKicker() {
    return isShift() && operatorController.getRightBumperButton();
  }

  public boolean isRunAgitator() {
    return !isShift() && operatorController.getLeftTriggerAxis() > 0.5;
  }

  public boolean isStopAgitator() {
    return isShift() && operatorController.getLeftTriggerAxis() > 0.5;
  }

  public boolean isCloseShoot() {
    return operatorController.getRightTriggerAxis() > 0.5;
  }

  public boolean isStopFlywheel() {
    return !isShift() && operatorController.getYButton();
  }

  public boolean isReverseFlywheel() {
    return isShift() && operatorController.getYButton();
  }

  public boolean isIntakeForwards() {
    return !isShift() && operatorController.getBButton();
  }

  public boolean isIntakeReverse() {
    return isShift() && operatorController.getBButton();
  }

  public boolean putHoodDown() {
    return operatorController.getAButton();
  }

  public boolean isClimbSolenoid() {
    return isShift() && operatorController.getXButton();
  }

  public boolean putClimbUp() {
    return operatorController.getPOV() == 0 || driverController.getPOV() == 0;
  }

  public boolean putClimbDown() {
    return operatorController.getPOV() == 180 || driverController.getPOV() == 180;
  }

  public boolean isOpenDoor() {
    return isShift() && operatorController.getPOV() == 270;
  }

  public boolean isCloseDoor() {
    return !isShift() && operatorController.getPOV() == 270;
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

  public GameController getOperatorController() {
    return operatorController;
  }

  public void initAutoSelectors() {

    SmartDashboard.putData("1310/auto/Auto Selector", autoPatternChooser);

    autoPatternChooser.setDefaultOption(
        "Do Nothing", Constants.AutoConstants.AutoPattern.DO_NOTHING);
    //    autoPatternChooser.addOption("Exit Zone", Constants.AutoConstants.AutoPattern.EXIT_ZONE);
    autoPatternChooser.addOption(
        "Simple Center", Constants.AutoConstants.AutoPattern.SIMPLE_CENTER);
    autoPatternChooser.addOption(
        "Opportunistic Outpost", Constants.AutoConstants.AutoPattern.OPPORTUNISTIC_OUTPOST);
    //    autoPatternChooser.addOption("Custom Auto", Constants.AutoConstants.AutoPattern.CUSTOM);
    autoPatternChooser.addOption(
        "Left Shoot Climb", Constants.AutoConstants.AutoPattern.LEFT_SHOOT_CLIMB);
    autoPatternChooser.addOption(
        "Right Shoot Climb", Constants.AutoConstants.AutoPattern.RIGHT_SHOOT_CLIMB);
    autoPatternChooser.addOption("Depot", Constants.AutoConstants.AutoPattern.DEPOT);
    autoPatternChooser.addOption(
        "Down To Earth", Constants.AutoConstants.AutoPattern.DOWN_TO_EARTH);

    SmartDashboard.putData("1310/auto/Custom Auto Selector", customAutoChooser);
    refreshCustomAutoChooser();
    //    autoPatternChooser.addOption("Shoot Center",
    // Constants.AutoConstants.AutoPattern.SHOOT_CENTER);

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
      case SIMPLE_CENTER -> new SimpleCenterAutoCommand(swerve, hopper, vision, climb, delay);
      case OPPORTUNISTIC_OUTPOST ->
          new OpportunisticOutpostAutoCommand(swerve, hopper, vision, climb, delay);
      case SHOOT_CENTER -> new ShootCenterAutoCommand(swerve, hopper, delay);
      case LEFT_SHOOT_CLIMB -> new LeftShootClimbAutoCommand(swerve, hopper, vision, climb, delay);
      case RIGHT_SHOOT_CLIMB ->
          new RightShootClimbAutoCommand(swerve, hopper, vision, climb, delay);
      case DEPOT -> new DepotAutoCommand(swerve, hopper, vision, climb, delay);
      case DOWN_TO_EARTH -> new DownToEarthAutoCommand(swerve, hopper, vision, climb, delay);
      case CUSTOM -> buildCustomAutoCommand(delay);
      default -> new InstantCommand();
    };
  }

  private Command buildCustomAutoCommand(double delay) {
    String selectedConfig = customAutoChooser.getSelected();
    if (selectedConfig == null || selectedConfig.isEmpty()) {
      System.out.println("OperatorInput: No custom auto selected");
      return new InstantCommand();
    }

    AutoConfig config = AutoConfigParser.loadAutoConfig(selectedConfig);
    if (config == null) {
      System.out.println("OperatorInput: Failed to load custom auto: " + selectedConfig);
      return new InstantCommand();
    }

    System.out.println("OperatorInput: Building custom auto: " + config.name);
    return autoCommandFactory.buildAutoCommand(config, delay);
  }

  public void refreshCustomAutoChooser() {
    List<String> configs = AutoConfigParser.listAutoConfigs();

    // SendableChooser has no removeOption/clear, so rebuild from scratch
    customAutoChooser.close();
    customAutoChooser = new SendableChooser<>();

    boolean first = true;
    for (String name : configs) {
      if (first) {
        customAutoChooser.setDefaultOption(name, name);
        first = false;
      } else {
        customAutoChooser.addOption(name, name);
      }
    }

    SmartDashboard.putData("1310/auto/Custom Auto Selector", customAutoChooser);
    System.out.println(
        "OperatorInput: Custom auto chooser refreshed with "
            + configs.size()
            + " configs: "
            + configs);
  }
}
