package frc.robot.commands.auto.config;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.SetAllianceGyroCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class AutoCommandFactory {

  private final AutoCommandRegistry registry;
  private final AutoCommandRegistry.SubsystemRegistry subsystems;
  private final SwerveSubsystem swerve;

  public AutoCommandFactory(
      AutoCommandRegistry registry,
      AutoCommandRegistry.SubsystemRegistry subsystems,
      SwerveSubsystem swerve) {
    this.registry = registry;
    this.subsystems = subsystems;
    this.swerve = swerve;
  }

  public Command buildAutoCommand(AutoConfig config, double delay) {
    List<Command> commands = new ArrayList<>();

    if (config == null || config.steps == null) {
      System.out.println("AutoCommandFactory: Invalid config");
      return new WaitCommand(0);
    }

    if (delay > 0) {
      commands.add(new WaitCommand(delay));
    }

    commands.add(new SetAllianceGyroCommand(swerve, config.startingHeadingDegrees));

    for (Map<String, Object> step : config.steps) {
      if (step == null) {
        continue;
      }
      Command cmd = buildStep(step);
      if (cmd != null) {
        commands.add(cmd);
      }
    }

    return new SequentialCommandGroup(commands.toArray(new Command[0]));
  }

  @SuppressWarnings("unchecked")
  private Command buildStep(Map<String, Object> step) {
    String rawType = getString(step, "type");
    if (rawType == null) {
      System.out.println("AutoCommandFactory: Step missing 'type'");
      return null;
    }

    // Map legacy format to new type names
    String type = mapLegacyType(rawType, step);

    // Handle structural types directly
    if ("parallel".equals(type)) {
      return buildParallelGroup(step);
    }
    if ("delay".equals(type)) {
      double duration = getDouble(step, "durationSeconds");
      return new WaitCommand(duration);
    }
    if ("sequential".equals(type)) {
      return buildSequentialGroup(step);
    }

    // Delegate to registry for all registered command types
    if (registry.hasType(type)) {
      Command cmd = registry.createCommand(type, step, subsystems);
      if (cmd != null) {
        // Apply timeout wrapping for commands that don't have internal timeout params
        cmd = applyTimeoutIfNeeded(cmd, type, step);
      }
      return cmd;
    }

    System.out.println("AutoCommandFactory: Unknown step type: " + type);
    return null;
  }

  /** Map legacy JSON format type names to new registry type names. */
  private String mapLegacyType(String rawType, Map<String, Object> step) {
    if ("drive".equals(rawType)) {
      String mode = getString(step, "mode");
      if ("distance".equals(mode)) {
        return "drive_distance";
      } else if ("time".equals(mode)) {
        return "drive_timed";
      } else if ("velocity".equals(mode)) {
        return "drive_velocity";
      } else if ("to_pose".equals(mode)) {
        return "drive_to_pose";
      }
    }

    if ("face_target".equals(rawType)) {
      String target = getString(step, "target");
      if ("hub".equals(target)) {
        return "face_hub";
      } else if ("point".equals(target)) {
        return "face_field_point";
      }
    }

    // Handle intake action field name mapping: JSON uses "action" but the
    // ConfigParam is named "intakeAction"
    if ("intake".equals(rawType)) {
      if (step.containsKey("action") && !step.containsKey("intakeAction")) {
        step.put("intakeAction", step.get("action"));
      }
    }

    return rawType;
  }

  /**
   * Apply .withTimeout() for command types that don't have internal timeout handling but have a
   * timeoutSeconds field in the step config.
   */
  private Command applyTimeoutIfNeeded(Command cmd, String type, Map<String, Object> step) {
    // These types handle their own timeout or don't need one
    if ("drive_velocity".equals(type)
        || "face_hub".equals(type)
        || "vision_approach_tag".equals(type)
        || "drive_field_oriented".equals(type)
        || "shooter".equals(type)
        || "null_drive".equals(type)) {
      double timeout = getDouble(step, "timeoutSeconds");
      if (timeout > 0) {
        return cmd.withTimeout(timeout);
      }
    }
    return cmd;
  }

  @SuppressWarnings("unchecked")
  private Command buildSequentialGroup(Map<String, Object> step) {
    List<Map<String, Object>> childSteps = (List<Map<String, Object>>) step.get("commands");
    if (childSteps == null || childSteps.isEmpty()) {
      System.out.println("AutoCommandFactory: Sequential group has no commands");
      return new WaitCommand(0);
    }
    List<Command> children = new ArrayList<>();
    for (Map<String, Object> child : childSteps) {
      Command cmd = buildStep(child);
      if (cmd != null) {
        children.add(cmd);
      }
    }
    if (children.isEmpty()) {
      return new WaitCommand(0);
    }
    return new SequentialCommandGroup(children.toArray(new Command[0]));
  }

  @SuppressWarnings("unchecked")
  private Command buildParallelGroup(Map<String, Object> step) {
    List<Map<String, Object>> childSteps = (List<Map<String, Object>>) step.get("commands");
    if (childSteps == null || childSteps.isEmpty()) {
      System.out.println("AutoCommandFactory: Parallel group has no commands");
      return new WaitCommand(0);
    }

    List<Command> children = new ArrayList<>();
    for (Map<String, Object> child : childSteps) {
      Command cmd = buildStep(child);
      if (cmd != null) {
        children.add(cmd);
      }
    }

    if (children.isEmpty()) {
      return new WaitCommand(0);
    }

    String endCondition = getString(step, "endCondition");
    Command[] cmds = children.toArray(new Command[0]);

    if ("first".equals(endCondition)) {
      return new ParallelRaceGroup(cmds);
    } else if ("deadline".equals(endCondition)) {
      int deadlineIndex =
          Math.max(0, Math.min((int) getDouble(step, "deadlineIndex"), children.size() - 1));
      Command deadline = children.remove(deadlineIndex);
      return new ParallelDeadlineGroup(deadline, children.toArray(new Command[0]));
    } else {
      return new ParallelCommandGroup(cmds);
    }
  }

  private static String getString(Map<String, Object> map, String key) {
    Object val = map.get(key);
    return val != null ? String.valueOf(val) : null;
  }

  private static double getDouble(Map<String, Object> map, String key) {
    Object val = map.get(key);
    if (val instanceof Number n) {
      return n.doubleValue();
    }
    if (val instanceof String s) {
      try {
        return Double.parseDouble(s);
      } catch (NumberFormatException e) {
        return 0;
      }
    }
    return 0;
  }
}
