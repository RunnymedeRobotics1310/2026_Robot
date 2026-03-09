package frc.robot.commands.auto.config;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonParseException;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class AutoConfigParser {

  private static final int MAX_CONFIG_NAME_LENGTH = 40;
  private static final int MAX_TOP_LEVEL_STEPS = 30;
  private static final int MAX_TOTAL_STEPS = 80;
  private static final int MAX_PARALLEL_CHILDREN = 6;

  private static final Gson gson = new GsonBuilder().setPrettyPrinting().create();

  /**
   * Runtime-only configs pushed from dashboard via NetworkTables. These are intentionally not
   * persisted on the robot filesystem.
   */
  private static final Map<String, String> runtimeConfigs = new ConcurrentHashMap<>();

  /** Read-only pre-packaged configs deployed with the robot code. */
  private static File getDeployAutosDirectory() {
    return new File(Filesystem.getDeployDirectory(), "autos");
  }

  public static List<String> listAutoConfigs() {
    // Use a set to deduplicate — runtime configs override deploy configs with the same name.
    java.util.Set<String> names = new LinkedHashSet<>();
    addConfigsFromDirectory(names, getDeployAutosDirectory());
    names.addAll(runtimeConfigs.keySet());
    return new ArrayList<>(names);
  }

  public static List<String> listRuntimeAutoConfigs() {
    return new ArrayList<>(runtimeConfigs.keySet());
  }

  public static List<String> listDeployAutoConfigs() {
    java.util.Set<String> set = new LinkedHashSet<>();
    addConfigsFromDirectory(set, getDeployAutosDirectory());
    return new ArrayList<>(set);
  }

  private static void addConfigsFromDirectory(java.util.Set<String> names, File dir) {
    if (dir.exists() && dir.isDirectory()) {
      File[] files = dir.listFiles((d, name) -> name.endsWith(".json"));
      if (files != null) {
        for (File file : files) {
          names.add(file.getName().replace(".json", ""));
        }
      }
    }
  }

  /** Resolve a deploy config file by name. */
  private static File resolveConfigFile(String name) {
    File deploy = new File(getDeployAutosDirectory(), name + ".json");
    if (deploy.exists()) {
      return deploy;
    }
    return null;
  }

  public static AutoConfig loadAutoConfig(String name) {
    String safeName = sanitizeConfigName(name);
    if (safeName == null) {
      System.out.println("AutoConfigParser: Invalid config name: " + name);
      return null;
    }

    String runtimeJson = runtimeConfigs.get(safeName);
    if (runtimeJson != null) {
      AutoConfig runtimeConfig = parseJson(runtimeJson);
      if (runtimeConfig == null) {
        System.out.println("AutoConfigParser: Invalid runtime config " + safeName + ", removing");
        runtimeConfigs.remove(safeName);
        return null;
      }
      String validationError = validateAutoConfig(runtimeConfig);
      if (validationError != null) {
        System.out.println(
            "AutoConfigParser: Invalid runtime config " + safeName + ": " + validationError);
        runtimeConfigs.remove(safeName);
        return null;
      }
      return runtimeConfig;
    }

    File file = resolveConfigFile(safeName);
    if (file == null) {
      System.out.println("AutoConfigParser: Config file not found: " + name);
      return null;
    }
    try (FileReader reader = new FileReader(file)) {
      AutoConfig config = gson.fromJson(reader, AutoConfig.class);
      String validationError = validateAutoConfig(config);
      if (validationError != null) {
        System.out.println("AutoConfigParser: Invalid config " + name + ": " + validationError);
        return null;
      }
      return config;
    } catch (IOException | JsonParseException e) {
      System.out.println("AutoConfigParser: Error loading config " + name + ": " + e.getMessage());
      return null;
    }
  }

  public static AutoConfig parseJson(String json) {
    try {
      return gson.fromJson(json, AutoConfig.class);
    } catch (JsonParseException e) {
      System.out.println("AutoConfigParser: Error parsing JSON: " + e.getMessage());
      return null;
    }
  }

  public static String toJson(AutoConfig config) {
    return gson.toJson(config);
  }

  public static String saveAutoConfig(String name, String json) {
    AutoConfig config = parseJson(json);
    if (config == null) {
      return "Invalid JSON: could not parse";
    }

    String validationError = validateAutoConfig(config);
    if (validationError != null) {
      return "Invalid config: " + validationError;
    }

    String safeName = sanitizeConfigName(name);
    if (safeName == null) {
      return "Invalid config name";
    }

    // Runtime-only save. Source-of-truth persistence stays in deploy JSON in the repo.
    runtimeConfigs.put(safeName, json);
    return "ok:runtime";
  }

  public static boolean deleteAutoConfig(String name) {
    String safeName = sanitizeConfigName(name);
    if (safeName == null) {
      return false;
    }
    // Only delete from runtime configs — deploy configs are immutable.
    return runtimeConfigs.remove(safeName) != null;
  }

  public static boolean isRuntimeConfig(String name) {
    String safeName = sanitizeConfigName(name);
    return safeName != null && runtimeConfigs.containsKey(safeName);
  }

  public static boolean isDeployConfig(String name) {
    String safeName = sanitizeConfigName(name);
    if (safeName == null) {
      return false;
    }
    return resolveConfigFile(safeName) != null;
  }

  public static String readAutoConfigJson(String name) {
    String safeName = sanitizeConfigName(name);
    if (safeName == null) {
      return null;
    }

    String runtimeJson = runtimeConfigs.get(safeName);
    if (runtimeJson != null) {
      return runtimeJson;
    }

    File file = resolveConfigFile(safeName);
    if (file == null) {
      return null;
    }
    try (FileReader reader = new FileReader(file)) {
      StringBuilder sb = new StringBuilder();
      char[] buffer = new char[1024];
      int read;
      while ((read = reader.read(buffer)) != -1) {
        sb.append(buffer, 0, read);
      }
      return sb.toString();
    } catch (IOException e) {
      return null;
    }
  }

  public static String sanitizeConfigName(String rawName) {
    if (rawName == null) {
      return null;
    }
    String safeName =
        rawName
            .trim()
            .toLowerCase(Locale.ROOT)
            .replaceAll("[^a-z0-9_]", "_")
            .replaceAll("_+", "_")
            .replaceAll("^_+", "")
            .replaceAll("_+$", "");

    if (safeName.isEmpty() || safeName.length() > MAX_CONFIG_NAME_LENGTH) {
      return null;
    }
    return safeName;
  }

  public static String validateAutoConfig(AutoConfig config) {
    if (config == null) {
      return "config is null";
    }
    if (sanitizeConfigName(config.name) == null) {
      return "name must be 1-" + MAX_CONFIG_NAME_LENGTH + " chars [a-z0-9_]";
    }
    if (Double.isNaN(config.startingHeadingDegrees)
        || Double.isInfinite(config.startingHeadingDegrees)) {
      return "startingHeadingDegrees must be finite";
    }
    if (config.steps == null || config.steps.isEmpty()) {
      return "missing 'steps' field";
    }
    if (config.steps.size() > MAX_TOP_LEVEL_STEPS) {
      return "too many top-level steps (max " + MAX_TOP_LEVEL_STEPS + ")";
    }

    int[] totalStepCount = new int[] {0};
    for (int i = 0; i < config.steps.size(); i++) {
      String stepError =
          validateStep(config.steps.get(i), "steps[" + i + "]", true, totalStepCount);
      if (stepError != null) {
        return stepError;
      }
    }
    return null;
  }

  @SuppressWarnings("unchecked")
  private static String validateStep(
      Map<String, Object> step, String path, boolean allowParallel, int[] totalStepCount) {
    if (step == null) {
      return path + ": step is null";
    }
    totalStepCount[0]++;
    if (totalStepCount[0] > MAX_TOTAL_STEPS) {
      return "too many total steps (max " + MAX_TOTAL_STEPS + ")";
    }

    Object typeObj = step.get("type");
    if (typeObj == null) {
      return path + ": missing step type";
    }
    String type = String.valueOf(typeObj);

    // Validate structural types
    if ("parallel".equals(type)) {
      if (!allowParallel) {
        return path + ": nested parallel blocks are not allowed";
      }
      Object endConditionObj = step.get("endCondition");
      if (endConditionObj == null) {
        return path + ": parallel.endCondition is required";
      }
      Object commandsObj = step.get("commands");
      if (!(commandsObj instanceof List)) {
        return path + ": parallel.commands must be an array";
      }
      List<Map<String, Object>> commands = (List<Map<String, Object>>) commandsObj;
      if (commands.size() < 2) {
        return path + ": parallel.commands must contain at least 2 commands";
      }
      if (commands.size() > MAX_PARALLEL_CHILDREN) {
        return path + ": parallel.commands exceeds max " + MAX_PARALLEL_CHILDREN;
      }
      String endCondition = String.valueOf(endConditionObj);
      if ("deadline".equals(endCondition)) {
        double deadlineIndex = getDouble(step, "deadlineIndex");
        if (deadlineIndex < 0 || deadlineIndex >= commands.size()) {
          return path + ": parallel.deadlineIndex must be between 0 and " + (commands.size() - 1);
        }
      }
      for (int i = 0; i < commands.size(); i++) {
        String childError =
            validateStep(commands.get(i), path + ".commands[" + i + "]", false, totalStepCount);
        if (childError != null) {
          return childError;
        }
      }
      return null;
    }

    if ("delay".equals(type)) {
      double duration = getDouble(step, "durationSeconds");
      if (duration <= 0 || duration > 15) {
        return path + ": delay.durationSeconds must be > 0 and <= 15";
      }
      return null;
    }

    // All other types are validated at command creation time by the registry
    return null;
  }

  private static double getDouble(Map<String, Object> map, String key) {
    Object val = map.get(key);
    if (val instanceof Number n) {
      return n.doubleValue();
    }
    return 0;
  }
}
