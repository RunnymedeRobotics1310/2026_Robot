package frc.robot.commands.auto.config;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonDeserializationContext;
import com.google.gson.JsonDeserializer;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Type;
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
    private static final double MAX_DRIVE_SPEED_MPS = 5.36;
    private static final double MAX_DRIVE_DISTANCE_METRES = 20.0;
    private static final double MAX_STEP_DURATION_SECONDS = 15.0;
    private static final double MAX_TIMEOUT_SECONDS = 15.0;
    private static final double MAX_FIELD_COORD_METRES = 20.0;
    private static final double MAX_HEADING_TOLERANCE_DEGREES = 20.0;
    private static final double MAX_POSITION_TOLERANCE_METRES = 2.0;

    private static final Gson gson = new GsonBuilder()
            .registerTypeAdapter(AutoStep.class, new AutoStepDeserializer())
            .setPrettyPrinting()
            .create();

    /**
     * Runtime-only configs pushed from dashboard via NetworkTables.
     * These are intentionally not persisted on the robot filesystem.
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

    /**
     * Resolve a deploy config file by name.
     */
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
                System.out.println("AutoConfigParser: Invalid runtime config " + safeName + ": " + validationError);
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
        String safeName = rawName.trim()
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
        if (Double.isNaN(config.startingHeadingDegrees) || Double.isInfinite(config.startingHeadingDegrees)) {
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
            String stepError = validateStep(config.steps.get(i), "steps[" + i + "]", true, totalStepCount);
            if (stepError != null) {
                return stepError;
            }
        }
        return null;
    }

    private static String validateStep(
            AutoStep step,
            String path,
            boolean allowParallel,
            int[] totalStepCount) {
        if (step == null) {
            return path + ": step is null";
        }
        totalStepCount[0]++;
        if (totalStepCount[0] > MAX_TOTAL_STEPS) {
            return "too many total steps (max " + MAX_TOTAL_STEPS + ")";
        }
        if (step.type == null) {
            return path + ": missing step type";
        }

        switch (step.type) {
            case drive:
                if (step.mode == null) {
                    return path + ": drive.mode is required";
                }
                if (step.speedMPS <= 0 || step.speedMPS > MAX_DRIVE_SPEED_MPS) {
                    return path + ": drive.speedMPS must be > 0 and <= " + MAX_DRIVE_SPEED_MPS;
                }
                if (step.mode == AutoStep.DriveMode.distance) {
                    if (!isFinite(step.direction)) {
                        return path + ": drive.direction must be finite";
                    }
                    if (!isFinite(step.headingDegrees)) {
                        return path + ": drive.headingDegrees must be finite";
                    }
                    if (step.distanceMetres <= 0 || step.distanceMetres > MAX_DRIVE_DISTANCE_METRES) {
                        return path + ": drive.distanceMetres must be > 0 and <= " + MAX_DRIVE_DISTANCE_METRES;
                    }
                    if (step.timeoutSeconds <= 0 || step.timeoutSeconds > MAX_TIMEOUT_SECONDS) {
                        return path + ": drive.timeoutSeconds must be > 0 and <= " + MAX_TIMEOUT_SECONDS;
                    }
                } else if (step.mode == AutoStep.DriveMode.time) {
                    if (!isFinite(step.direction)) {
                        return path + ": drive.direction must be finite";
                    }
                    if (!isFinite(step.headingDegrees)) {
                        return path + ": drive.headingDegrees must be finite";
                    }
                    if (step.durationSeconds <= 0 || step.durationSeconds > MAX_STEP_DURATION_SECONDS) {
                        return path + ": drive.durationSeconds must be > 0 and <= " + MAX_STEP_DURATION_SECONDS;
                    }
                } else {
                    if (!isFinite(step.xMetres) || Math.abs(step.xMetres) > MAX_FIELD_COORD_METRES) {
                        return path + ": drive.xMetres must be finite and <= " + MAX_FIELD_COORD_METRES + " magnitude";
                    }
                    if (!isFinite(step.yMetres) || Math.abs(step.yMetres) > MAX_FIELD_COORD_METRES) {
                        return path + ": drive.yMetres must be finite and <= " + MAX_FIELD_COORD_METRES + " magnitude";
                    }
                    if (!isFinite(step.headingDegrees)) {
                        return path + ": drive.headingDegrees must be finite";
                    }
                    if (step.positionToleranceMetres <= 0 || step.positionToleranceMetres > MAX_POSITION_TOLERANCE_METRES) {
                        return path + ": drive.positionToleranceMetres must be > 0 and <= "
                                + MAX_POSITION_TOLERANCE_METRES;
                    }
                    if (step.headingToleranceDegrees <= 0 || step.headingToleranceDegrees > MAX_HEADING_TOLERANCE_DEGREES) {
                        return path + ": drive.headingToleranceDegrees must be > 0 and <= "
                                + MAX_HEADING_TOLERANCE_DEGREES;
                    }
                    if (step.timeoutSeconds <= 0 || step.timeoutSeconds > MAX_TIMEOUT_SECONDS) {
                        return path + ": drive.timeoutSeconds must be > 0 and <= " + MAX_TIMEOUT_SECONDS;
                    }
                }
                return null;

            case rotate:
                if (!isFinite(step.headingDegrees)) {
                    return path + ": rotate.headingDegrees must be finite";
                }
                if (step.timeoutSeconds <= 0 || step.timeoutSeconds > MAX_TIMEOUT_SECONDS) {
                    return path + ": rotate.timeoutSeconds must be > 0 and <= " + MAX_TIMEOUT_SECONDS;
                }
                return null;

            case shooter:
                if (step.action == null) {
                    return path + ": shooter.action is required";
                }
                if (step.action == AutoStep.ShooterAction.off) {
                    return null;
                }
                if (step.rpm <= 0 || step.rpm > 6200) {
                    return path + ": shooter.rpm must be > 0 and <= 6200";
                }
                if (step.hoodPosition < 0 || step.hoodPosition > 1) {
                    return path + ": shooter.hoodPosition must be between 0 and 1";
                }
                if (step.kickerSpeed < -1 || step.kickerSpeed > 1) {
                    return path + ": shooter.kickerSpeed must be between -1 and 1";
                }
                if (step.kickerDelaySeconds < 0 || step.kickerDelaySeconds > MAX_TIMEOUT_SECONDS) {
                    return path + ": shooter.kickerDelaySeconds must be between 0 and " + MAX_TIMEOUT_SECONDS;
                }
                if (step.action == AutoStep.ShooterAction.on_with_duration
                        && (step.durationSeconds <= 0 || step.durationSeconds > MAX_STEP_DURATION_SECONDS)) {
                    return path + ": shooter.durationSeconds must be > 0 and <= " + MAX_STEP_DURATION_SECONDS;
                }
                return null;

            case intake:
                if (step.intakeAction == null) {
                    return path + ": intake.action is required";
                }
                if (step.intakeAction == AutoStep.IntakeAction.off) {
                    return null;
                }
                if (step.speed < -1 || step.speed > 1 || step.speed == 0) {
                    return path + ": intake.speed must be between -1 and 1 and not 0";
                }
                if (step.intakeAction == AutoStep.IntakeAction.on_with_duration
                        && (step.durationSeconds <= 0 || step.durationSeconds > MAX_STEP_DURATION_SECONDS)) {
                    return path + ": intake.durationSeconds must be > 0 and <= " + MAX_STEP_DURATION_SECONDS;
                }
                return null;

            case delay:
                if (step.durationSeconds <= 0 || step.durationSeconds > MAX_STEP_DURATION_SECONDS) {
                    return path + ": delay.durationSeconds must be > 0 and <= " + MAX_STEP_DURATION_SECONDS;
                }
                return null;

            case parallel:
                if (!allowParallel) {
                    return path + ": nested parallel blocks are not allowed";
                }
                if (step.endCondition == null) {
                    return path + ": parallel.endCondition is required";
                }
                if (step.commands == null || step.commands.size() < 2) {
                    return path + ": parallel.commands must contain at least 2 commands";
                }
                if (step.commands.size() > MAX_PARALLEL_CHILDREN) {
                    return path + ": parallel.commands exceeds max " + MAX_PARALLEL_CHILDREN;
                }
                if (step.endCondition == AutoStep.ParallelEndCondition.deadline
                        && (step.deadlineIndex < 0 || step.deadlineIndex >= step.commands.size())) {
                    return path + ": parallel.deadlineIndex must be between 0 and " + (step.commands.size() - 1);
                }
                for (int i = 0; i < step.commands.size(); i++) {
                    String childError = validateStep(
                            step.commands.get(i), path + ".commands[" + i + "]", false, totalStepCount);
                    if (childError != null) {
                        return childError;
                    }
                }
                return null;

            case drive_velocity:
                if (step.frame == null) {
                    return path + ": drive_velocity.frame is required";
                }
                if (!isFinite(step.vxMPS) || Math.abs(step.vxMPS) > MAX_DRIVE_SPEED_MPS) {
                    return path + ": drive_velocity.vxMPS must be finite and <= " + MAX_DRIVE_SPEED_MPS + " magnitude";
                }
                if (!isFinite(step.vyMPS) || Math.abs(step.vyMPS) > MAX_DRIVE_SPEED_MPS) {
                    return path + ": drive_velocity.vyMPS must be finite and <= " + MAX_DRIVE_SPEED_MPS + " magnitude";
                }
                if (!isFinite(step.headingDegrees)) {
                    return path + ": drive_velocity.headingDegrees must be finite";
                }
                if (step.durationSeconds <= 0 || step.durationSeconds > MAX_STEP_DURATION_SECONDS) {
                    return path + ": drive_velocity.durationSeconds must be > 0 and <= " + MAX_STEP_DURATION_SECONDS;
                }
                return null;

            case face_target:
                if (step.target == null) {
                    return path + ": face_target.target is required";
                }
                if (step.target == AutoStep.FaceTargetType.point) {
                    if (!isFinite(step.targetXMetres) || Math.abs(step.targetXMetres) > MAX_FIELD_COORD_METRES) {
                        return path + ": face_target.targetXMetres must be finite and <= " + MAX_FIELD_COORD_METRES
                                + " magnitude";
                    }
                    if (!isFinite(step.targetYMetres) || Math.abs(step.targetYMetres) > MAX_FIELD_COORD_METRES) {
                        return path + ": face_target.targetYMetres must be finite and <= " + MAX_FIELD_COORD_METRES
                                + " magnitude";
                    }
                }
                if (step.headingToleranceDegrees <= 0 || step.headingToleranceDegrees > MAX_HEADING_TOLERANCE_DEGREES) {
                    return path + ": face_target.headingToleranceDegrees must be > 0 and <= "
                            + MAX_HEADING_TOLERANCE_DEGREES;
                }
                if (step.timeoutSeconds <= 0 || step.timeoutSeconds > MAX_TIMEOUT_SECONDS) {
                    return path + ": face_target.timeoutSeconds must be > 0 and <= " + MAX_TIMEOUT_SECONDS;
                }
                return null;

            case vision_approach_tag:
                if (step.timeoutSeconds <= 0 || step.timeoutSeconds > MAX_TIMEOUT_SECONDS) {
                    return path + ": vision_approach_tag.timeoutSeconds must be > 0 and <= " + MAX_TIMEOUT_SECONDS;
                }
                return null;

            case hold:
                if (step.durationSeconds < 0 || step.durationSeconds > MAX_STEP_DURATION_SECONDS) {
                    return path + ": hold.durationSeconds must be >= 0 and <= " + MAX_STEP_DURATION_SECONDS;
                }
                return null;

            default:
                return path + ": unknown step type";
        }
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    /**
     * Custom deserializer that handles the intake action field name collision.
     * In JSON, intake steps use "action" for the intake action, but we store it
     * as intakeAction in the Java model to avoid collision with shooter's action field.
     */
    private static class AutoStepDeserializer implements JsonDeserializer<AutoStep> {

        @Override
        public AutoStep deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context)
                throws JsonParseException {
            JsonObject obj = json.getAsJsonObject();
            AutoStep step = new AutoStep();

            if (!obj.has("type")) {
                throw new JsonParseException("Step is missing required field 'type'");
            }
            step.type = parseEnum(
                    obj.get("type").getAsString(),
                    AutoStep.StepType.class,
                    "type");

            switch (step.type) {
                case drive:
                    step.direction = getDouble(obj, "direction");
                    step.speedMPS = getDouble(obj, "speedMPS");
                    if (obj.has("mode")) {
                        step.mode = parseEnum(obj.get("mode").getAsString(), AutoStep.DriveMode.class, "mode");
                    }
                    step.distanceMetres = getDouble(obj, "distanceMetres");
                    step.durationSeconds = getDouble(obj, "durationSeconds");
                    step.headingDegrees = getDouble(obj, "headingDegrees");
                    step.timeoutSeconds = getDouble(obj, "timeoutSeconds");
                    step.xMetres = getDouble(obj, "xMetres");
                    step.yMetres = getDouble(obj, "yMetres");
                    step.positionToleranceMetres = getDouble(obj, "positionToleranceMetres");
                    step.headingToleranceDegrees = getDouble(obj, "headingToleranceDegrees");
                    break;

                case rotate:
                    step.headingDegrees = getDouble(obj, "headingDegrees");
                    step.timeoutSeconds = getDouble(obj, "timeoutSeconds");
                    break;

                case shooter:
                    if (obj.has("action")) {
                        step.action = parseEnum(
                                obj.get("action").getAsString(),
                                AutoStep.ShooterAction.class,
                                "action");
                    }
                    step.rpm = getDouble(obj, "rpm");
                    step.hoodPosition = getDouble(obj, "hoodPosition");
                    step.kickerSpeed = getDouble(obj, "kickerSpeed");
                    step.kickerDelaySeconds = getDouble(obj, "kickerDelaySeconds");
                    step.durationSeconds = getDouble(obj, "durationSeconds");
                    break;

                case intake:
                    if (obj.has("action")) {
                        step.intakeAction = parseEnum(
                                obj.get("action").getAsString(),
                                AutoStep.IntakeAction.class,
                                "action");
                    }
                    step.speed = getDouble(obj, "speed");
                    step.durationSeconds = getDouble(obj, "durationSeconds");
                    break;

                case delay:
                    step.durationSeconds = getDouble(obj, "durationSeconds");
                    break;

                case parallel:
                    if (obj.has("endCondition")) {
                        step.endCondition = parseEnum(
                                obj.get("endCondition").getAsString(),
                                AutoStep.ParallelEndCondition.class,
                                "endCondition");
                    }
                    step.deadlineIndex = getInt(obj, "deadlineIndex");
                    if (obj.has("commands")) {
                        step.commands = new ArrayList<>();
                        for (JsonElement elem : obj.getAsJsonArray("commands")) {
                            step.commands.add(context.deserialize(elem, AutoStep.class));
                        }
                    }
                    break;

                case drive_velocity:
                    if (obj.has("frame")) {
                        step.frame = parseEnum(
                                obj.get("frame").getAsString(),
                                AutoStep.VelocityFrame.class,
                                "frame");
                    }
                    step.vxMPS = getDouble(obj, "vxMPS");
                    step.vyMPS = getDouble(obj, "vyMPS");
                    step.headingDegrees = getDouble(obj, "headingDegrees");
                    step.durationSeconds = getDouble(obj, "durationSeconds");
                    break;

                case face_target:
                    if (obj.has("target")) {
                        step.target = parseEnum(
                                obj.get("target").getAsString(),
                                AutoStep.FaceTargetType.class,
                                "target");
                    }
                    step.targetXMetres = getDouble(obj, "targetXMetres");
                    step.targetYMetres = getDouble(obj, "targetYMetres");
                    step.headingToleranceDegrees = getDouble(obj, "headingToleranceDegrees");
                    step.timeoutSeconds = getDouble(obj, "timeoutSeconds");
                    break;

                case vision_approach_tag:
                    step.rightSide = getBoolean(obj, "rightSide");
                    step.timeoutSeconds = getDouble(obj, "timeoutSeconds");
                    break;

                case hold:
                    step.durationSeconds = getDouble(obj, "durationSeconds");
                    break;
            }

            return step;
        }

        private <T extends Enum<T>> T parseEnum(String value, Class<T> enumClass, String fieldName) {
            try {
                return Enum.valueOf(enumClass, value);
            } catch (IllegalArgumentException e) {
                throw new JsonParseException("Invalid value '" + value + "' for field '" + fieldName + "'");
            }
        }

        private double getDouble(JsonObject obj, String field) {
            return obj.has(field) ? obj.get(field).getAsDouble() : 0;
        }

        private int getInt(JsonObject obj, String field) {
            return obj.has(field) ? obj.get(field).getAsInt() : 0;
        }

        private boolean getBoolean(JsonObject obj, String field) {
            return obj.has(field) && obj.get(field).getAsBoolean();
        }
    }
}
