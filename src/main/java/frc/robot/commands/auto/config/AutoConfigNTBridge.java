package frc.robot.commands.auto.config;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedString;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class AutoConfigNTBridge {

    private final NetworkTable table;
    private final StringArrayPublisher availableAutosPub;
    private final StringArrayPublisher deployAutosPub;
    private final StringArrayPublisher runtimeAutosPub;
    private final StringPublisher lastWriteStatusPub;
    private final StringSubscriber writeConfigSub;
    private final StringSubscriber deleteConfigSub;
    private final Map<String, StringPublisher> configPublishers = new HashMap<>();

    private Runnable onConfigsChanged;

    public AutoConfigNTBridge() {
        table = NetworkTableInstance.getDefault()
                .getTable("SmartDashboard")
                .getSubTable("1310")
                .getSubTable("autoconfig");

        availableAutosPub = table.getStringArrayTopic("availableAutos").publish();
        deployAutosPub = table.getStringArrayTopic("deployAutos").publish();
        runtimeAutosPub = table.getStringArrayTopic("runtimeAutos").publish();
        lastWriteStatusPub = table.getStringTopic("lastWriteStatus").publish();
        writeConfigSub = table.getStringTopic("writeConfig").subscribe("");
        deleteConfigSub = table.getStringTopic("deleteConfig").subscribe("");

        // Initial publish of available configs
        refreshAvailableAutos();
    }

    public void setOnConfigsChanged(Runnable callback) {
        this.onConfigsChanged = callback;
    }

    public void periodic() {
        // Process every queued write request so repeated identical writes are not dropped.
        TimestampedString[] writeQueue = writeConfigSub.readQueue();
        for (TimestampedString update : writeQueue) {
            if (!update.value.isEmpty()) {
                handleWriteConfig(update.value);
            }
        }

        // Process every queued delete request so repeated identical deletes are not dropped.
        TimestampedString[] deleteQueue = deleteConfigSub.readQueue();
        for (TimestampedString update : deleteQueue) {
            if (!update.value.isEmpty()) {
                handleDeleteConfig(update.value);
            }
        }
    }

    private void handleWriteConfig(String json) {
        if (!DriverStation.isDisabled()) {
            lastWriteStatusPub.set("Error: Config changes are only allowed while Disabled");
            return;
        }

        AutoConfig config = AutoConfigParser.parseJson(json);
        if (config == null) {
            lastWriteStatusPub.set("Error: Invalid JSON");
            return;
        }
        String validationError = AutoConfigParser.validateAutoConfig(config);
        if (validationError != null) {
            lastWriteStatusPub.set("Error: " + validationError);
            return;
        }

        String safeName = AutoConfigParser.sanitizeConfigName(config.name);
        if (safeName == null) {
            lastWriteStatusPub.set("Error: Invalid config name");
            return;
        }

        String result = AutoConfigParser.saveAutoConfig(safeName, json);
        lastWriteStatusPub.set(result);

        if (result.startsWith("ok")) {
            refreshAvailableAutos();
            publishConfig(safeName, json);
            if (onConfigsChanged != null) {
                onConfigsChanged.run();
            }
        }
    }

    private void handleDeleteConfig(String name) {
        if (!DriverStation.isDisabled()) {
            lastWriteStatusPub.set("Error: Config changes are only allowed while Disabled");
            return;
        }

        String safeName = AutoConfigParser.sanitizeConfigName(name);
        if (safeName == null) {
            lastWriteStatusPub.set("Error: Invalid config name");
            return;
        }

        boolean deleted = AutoConfigParser.deleteAutoConfig(safeName);
        if (deleted) {
            lastWriteStatusPub.set("ok");
            // Remove the config topic publisher
            StringPublisher pub = configPublishers.remove(safeName);
            if (pub != null) {
                pub.close();
            }
            refreshAvailableAutos();
            if (onConfigsChanged != null) {
                onConfigsChanged.run();
            }
        } else {
            if (AutoConfigParser.isDeployConfig(safeName)) {
                lastWriteStatusPub.set("Error: Deploy config is immutable: " + safeName);
                return;
            }
            lastWriteStatusPub.set("Error: Config not found: " + safeName);
        }
    }

    private void refreshAvailableAutos() {
        List<String> configs = AutoConfigParser.listAutoConfigs();
        List<String> deployConfigs = AutoConfigParser.listDeployAutoConfigs();
        List<String> runtimeConfigs = AutoConfigParser.listRuntimeAutoConfigs();
        Set<String> configSet = new HashSet<>(configs);

        // Close any stale publishers for configs that no longer exist.
        configPublishers.entrySet().removeIf(entry -> {
            if (!configSet.contains(entry.getKey())) {
                entry.getValue().close();
                return true;
            }
            return false;
        });

        availableAutosPub.set(configs.toArray(new String[0]));
        deployAutosPub.set(deployConfigs.toArray(new String[0]));
        runtimeAutosPub.set(runtimeConfigs.toArray(new String[0]));

        // Publish each config's JSON
        for (String name : configs) {
            String json = AutoConfigParser.readAutoConfigJson(name);
            if (json != null) {
                publishConfig(name, json);
            }
        }
    }

    private void publishConfig(String name, String json) {
        StringPublisher pub = configPublishers.get(name);
        if (pub == null) {
            pub = table.getSubTable("configs").getStringTopic(name).publish();
            configPublishers.put(name, pub);
        }
        pub.set(json);
    }
}
