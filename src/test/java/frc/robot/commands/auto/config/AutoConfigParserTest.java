package frc.robot.commands.auto.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class AutoConfigParserTest {

    @Test
    void sanitizeConfigNameNormalizesInput() {
        assertEquals("my_auto_2026", AutoConfigParser.sanitizeConfigName(" My Auto 2026 "));
        assertNull(AutoConfigParser.sanitizeConfigName("___"));
    }

    @Test
    void validateAutoConfigAcceptsValidDriveConfig() {
        Map<String, Object> step = new HashMap<>();
        step.put("type", "drive_distance");
        step.put("direction", 0.0);
        step.put("speedMPS", 1.0);
        step.put("distanceMetres", 1.2);
        step.put("headingDegrees", 0.0);
        step.put("timeoutSeconds", 3.0);

        AutoConfig config = new AutoConfig();
        config.name = "test_auto";
        config.version = 1;
        config.startingHeadingDegrees = 0;
        config.steps = List.of(step);

        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void validateAutoConfigRejectsNestedParallel() {
        Map<String, Object> nested = new HashMap<>();
        nested.put("type", "parallel");
        nested.put("endCondition", "all");
        nested.put("commands", List.of(buildDelayStep(0.5), buildDelayStep(0.5)));

        Map<String, Object> parent = new HashMap<>();
        parent.put("type", "parallel");
        parent.put("endCondition", "all");
        parent.put("commands", List.of(buildDelayStep(0.5), nested));

        AutoConfig config = new AutoConfig();
        config.name = "nested_parallel";
        config.version = 1;
        config.startingHeadingDegrees = 0;
        config.steps = List.of(parent);

        String error = AutoConfigParser.validateAutoConfig(config);
        assertNotNull(error);
        assertTrue(error.contains("nested parallel"));
    }

    @Test
    void validateAutoConfigRejectsInvalidDeadlineIndex() {
        Map<String, Object> parallel = new HashMap<>();
        parallel.put("type", "parallel");
        parallel.put("endCondition", "deadline");
        parallel.put("deadlineIndex", 3.0);
        parallel.put("commands", List.of(buildDelayStep(0.5), buildDelayStep(0.5)));

        AutoConfig config = new AutoConfig();
        config.name = "deadline_bad";
        config.version = 1;
        config.startingHeadingDegrees = 0;
        config.steps = List.of(parallel);

        String error = AutoConfigParser.validateAutoConfig(config);
        assertNotNull(error);
        assertTrue(error.contains("deadlineIndex"));
    }

    @Test
    void validateAutoConfigAcceptsDriveToPoseAndHold() {
        Map<String, Object> driveToPose = new HashMap<>();
        driveToPose.put("type", "drive_to_pose");
        driveToPose.put("speedMPS", 2.0);
        driveToPose.put("xMetres", 2.4);
        driveToPose.put("yMetres", 4.0);
        driveToPose.put("headingDegrees", 180.0);
        driveToPose.put("positionToleranceMetres", 0.05);
        driveToPose.put("headingToleranceDegrees", 2.0);
        driveToPose.put("timeoutSeconds", 6.0);

        Map<String, Object> hold = new HashMap<>();
        hold.put("type", "hold");
        hold.put("durationSeconds", 1.0);

        AutoConfig config = new AutoConfig();
        config.name = "new_types_ok";
        config.version = 1;
        config.startingHeadingDegrees = 0;
        config.steps = List.of(driveToPose, hold);

        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void validateAutoConfigAcceptsSetPoseStep() {
        Map<String, Object> setPose = new HashMap<>();
        setPose.put("type", "set_pose");
        setPose.put("xMetres", 1.5);
        setPose.put("yMetres", 2.25);
        setPose.put("headingDegrees", 90.0);

        AutoConfig config = new AutoConfig();
        config.name = "set_pose_ok";
        config.version = 1;
        config.startingHeadingDegrees = 0;
        config.steps = List.of(setPose, buildDelayStep(0.5));

        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void validateAutoConfigAcceptsDeadlineParallelWithHoldUntilInterrupted() {
        Map<String, Object> shooter = new HashMap<>();
        shooter.put("type", "shooter");
        shooter.put("action", "on_with_duration");
        shooter.put("rpm", 1500.0);
        shooter.put("hoodPosition", 0.0);
        shooter.put("kickerSpeed", -0.7);
        shooter.put("kickerDelaySeconds", 2.0);
        shooter.put("durationSeconds", 10.0);

        Map<String, Object> hold = new HashMap<>();
        hold.put("type", "hold");
        hold.put("durationSeconds", 0.0);

        Map<String, Object> parallel = new HashMap<>();
        parallel.put("type", "parallel");
        parallel.put("endCondition", "deadline");
        parallel.put("deadlineIndex", 0.0);
        parallel.put("commands", List.of(shooter, hold));

        AutoConfig config = new AutoConfig();
        config.name = "deadline_hold_ok";
        config.version = 1;
        config.startingHeadingDegrees = 0;
        config.steps = List.of(parallel);

        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    @SuppressWarnings("unchecked")
    void parseJsonParsesNewStepTypes() {
        String json = """
                {
                  "name": "new_steps",
                  "version": 1,
                  "startingHeadingDegrees": 0,
                  "steps": [
                    {
                      "type": "set_pose",
                      "xMetres": 1.0,
                      "yMetres": 2.0,
                      "headingDegrees": 45
                    },
                    {
                      "type": "drive_velocity",
                      "vxMPS": 1.0,
                      "vyMPS": 0.5,
                      "headingDegrees": 0,
                      "timeoutSeconds": 1.5
                    },
                    {
                      "type": "face_hub",
                      "headingToleranceDegrees": 3,
                      "timeoutSeconds": 2
                    },
                    {
                      "type": "vision_approach_tag",
                      "rightSide": false,
                      "timeoutSeconds": 5
                    }
                  ]
                }
                """;
        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals(4, config.steps.size());
        assertEquals("set_pose", config.steps.get(0).get("type"));
        assertEquals("drive_velocity", config.steps.get(1).get("type"));
        assertEquals("face_hub", config.steps.get(2).get("type"));
        assertEquals("vision_approach_tag", config.steps.get(3).get("type"));
    }

    @Test
    void parseJsonAcceptsDriveVelocityType() {
        String json = """
                {
                  "name": "velocity_type",
                  "version": 1,
                  "startingHeadingDegrees": 0,
                  "steps": [
                    {
                      "type": "drive_velocity",
                      "vxMPS": 0.8,
                      "vyMPS": 0.2,
                      "headingDegrees": 15,
                      "timeoutSeconds": 1.2
                    }
                  ]
                }
                """;

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals(1, config.steps.size());
        assertEquals("drive_velocity", config.steps.get(0).get("type"));
        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void parseJsonAcceptsUnknownStepTypes() {
        String json = """
                {
                  "name": "unknown_type",
                  "version": 1,
                  "startingHeadingDegrees": 0,
                  "steps": [
                    { "type": "some_new_command", "param1": 42 }
                  ]
                }
                """;

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals(1, config.steps.size());
        // Unknown types are now accepted at parse time; validation happens at creation
        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void saveAutoConfigStoresRuntimeOnly() {
        String json = """
                {
                  "name": "runtime_test_auto",
                  "version": 1,
                  "startingHeadingDegrees": 0,
                  "steps": [
                    { "type": "delay", "durationSeconds": 1.0 }
                  ]
                }
                """;

        assertEquals("ok:runtime", AutoConfigParser.saveAutoConfig("runtime_test_auto", json));
        assertTrue(AutoConfigParser.isRuntimeConfig("runtime_test_auto"));
        assertTrue(AutoConfigParser.listRuntimeAutoConfigs().contains("runtime_test_auto"));
        assertNotNull(AutoConfigParser.loadAutoConfig("runtime_test_auto"));
        assertTrue(AutoConfigParser.deleteAutoConfig("runtime_test_auto"));
        assertFalse(AutoConfigParser.isRuntimeConfig("runtime_test_auto"));
    }

    @Test
    @SuppressWarnings("unchecked")
    void opportunisticOutpostTemplateParsesAndValidates() throws IOException {
        Path template = Path.of("src/main/deploy/autos/opportunistic_outpost_configurable.json");
        String json = Files.readString(template);

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals("opportunistic_outpost_configurable", config.name);
        assertEquals(8, config.steps.size());
        assertNull(AutoConfigParser.validateAutoConfig(config));

        Map<String, Object> parallel = config.steps.get(5);
        assertEquals("parallel", parallel.get("type"));
        assertEquals("deadline", parallel.get("endCondition"));
        List<Map<String, Object>> cmds = (List<Map<String, Object>>) parallel.get("commands");
        assertEquals("hold", cmds.get(1).get("type"));
    }

    @Test
    @SuppressWarnings("unchecked")
    void simpleCenterTemplateParsesAndValidates() throws IOException {
        Path template = Path.of("src/main/deploy/autos/simple_center_configurable.json");
        String json = Files.readString(template);

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals("simple_center_configurable", config.name);
        assertEquals(4, config.steps.size());
        assertNull(AutoConfigParser.validateAutoConfig(config));

        Map<String, Object> first = config.steps.get(0);
        assertEquals("parallel", first.get("type"));
        assertEquals("deadline", first.get("endCondition"));
        List<Map<String, Object>> cmds = (List<Map<String, Object>>) first.get("commands");
        assertEquals("shooter", cmds.get(0).get("type"));
        assertEquals("hold", cmds.get(1).get("type"));

        Map<String, Object> finalStep = config.steps.get(3);
        assertEquals("vision_approach_tag", finalStep.get("type"));
        assertEquals(true, finalStep.get("rightSide"));
    }

    @Test
    void odometryStraightCompactTemplateParsesAndValidates() throws IOException {
        Path template = Path.of("src/main/deploy/autos/odometry_cal_straight_compact.json");
        String json = Files.readString(template);

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals("odometry_cal_straight_compact", config.name);
        assertEquals(4, config.steps.size());
        assertEquals("set_pose", config.steps.get(0).get("type"));
        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void odometrySpinCompactTemplateParsesAndValidates() throws IOException {
        Path template = Path.of("src/main/deploy/autos/odometry_cal_spin_compact.json");
        String json = Files.readString(template);

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals("odometry_cal_spin_compact", config.name);
        assertEquals(19, config.steps.size());
        assertEquals("set_pose", config.steps.get(0).get("type"));
        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void odometryTurnCompactTemplateParsesAndValidates() throws IOException {
        Path template = Path.of("src/main/deploy/autos/odometry_cal_turn_compact.json");
        String json = Files.readString(template);

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals("odometry_cal_turn_compact", config.name);
        assertEquals(19, config.steps.size());
        assertEquals("set_pose", config.steps.get(0).get("type"));
        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    private Map<String, Object> buildDelayStep(double durationSeconds) {
        Map<String, Object> step = new HashMap<>();
        step.put("type", "delay");
        step.put("durationSeconds", durationSeconds);
        return step;
    }
}
