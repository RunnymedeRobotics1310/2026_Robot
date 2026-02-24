package frc.robot.commands.auto.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

class AutoConfigParserTest {

    @Test
    void sanitizeConfigNameNormalizesInput() {
        assertEquals("my_auto_2026", AutoConfigParser.sanitizeConfigName(" My Auto 2026 "));
        assertNull(AutoConfigParser.sanitizeConfigName("___"));
    }

    @Test
    void validateAutoConfigAcceptsValidDriveConfig() {
        AutoStep step = new AutoStep();
        step.type = AutoStep.StepType.drive;
        step.mode = AutoStep.DriveMode.distance;
        step.direction = 0;
        step.speedMPS = 1.0;
        step.distanceMetres = 1.2;
        step.headingDegrees = 0;
        step.timeoutSeconds = 3.0;

        AutoConfig config = new AutoConfig();
        config.name = "test_auto";
        config.version = 1;
        config.startingHeadingDegrees = 0;
        config.steps = List.of(step);

        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void validateAutoConfigRejectsNestedParallel() {
        AutoStep nested = new AutoStep();
        nested.type = AutoStep.StepType.parallel;
        nested.endCondition = AutoStep.ParallelEndCondition.all;
        nested.commands = List.of(buildDelayStep(0.5), buildDelayStep(0.5));

        AutoStep parent = new AutoStep();
        parent.type = AutoStep.StepType.parallel;
        parent.endCondition = AutoStep.ParallelEndCondition.all;
        parent.commands = List.of(buildDelayStep(0.5), nested);

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
        AutoStep parallel = new AutoStep();
        parallel.type = AutoStep.StepType.parallel;
        parallel.endCondition = AutoStep.ParallelEndCondition.deadline;
        parallel.deadlineIndex = 3;
        parallel.commands = List.of(buildDelayStep(0.5), buildDelayStep(0.5));

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
        AutoStep driveToPose = new AutoStep();
        driveToPose.type = AutoStep.StepType.drive;
        driveToPose.mode = AutoStep.DriveMode.to_pose;
        driveToPose.speedMPS = 2.0;
        driveToPose.xMetres = 2.4;
        driveToPose.yMetres = 4.0;
        driveToPose.headingDegrees = 180;
        driveToPose.positionToleranceMetres = 0.05;
        driveToPose.headingToleranceDegrees = 2.0;
        driveToPose.timeoutSeconds = 6.0;

        AutoStep hold = new AutoStep();
        hold.type = AutoStep.StepType.hold;
        hold.durationSeconds = 1.0;

        AutoConfig config = new AutoConfig();
        config.name = "new_types_ok";
        config.version = 1;
        config.startingHeadingDegrees = 0;
        config.steps = List.of(driveToPose, hold);

        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void validateAutoConfigAcceptsDeadlineParallelWithHoldUntilInterrupted() {
        AutoStep shooter = new AutoStep();
        shooter.type = AutoStep.StepType.shooter;
        shooter.action = AutoStep.ShooterAction.on_with_duration;
        shooter.rpm = 1500;
        shooter.hoodPosition = 0.0;
        shooter.kickerSpeed = -0.7;
        shooter.kickerDelaySeconds = 2.0;
        shooter.durationSeconds = 10.0;

        AutoStep hold = new AutoStep();
        hold.type = AutoStep.StepType.hold;
        hold.durationSeconds = 0.0;

        AutoStep parallel = new AutoStep();
        parallel.type = AutoStep.StepType.parallel;
        parallel.endCondition = AutoStep.ParallelEndCondition.deadline;
        parallel.deadlineIndex = 0;
        parallel.commands = List.of(shooter, hold);

        AutoConfig config = new AutoConfig();
        config.name = "deadline_hold_ok";
        config.version = 1;
        config.startingHeadingDegrees = 0;
        config.steps = List.of(parallel);

        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void parseJsonParsesNewStepTypes() {
        String json = """
                {
                  "name": "new_steps",
                  "version": 1,
                  "startingHeadingDegrees": 0,
                  "steps": [
                    {
                      "type": "drive_velocity",
                      "frame": "robot",
                      "vxMPS": 1.0,
                      "vyMPS": 0.5,
                      "headingDegrees": 0,
                      "durationSeconds": 1.5
                    },
                    {
                      "type": "face_target",
                      "target": "hub",
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
        assertEquals(3, config.steps.size());
        assertEquals(AutoStep.StepType.drive_velocity, config.steps.get(0).type);
        assertEquals(AutoStep.VelocityFrame.robot, config.steps.get(0).frame);
        assertEquals(AutoStep.StepType.face_target, config.steps.get(1).type);
        assertEquals(AutoStep.FaceTargetType.hub, config.steps.get(1).target);
        assertEquals(AutoStep.StepType.vision_approach_tag, config.steps.get(2).type);
    }

    @Test
    void parseJsonRejectsInvalidStepType() {
        String invalidJson = """
                {
                  "name": "bad_auto",
                  "version": 1,
                  "startingHeadingDegrees": 0,
                  "steps": [
                    { "type": "not_a_real_step" }
                  ]
                }
                """;

        assertNull(AutoConfigParser.parseJson(invalidJson));
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
    void opportunisticOutpostTemplateParsesAndValidates() throws IOException {
        Path template = Path.of("src/main/deploy/autos/opportunistic_outpost_configurable.json");
        String json = Files.readString(template);

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals("opportunistic_outpost_configurable", config.name);
        assertEquals(8, config.steps.size());
        assertNull(AutoConfigParser.validateAutoConfig(config));

        AutoStep parallel = config.steps.get(5);
        assertEquals(AutoStep.StepType.parallel, parallel.type);
        assertEquals(AutoStep.ParallelEndCondition.deadline, parallel.endCondition);
        assertEquals(0, parallel.deadlineIndex);
        assertEquals(AutoStep.StepType.hold, parallel.commands.get(1).type);
        assertEquals(0.0, parallel.commands.get(1).durationSeconds);
    }

    @Test
    void simpleCenterTemplateParsesAndValidates() throws IOException {
        Path template = Path.of("src/main/deploy/autos/simple_center_configurable.json");
        String json = Files.readString(template);

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals("simple_center_configurable", config.name);
        assertEquals(4, config.steps.size());
        assertNull(AutoConfigParser.validateAutoConfig(config));

        AutoStep first = config.steps.get(0);
        assertEquals(AutoStep.StepType.parallel, first.type);
        assertEquals(AutoStep.ParallelEndCondition.deadline, first.endCondition);
        assertEquals(0, first.deadlineIndex);
        assertEquals(AutoStep.StepType.shooter, first.commands.get(0).type);
        assertEquals(AutoStep.StepType.hold, first.commands.get(1).type);

        AutoStep finalStep = config.steps.get(3);
        assertEquals(AutoStep.StepType.vision_approach_tag, finalStep.type);
        assertTrue(finalStep.rightSide);
    }

    @Test
    void odometryStraightCompactTemplateParsesAndValidates() throws IOException {
        Path template = Path.of("src/main/deploy/autos/odometry_cal_straight_compact.json");
        String json = Files.readString(template);

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals("odometry_cal_straight_compact", config.name);
        assertEquals(3, config.steps.size());
        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void odometrySpinCompactTemplateParsesAndValidates() throws IOException {
        Path template = Path.of("src/main/deploy/autos/odometry_cal_spin_compact.json");
        String json = Files.readString(template);

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals("odometry_cal_spin_compact", config.name);
        assertEquals(18, config.steps.size());
        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    @Test
    void odometryTurnCompactTemplateParsesAndValidates() throws IOException {
        Path template = Path.of("src/main/deploy/autos/odometry_cal_turn_compact.json");
        String json = Files.readString(template);

        AutoConfig config = AutoConfigParser.parseJson(json);
        assertNotNull(config);
        assertEquals("odometry_cal_turn_compact", config.name);
        assertEquals(18, config.steps.size());
        assertNull(AutoConfigParser.validateAutoConfig(config));
    }

    private AutoStep buildDelayStep(double durationSeconds) {
        AutoStep step = new AutoStep();
        step.type = AutoStep.StepType.delay;
        step.durationSeconds = durationSeconds;
        return step;
    }
}
