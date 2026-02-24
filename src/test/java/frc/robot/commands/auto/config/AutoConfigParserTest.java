package frc.robot.commands.auto.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

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

    private AutoStep buildDelayStep(double durationSeconds) {
        AutoStep step = new AutoStep();
        step.type = AutoStep.StepType.delay;
        step.durationSeconds = durationSeconds;
        return step;
    }
}
