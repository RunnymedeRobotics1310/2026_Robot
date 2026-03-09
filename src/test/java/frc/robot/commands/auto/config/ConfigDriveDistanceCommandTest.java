package frc.robot.commands.auto.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.commands.swerve.DriveDistanceCommand;
import org.junit.jupiter.api.Test;

class ConfigDriveDistanceCommandTest {

  @Test
  void calculateStopDistanceIncludesBrakingDelayAndMargin() {
    double stopDistance = DriveDistanceCommand.calculateStopDistanceMetres(1.5, 10.0, 0.06, 0.02);

    assertEquals(0.2225, stopDistance, 1e-9);
  }

  @Test
  void calculateStopDistanceGrowsWithSpeed() {
    double atOneMPS = DriveDistanceCommand.calculateStopDistanceMetres(1.0, 10.0, 0.06, 0.02);
    double atTwoMPS = DriveDistanceCommand.calculateStopDistanceMetres(2.0, 10.0, 0.06, 0.02);

    assertTrue(atTwoMPS > atOneMPS);
  }

  @Test
  void calculateStopDistanceClampsNegativeInputs() {
    double stopDistance = DriveDistanceCommand.calculateStopDistanceMetres(-1.0, -2.0, -0.1, -0.5);

    assertEquals(0.0, stopDistance, 1e-9);
  }
}
