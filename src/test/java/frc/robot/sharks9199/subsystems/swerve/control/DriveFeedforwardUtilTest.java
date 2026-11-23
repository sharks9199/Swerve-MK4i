package frc.robot.sharks9199.subsystems.swerve.control;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import frc.robot.sharks.subsystems.swerve.control.DriveFeedforwardUtil;

class DriveFeedforwardUtilTest {
  @Test void disabledByDefault() {
    assertArrayEquals(new double[4], DriveFeedforwardUtil.calcularVolts(
        new double[]{20, -20, 30, 40}, 0.1, 2, false));
  }

  @Test void clampsAndRejectsNonFinite() {
    assertArrayEquals(new double[]{2, -2, 0, 0.5}, DriveFeedforwardUtil.calcularVolts(
        new double[]{50, -50, Double.NaN, 5}, 0.1, 2, true), 1e-8);
  }

  @Test void refusesUncalibratedGain() {
    assertArrayEquals(new double[4], DriveFeedforwardUtil.calcularVolts(
        new double[]{1, 2, 3, 4}, 0.0, 2, true));
  }

  @Test void rejectsWrongModuleCount() {
    assertArrayEquals(new double[4], DriveFeedforwardUtil.calcularVolts(
        new double[]{1, 2}, 0.1, 2, true));
  }
}
