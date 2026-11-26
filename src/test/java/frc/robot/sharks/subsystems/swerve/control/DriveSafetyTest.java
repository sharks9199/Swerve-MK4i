package frc.robot.sharks.subsystems.swerve.control;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class DriveSafetyTest {
  @Test void faultLatchesAcrossRecoveryAndDisable() {
    DriveSafety s = new DriveSafety();
    s.update(false, true, 1.0);
    assertFalse(s.blocked());
    s.update(true, true, 2.0);
    assertTrue(s.latched());
    s.update(false, false, 3.0);
    assertTrue(s.blocked());
    assertFalse(s.rearm(3.1, 0.5, true));
    assertFalse(s.rearm(4.0, 0.5, false));
    assertTrue(s.rearm(4.0, 0.5, true));
    assertFalse(s.blocked());
  }
  @Test void startupFaultBlocksWithoutLatchingWhenDisabled() {
    DriveSafety s = new DriveSafety();
    s.update(true, false, 0.0);
    assertTrue(s.blocked());
    assertFalse(s.latched());
    s.update(false, false, 1.0);
    assertFalse(s.blocked());
  }
  @Test void factorAndRampAreBounded() {
    assertEquals(1.0, DriveSafety.factor(12, 10, 7), 1e-9);
    assertEquals(0.35, DriveSafety.factor(7, 10, 7), 1e-9);
    assertEquals(1.0, DriveSafety.factor(5, 10, 18), 1e-9);
    assertEquals(0.35, DriveSafety.factor(18, 10, 18), 1e-9);
    assertEquals(0.0, DriveSafety.factor(Double.NaN, 10, 7), 1e-9);
    assertEquals(0.1, DriveSafety.step(0, 3, 0.1), 1e-9);
    assertEquals(-0.1, DriveSafety.step(0, -3, 0.1), 1e-9);
  }
}
