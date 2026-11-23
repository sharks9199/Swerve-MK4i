package frc.robot.sharks.subsystems.swerve.control;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import frc.robot.sharks.subsystems.swerve.control.SnapController;

class SnapControllerTest {
  @Test void snapsToLongitudinalAxisWithHysteresis() {
    var snap = new SnapController();
    var t1 = snap.process(0.75, 0.10);
    assertEquals(0.0, t1.getY(), 1e-8);
    assertTrue(snap.isSnappingLongitudinal());
    var t2 = snap.process(0.75, 0.17);
    assertEquals(0.0, t2.getY(), 1e-8);
    var t3 = snap.process(0.75, 0.25);
    assertFalse(snap.isSnappingLongitudinal());
    assertTrue(t3.getY() > 0.0);
  }

  @Test void resetAndDeadband() {
    var snap = new SnapController();
    assertEquals(0.0, snap.process(0.01, 0.01).getNorm(), 1e-8);
    snap.process(0.70, 0.05);
    snap.reset();
    assertFalse(snap.isSnappingLongitudinal());
  }
}
