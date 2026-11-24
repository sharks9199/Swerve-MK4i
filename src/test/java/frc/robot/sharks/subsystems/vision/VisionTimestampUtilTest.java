package frc.robot.sharks.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

import frc.robot.sharks.subsystems.vision.VisionTimestampUtil;

class VisionTimestampUtilTest {
  @Test void freshFrameKeepsOriginalTimestampAcrossLoops() {
    long updated = 2_000_000L;
    assertEquals(1.970, VisionTimestampUtil.timestampCaptura(updated, 30, 2.020, 0.5), 1e-9);
    assertEquals(1.970, VisionTimestampUtil.timestampCaptura(updated, 30, 2.040, 0.5), 1e-9);
  }

  @Test void freshPublicationWithOldCaptureIsDiscarded() {
    assertEquals(0, VisionTimestampUtil.timestampCaptura(2_000_000L, 700, 2.0, 0.5));
    assertEquals(1.95, VisionTimestampUtil.timestampCaptura(2_000_000L, 50, 2.0, 0.5), 1e-9);
  }

  @Test void staleFutureAndInvalidSamplesAreDiscarded() {
    assertEquals(0, VisionTimestampUtil.timestampCaptura(1_000_000L, 20, 2.0, 0.5));
    assertEquals(0, VisionTimestampUtil.timestampCaptura(2_500_000L, 20, 2.0, 0.5));
    assertEquals(0, VisionTimestampUtil.timestampCaptura(0, 20, 2.0, 0.5));
    assertEquals(0, VisionTimestampUtil.timestampCaptura(2_000_000L, Double.NaN, 2.0, 0.5));
  }
}
