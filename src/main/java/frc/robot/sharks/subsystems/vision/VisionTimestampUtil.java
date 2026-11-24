package frc.robot.sharks.subsystems.vision;

/** Checks a real Limelight pose frame and computes its capture time in FPGA seconds. */
public final class VisionTimestampUtil {
  private VisionTimestampUtil() {}

  /**
   * @param updateMicros NT entry's last-update timestamp (server time, microseconds)
   * @param latencyMs camera latency in milliseconds
   * @param nowSeconds current FPGA time, in seconds
   * @param maxAgeSeconds maximum acceptable age of the received message
   * @return estimated frame capture timestamp in seconds, or zero if invalid/stale
   */
  public static double timestampCaptura(
      long updateMicros, double latencyMs, double nowSeconds, double maxAgeSeconds) {
    if (updateMicros <= 0 || !Double.isFinite(latencyMs) || latencyMs < 0
        || !Double.isFinite(nowSeconds) || !Double.isFinite(maxAgeSeconds)
        || maxAgeSeconds <= 0) {
      return 0.0;
    }
    double publishedAt = updateMicros / 1_000_000.0;
    if (publishedAt > nowSeconds + 0.10 || nowSeconds - publishedAt > maxAgeSeconds) {
      return 0.0;
    }
    double capturedAt = publishedAt - latencyMs / 1000.0;
    return capturedAt > 0.0 && capturedAt <= nowSeconds + 0.10
        && nowSeconds - capturedAt <= maxAgeSeconds
        ? capturedAt : 0.0;
  }
}
