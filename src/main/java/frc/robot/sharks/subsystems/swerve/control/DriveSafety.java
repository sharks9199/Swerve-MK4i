package frc.robot.sharks.subsystems.swerve.control;

/** Stateful fault latch. Disabled operation never clears a latched fault. */
public final class DriveSafety {
  private boolean latched;
  private boolean healthy;
  private double healthySince = Double.NaN;
  public void update(boolean fault, boolean enabled, double now) {
    healthy = !fault && Double.isFinite(now);
    if (!healthy) {
      healthySince = Double.NaN;
      if (enabled) latched = true;
    } else if (!Double.isFinite(healthySince) || now < healthySince) {
      healthySince = now;
    }
  }
  public boolean blocked() { return latched || !healthy; }
  public boolean latched() { return latched; }
  public boolean rearm(double now, double delay, boolean neutral) {
    if (!neutral || !healthy || !Double.isFinite(now) || !Double.isFinite(delay)
        || delay < 0 || !Double.isFinite(healthySince) || now - healthySince < delay) return false;
    latched = false;
    return true;
  }
  /** Monotonic derating, 0.35 at stop threshold; hard stop handled separately. */
  public static double factor(double value, double full, double stop) {
    if (!Double.isFinite(value) || !Double.isFinite(full) || !Double.isFinite(stop)
        || full == stop) return 0.0;
    return 0.35 + 0.65 * Math.max(0.0, Math.min(1.0, (value-stop)/(full-stop)));
  }
  public static double step(double current, double target, double limit) {
    return current + Math.max(-limit, Math.min(limit, target-current));
  }
}
