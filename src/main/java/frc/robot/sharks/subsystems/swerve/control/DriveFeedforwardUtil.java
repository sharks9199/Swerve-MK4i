package frc.robot.sharks.subsystems.swerve.control;

/** Calculates bounded acceleration feedforward voltages for voltage-controlled modules. */
public final class DriveFeedforwardUtil {
  private DriveFeedforwardUtil() {}

  /**
   * Wheel acceleration feedforward: volts = acceleration in m/s^2 times characterized kA.
   * kA must be measured (e.g., SysId). Keep disabled until tuned on a real robot.
   * @return one voltage per swerve module in FL, FR, BL, BR order
   */
  public static double[] calcularVolts(double[] accelerationsMps2, double kAVoltsPerMps2,
      double maxAbsoluteVolts, boolean enabled) {
    double[] output = new double[4];
    if (!enabled || accelerationsMps2 == null || accelerationsMps2.length != 4) return output;
    if (!Double.isFinite(kAVoltsPerMps2) || kAVoltsPerMps2 <= 0
        || !Double.isFinite(maxAbsoluteVolts) || maxAbsoluteVolts <= 0) return output;
    for (int i = 0; i < output.length; i++) {
      double acceleration = accelerationsMps2[i];
      if (Double.isFinite(acceleration)) {
        double volts = acceleration * kAVoltsPerMps2;
        output[i] = Math.max(-maxAbsoluteVolts, Math.min(maxAbsoluteVolts, volts));
      }
    }
    return output;
  }
}
