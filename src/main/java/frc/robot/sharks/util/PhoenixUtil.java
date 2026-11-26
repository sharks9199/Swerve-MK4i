package frc.robot.sharks.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;

/** Helper methods for working with CTRE Phoenix 6 devices. */
public final class PhoenixUtil {
  private PhoenixUtil() {}

  /**
   * Attempts a Phoenix command multiple times until it succeeds.
   *
   * @param maxAttempts maximum number of attempts before giving up
   * @param command supplier that executes the command and returns its {@link StatusCode}
   */
  public static boolean tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    return tryUntilOk("Phoenix", maxAttempts, command);
  }

  public static boolean tryUntilOk(String device, int maxAttempts, Supplier<StatusCode> command) {
    StatusCode lastError = null;
    for (int i = 0; i < maxAttempts; i++) {
      lastError = command.get();
      if (lastError.isOK()) return true;
    }
    DriverStation.reportError(device + ": configuracao falhou: " + lastError, false);
    return false;
  }
}
