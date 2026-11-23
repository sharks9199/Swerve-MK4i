package frc.robot.sharks.subsystems.swerve.control;

/** Simple sensor-health interlock. This is NOT a tip-over or physics safety system. */
public final class DriveSensorHealth {
  private DriveSensorHealth() {}

  /** True if gyro or any of the four swerve modules is unavailable. */
  public static boolean criticalFault(boolean gyroConnected, boolean[] modulesConnected) {
    if (!gyroConnected || modulesConnected == null || modulesConnected.length != 4) return true;
    for (boolean connected : modulesConnected) {
      if (!connected) return true;
    }
    return false;
  }
}
