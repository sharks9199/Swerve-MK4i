package frc.robot.sharks.subsystems.swerve.control;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.sharks.subsystems.swerve.control.TranslationSlewLimiter;

import org.junit.jupiter.api.Test;

class TranslationSlewLimiterTest {
  @Test void limitsVectorMagnitudeNotAxesSeparately() {
    var limiter = new TranslationSlewLimiter(2.0, 0.02);
    Translation2d next = limiter.calculate(new Translation2d(3.0, 4.0));
    assertEquals(0.04, next.getNorm(), 1e-9);
    assertEquals(0.6, next.getX() / next.getNorm(), 1e-9);
    assertEquals(0.8, next.getY() / next.getNorm(), 1e-9);
  }

  @Test void invalidInputsDoNotPoisonFollowingCycles() {
    var limiter = new TranslationSlewLimiter(2.0, 0.02);
    assertEquals(0.0, limiter.calculate(new Translation2d(Double.NaN, 0.0)).getNorm(), 1e-9);
    assertEquals(0.0, limiter.calculate(new Translation2d(1.0, 0.0), Double.POSITIVE_INFINITY).getNorm(), 1e-9);
    limiter.reset(new Translation2d(Double.POSITIVE_INFINITY, 0.0));
    assertEquals(0.04, limiter.calculate(new Translation2d(1.0, 0.0)).getNorm(), 1e-9);
  }

  @Test void invalidConfigurationIsRejected() {
    assertThrows(IllegalArgumentException.class, () -> new TranslationSlewLimiter(Double.NaN, 0.02));
    assertThrows(IllegalArgumentException.class, () -> new TranslationSlewLimiter(2.0, 0.0));
  }

  @Test void resetClearsOldVelocity() {
    var limiter = new TranslationSlewLimiter(2.0, 0.02);
    limiter.calculate(new Translation2d(3, 0));
    limiter.reset(Translation2d.kZero);
    assertEquals(0, limiter.calculate(Translation2d.kZero).getNorm(), 1e-9);
  }
}
