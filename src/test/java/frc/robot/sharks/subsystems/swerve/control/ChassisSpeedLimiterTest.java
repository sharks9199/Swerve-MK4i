package frc.robot.sharks.subsystems.swerve.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

import frc.robot.sharks.subsystems.swerve.control.ChassisSpeedLimiter;


/** Testes isolados da matematica de limitacao, sem HAL nem simulacao. */
class ChassisSpeedLimiterTest {
  @Test
  void diagonalRespeitaNorma() {
    var r = ChassisSpeedLimiter.limitar(3.0, 4.0, 0.0, 3.0, 2.0);
    assertEquals(1.8, r.vxMetrosSegundo(), 1e-10);
    assertEquals(2.4, r.vyMetrosSegundo(), 1e-10);
  }

  @Test
  void angularEhLimitadaNosDoisSentidos() {
    assertEquals(2.0, ChassisSpeedLimiter.limitar(0, 0, 5, 3, 2).omegaRadSegundo());
    assertEquals(-2.0, ChassisSpeedLimiter.limitar(0, 0, -5, 3, 2).omegaRadSegundo());
  }

  @Test
  void entradaInvalidaNaoPropagaNan() {
    var r = ChassisSpeedLimiter.limitar(Double.NaN, Double.POSITIVE_INFINITY,
        Double.NEGATIVE_INFINITY, 3.0, 2.0);
    assertEquals(0.0, r.vxMetrosSegundo());
    assertEquals(0.0, r.vyMetrosSegundo());
    assertEquals(0.0, r.omegaRadSegundo());
  }

  @Test
  void limitesInvalidosSaoRejeitados() {
    assertThrows(IllegalArgumentException.class,
        () -> ChassisSpeedLimiter.limitar(1, 0, 0, Double.NaN, 1));
    assertThrows(IllegalArgumentException.class,
        () -> ChassisSpeedLimiter.limitar(1, 0, 0, 1, -1));
  }

  @Test
  void comandoAbaixoDoLimitePermaneceIgual() {
    var r = ChassisSpeedLimiter.limitar(1.0, -1.0, -0.5, 3.0, 2.0);
    assertEquals(1.0, r.vxMetrosSegundo());
    assertEquals(-1.0, r.vyMetrosSegundo());
    assertEquals(-0.5, r.omegaRadSegundo());
  }
}
