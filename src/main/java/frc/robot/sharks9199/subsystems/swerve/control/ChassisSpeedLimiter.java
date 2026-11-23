package frc.robot.sharks9199.subsystems.swerve.control;

/**
 * Limita solicitacoes de velocidade do chassi de forma independente da WPILib.
 *
 * <p>O vetor de translacao e limitado por sua norma euclidiana, nao por eixo:
 * permitir 3 m/s simultaneamente em X e Y produziria 4,24 m/s na diagonal.
 * Valores nao finitos recebidos como solicitacao sao substituidos por zero.
 * Limites invalidos provocam falha explicita, evitando normalizacao silenciosa.
 *
 * <p>Este utilitario nao substitui limite de corrente, controle de aceleracao,
 * modelagem de tombamento ou validacao fisica no campo.
 */
public final class ChassisSpeedLimiter {
  private ChassisSpeedLimiter() {}

  /** Resultado imutavel em coordenadas do chassi. */
  public record Resultado(double vxMetrosSegundo, double vyMetrosSegundo, double omegaRadSegundo) {}

  /**
   * Limita velocidades de translacao e rotacao.
   *
   * @param vx velocidade longitudinal solicitada (m/s)
   * @param vy velocidade lateral solicitada (m/s)
   * @param omega velocidade angular solicitada (rad/s)
   * @param limiteLinear velocidade translacional maxima (m/s), nao negativa
   * @param limiteAngular velocidade angular maxima (rad/s), nao negativa
   * @return velocidades dentro dos limites, sem valores NaN/infinito
   */
  public static Resultado limitar(
      double vx, double vy, double omega, double limiteLinear, double limiteAngular) {
    if (!Double.isFinite(limiteLinear) || limiteLinear < 0.0
        || !Double.isFinite(limiteAngular) || limiteAngular < 0.0) {
      throw new IllegalArgumentException("Limites de velocidade devem ser finitos e nao negativos");
    }
    vx = Double.isFinite(vx) ? vx : 0.0;
    vy = Double.isFinite(vy) ? vy : 0.0;
    omega = Double.isFinite(omega) ? omega : 0.0;

    double norma = Math.hypot(vx, vy);
    if (norma > limiteLinear && norma > 0.0) {
      double proporcao = limiteLinear / norma;
      vx *= proporcao;
      vy *= proporcao;
    }
    omega = Math.max(-limiteAngular, Math.min(limiteAngular, omega));
    return new Resultado(vx, vy, omega);
  }
}
