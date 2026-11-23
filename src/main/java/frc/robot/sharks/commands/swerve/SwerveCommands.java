package frc.robot.sharks.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.sharks.Constants;
import frc.robot.sharks.subsystems.swerve.Drive;
import frc.robot.sharks.subsystems.swerve.control.HeadingController;
import frc.robot.sharks.subsystems.swerve.control.SnapController;
import frc.robot.sharks.subsystems.swerve.control.TranslationSlewLimiter;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** Factory methods for swerve drivetrain commands. */
public final class SwerveCommands {
  private SwerveCommands() {}

  /**
   * Creates the field-relative teleoperated drive command.
   *
   * <p>Behavior:
   *
   * <ul>
   *   <li>Translation input is shaped by a {@link SnapController} and rate-limited.
   *   <li>Manual rotation input has priority and disables heading hold.
   *   <li>When translating without rotation input, the heading is held by the {@link
   *       HeadingController}.
   *   <li>When the sticks are idle, the heading controller is disabled and the robot stops turning.
   * </ul>
   *
   * @param drive the drive subsystem
   * @param headingController heading-hold controller
   * @param frenteTras forward/backward axis, in [-1, 1] (forward positive)
   * @param lateral left/right axis, in [-1, 1] (left positive)
   * @param rotacao rotation axis, in [-1, 1] (counterclockwise positive)
   * @return the drive command; it never finishes on its own
   */
  public static Command movimentacaoDrive(
      Drive drive,
      HeadingController headingController,
      DoubleSupplier frenteTras,
      DoubleSupplier lateral,
      DoubleSupplier rotacao) {
    SnapController inputProcessor = new SnapController();

    TranslationSlewLimiter translationLimiter =
        new TranslationSlewLimiter(
            Constants.Swerve.ACELERACAO_MAXIMA_MPS2, Constants.Swerve.PERIODO_CONTROLE_SEGUNDOS);
    SlewRateLimiter omegaLimiter =
        new SlewRateLimiter(Constants.Swerve.ACELERACAO_ANGULAR_MAXIMA_RAD_S2);

    return Commands.run(
            () -> {

              double rawLongitudinal = frenteTras.getAsDouble();
              double rawLateral = lateral.getAsDouble();
              double rawRotation = rotacao.getAsDouble();

              Translation2d translation = inputProcessor.process(rawLongitudinal, rawLateral);
              boolean transladando = translation.getNorm() > 1e-6;

              double rotationInput =
                  MathUtil.applyDeadband(rawRotation, Constants.Swerve.DEADBAND_JOYSTICK);
              boolean giroManual = Math.abs(rotationInput) > 1e-6;

              rotationInput = Math.copySign(rotationInput * rotationInput, rotationInput);

              double targetOmega;
              if (giroManual) {

                headingController.desativar();
                targetOmega = rotationInput * drive.getVelocidadeAngularMaximaAtual();
              } else if (!transladando) {

                headingController.desativar();
                targetOmega = 0.0;
              } else {

                if (!headingController.estaAtivo()) {
                  if (Math.abs(drive.getYawVelocityRadPerSec())
                      <= Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S) {
                    headingController.segurarHeadingAtual(
                        drive.getHeading(), drive.getYawVelocityRadPerSec());
                  }
                }
                targetOmega =
                    headingController.estaAtivo()
                        ? headingController.calcular(drive.getHeading())
                        : 0.0;
              }

              Translation2d targetVelocity =
                  translation.times(drive.getVelocidadeMaximaAtual());

              Translation2d velocity = translationLimiter.calculate(
                  targetVelocity, drive.getAceleracaoMaximaAtual());

              // Clamp immediately when slow mode reduces the allowed velocity.
              double limiteVelocidade = drive.getVelocidadeMaximaAtual();
              if (velocity.getNorm() > limiteVelocidade) {
                velocity = velocity.times(limiteVelocidade / velocity.getNorm());
                translationLimiter.reset(velocity);
              }
              double vx = velocity.getX();
              double vy = velocity.getY();
              double omega = omegaLimiter.calculate(targetOmega);
              // Evita exceder o modo lento ao trocar de perfil em movimento.
              omega = MathUtil.clamp(
                  omega,
                  -drive.getVelocidadeAngularMaximaAtual(),
                  drive.getVelocidadeAngularMaximaAtual());
              // Sincroniza o estado interno do limitador depois do clamp do modo lento.
              omegaLimiter.reset(omega);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drive.getRotation()));

              headingController.atualizarDashboard(drive.getHeading(), omega);

              Rotation2d ghostHeading =
                  headingController.estaAtivo()
                      ? headingController.getHeadingAlvo()
                      : drive.getHeading();
              Logger.recordOutput(
                  "HeadingController/Ghost",
                  new Pose2d(drive.getPose().getTranslation(), ghostHeading));

              SmartDashboard.putBoolean("DriveInput/GiroManual", giroManual);
              SmartDashboard.putNumber("DriveInput/Longitudinal", translation.getX());
              SmartDashboard.putNumber("DriveInput/Lateral", translation.getY());
              SmartDashboard.putNumber("DriveInput/VelocidadeComandadaMps", velocity.getNorm());
              SmartDashboard.putBoolean("Drive/ModoLentoAtivo", drive.isModoLentoAtivo());
              SmartDashboard.putNumber("Drive/VelocidadeMaximaMps", drive.getVelocidadeMaximaAtual());
              SmartDashboard.putNumber("Drive/AceleracaoMaximaMps2", drive.getAceleracaoMaximaAtual());
              SmartDashboard.putNumber("Drive/VelocidadeAngularMaximaRadS", drive.getVelocidadeAngularMaximaAtual());
            },
            drive)
        .finallyDo(
            interrupted -> {

              headingController.desativar();
              inputProcessor.reset();
              translationLimiter.reset(Translation2d.kZero);
              omegaLimiter.reset(0.0);
              drive.stop();
            });
  }
}
