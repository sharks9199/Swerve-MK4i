
package frc.robot.sharks.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.sharks.Constants;
import frc.robot.sharks.subsystems.swerve.Drive;

import org.littletonrobotics.junction.Logger;

/** Reads AprilTag poses and fuses validated vision measurements into Drive odometry. */
public class Vision extends SubsystemBase {

  /** Limelight pipeline index configured for AprilTag detection. */
  private static final int PIPELINE_APRILTAG = 0;

  /** Limelight pipeline index configured for Game Piece detection. */
  private static final int PIPELINE_GAMEPIECE = 1;

  /** Limelight NetworkTables name. */
  private static final String LIMELIGHT_NAME = "limelight";

  /** Minimum number of tags required to accept a measurement. */
  private static final int TAGS_MINIMAS = 1;

  private final Drive drive;
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  private boolean trackingGamepiece = false;

  /** Timestamp of the last accepted measurement; prevents fusing the same frame twice. */
  private double ultimoTimestampAceito = -1.0;

  /**
   * Creates the vision subsystem and selects the AprilTag pipeline.
   *
   * @param drive the drive subsystem that receives vision measurements
   * @param io camera hardware abstraction
   */
  public Vision(Drive drive, VisionIO io) {
    this.drive = drive;
    this.io = io;
    io.definirPipeline(PIPELINE_APRILTAG);
  }

  @Override
  public void periodic() {

    io.definirOrientacao(
        drive.getHeading().getDegrees(), Math.toDegrees(drive.getYawVelocityRadPerSec()));
    io.updateInputs(inputs);
    Logger.processInputs("Vision/Limelight", inputs);

    // Game Piece tracking telemetry.
    Logger.recordOutput("Vision/Gamepiece/Tracking", trackingGamepiece);
    Logger.recordOutput("Vision/Gamepiece/HasTarget", hasTarget());
    Logger.recordOutput("Vision/Gamepiece/Tx", getTx());

    // Prevents AprilTag pose fusion during Game Piece tracking
    // or while the camera is switching back to the AprilTag pipeline.
    if (trackingGamepiece
        || LimelightHelpers.getCurrentPipelineIndex(LIMELIGHT_NAME)
            != PIPELINE_APRILTAG) {
      Logger.recordOutput("Vision/Accepted", false);
      Logger.recordOutput("Vision/AcceptedSource", "None");
      return;
    }

    Logger.recordOutput("Vision/AprilTag/Visible", inputs.temAlvo);
    Logger.recordOutput("Vision/AprilTag/ID", inputs.aprilTagID);
    Logger.recordOutput("Vision/MegaTag1/TagCount", inputs.quantidadeTagsMegaTag1);
    Logger.recordOutput("Vision/MegaTag2/TagCount", inputs.quantidadeTagsMegaTag2);
    Logger.recordOutput("Vision/MegaTag2/DistanceMeters", inputs.distanciaMediaTagsMegaTag2);

    if (medicaoValida(
        inputs.poseMegaTag2,
        inputs.timestampMegaTag2,
        inputs.quantidadeTagsMegaTag2,
        inputs.distanciaMediaTagsMegaTag2,
        inputs.ambiguidadeMinimaMegaTag2)) {
      int tags = inputs.quantidadeTagsMegaTag2;
      double sigmaXY =
          tags >= 2
              ? Constants.Vision.MT2_DESVIO_XY_MULTIPLAS_TAGS_METROS
              : Constants.Vision.MT2_DESVIO_XY_TAG_UNICA_METROS;
      drive.adicionarMedicaoVisao(
          inputs.poseMegaTag2,
          inputs.timestampMegaTag2,
          sigmaXY,
          sigmaXY,
          Constants.Vision.MT2_DESVIO_THETA_RAD);
      ultimoTimestampAceito = inputs.timestampMegaTag2;
      Logger.recordOutput("Vision/Accepted", true);
      Logger.recordOutput("Vision/AcceptedSource", "MegaTag2");
      return;
    }

    if (medicaoValida(
        inputs.poseMegaTag1,
        inputs.timestampMegaTag1,
        inputs.quantidadeTagsMegaTag1,
        inputs.distanciaMediaTagsMegaTag1,
        inputs.ambiguidadeMinimaMegaTag1)) {
      int tags = inputs.quantidadeTagsMegaTag1;
      double sigmaXY =
          tags >= 2
              ? Constants.Vision.MT1_DESVIO_XY_MULTIPLAS_TAGS_METROS
              : Constants.Vision.MT1_DESVIO_XY_TAG_UNICA_METROS;
      double sigmaTheta =
          tags >= 2
              ? Constants.Vision.MT1_DESVIO_THETA_MULTIPLAS_TAGS_RAD
              : Constants.Vision.MT1_DESVIO_THETA_TAG_UNICA_RAD;
      drive.adicionarMedicaoVisao(
          inputs.poseMegaTag1, inputs.timestampMegaTag1, sigmaXY, sigmaXY, sigmaTheta);
      ultimoTimestampAceito = inputs.timestampMegaTag1;
      Logger.recordOutput("Vision/Accepted", true);
      Logger.recordOutput("Vision/AcceptedSource", "MegaTag1");
    } else {
      Logger.recordOutput("Vision/Accepted", false);
      Logger.recordOutput("Vision/AcceptedSource", "None");
    }
  }

  /**
   * Checks whether a vision measurement is safe to fuse into odometry.
   *
   * @param pose measured robot pose
   * @param timestamp capture timestamp, in seconds
   * @param tags number of tags used
   * @param distancia average distance to the tags, in meters
   * @param ambiguidade pose ambiguity of this specific measurement source
   * @return {@code true} if the measurement should be accepted
   */
  private boolean medicaoValida(
      Pose2d pose, double timestamp, int tags, double distancia, double ambiguidade) {
    if (tags < TAGS_MINIMAS
        || !Double.isFinite(timestamp)
        || timestamp <= 0.0
        || timestamp <= ultimoTimestampAceito
        || !Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getRadians())
        || !Double.isFinite(ambiguidade)
        || !Double.isFinite(distancia)
        || distancia <= 0.0
        || distancia > Constants.Vision.DISTANCIA_MAXIMA_VISAO_METROS) {
      return false;
    }

    if (Math.abs(drive.getYawVelocityRadPerSec())
        > Constants.Vision.VELOCIDADE_ANGULAR_MAXIMA_PARA_VISAO_RAD_S) {
      return false;
    }

    double erro = pose.getTranslation().getDistance(drive.getPose().getTranslation());
    double tolerancia = Math.max(1.0, distancia * (tags >= 2 ? 1.5 : 1.0));
    if (erro > tolerancia) {
      return false;
    }

    if (tags == 1
        && ambiguidade > Constants.Vision.AMBIGUIDADE_MAXIMA_TAG_UNICA_HEADING
        && ambiguidade > 0.0) {
      return false;
    }

    return !DriverStation.isEStopped();
  }

  // ---------------------------------------------------------------------------
  // Game Piece Tracking
  // ---------------------------------------------------------------------------

  /** Returns the horizontal target offset, in degrees. */
  public double getTx() {
    return LimelightHelpers.getTX(LIMELIGHT_NAME);
  }

  /** Returns whether a valid Game Piece target is detected. */
  public boolean hasTarget() {
    return trackingGamepiece
        && LimelightHelpers.getCurrentPipelineIndex(LIMELIGHT_NAME)
            == PIPELINE_GAMEPIECE
        && LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  /** Enables Game Piece detection. */
  public void enableGamepieceTracking() {
    trackingGamepiece = true;
    io.definirPipeline(PIPELINE_GAMEPIECE);
  }

  /** Restores AprilTag detection. */
  public void disableGamepieceTracking() {
    trackingGamepiece = false;
    io.definirPipeline(PIPELINE_APRILTAG);
  }
}
