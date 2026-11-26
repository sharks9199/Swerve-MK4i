package frc.robot.sharks.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sharks.Constants;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.Random;
import java.util.function.Supplier;

/**
 * Simulated Limelight.
 *
 * <p>Given the robot's "true" pose, it determines which AprilTags the camera would see (range,
 * field of view and tag facing direction) and produces noisy MegaTag 1 / MegaTag 2 poses, at a
 * limited frame rate and with latency, just like {@link VisionIOReal} would deliver.
 *
 * <p>Limitations: tags are never occluded by other field elements, and the model is 2D (camera
 * height, pitch and tag height are ignored).
 */
public class VisionIOSim implements VisionIO {

  /** Camera pose relative to the robot center (default: 0.3 m forward, facing forward). */
  private static final Transform2d ROBOT_TO_CAMERA =
      new Transform2d(new Translation2d(0.3, 0.0), Rotation2d.kZero);

  /** Horizontal field of view, in degrees. */
  private static final double HORIZONTAL_FOV_DEG = 75.0;

  /** Tags viewed at more than this angle off their normal are considered unreadable, in degrees. */
  private static final double MAX_TAG_VIEW_ANGLE_DEG = 70.0;

  /** Camera frame rate, in Hz. */
  private static final double FRAME_RATE_HZ = 30.0;

  /** Pipeline plus capture latency, in seconds. */
  private static final double LATENCY_SECONDS = 0.030;

  /** Position noise standard deviation per meter of tag distance (MegaTag 2). */
  private static final double MT2_XY_NOISE_PER_METER = 0.004;

  /** Position noise standard deviation per meter of tag distance (MegaTag 1). */
  private static final double MT1_XY_NOISE_PER_METER = 0.005;

  /** Heading noise standard deviation of MegaTag 1, in degrees. */
  private static final double MT1_HEADING_NOISE_DEG = 1.0;

  /** Ambiguity reported for a single, well-seen tag. */
  private static final double SINGLE_TAG_AMBIGUITY = 0.007;

  private final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(Constants.Vision.CAMPO);
  private final Supplier<Pose2d> truePoseSupplier;
  private final Random random = new Random(0);

  private record Frame(double timestamp, Pose2d pose) {}
  private final Queue<Frame> pendingFrames = new ArrayDeque<>();
  private double nextFrameTime = -1.0;
  private double lastUpdateTime = -1.0;
  private boolean pipelineChanged = true;

  private int pipeline = 0;
  @SuppressWarnings("unused")
  private double robotYawDeg = 0.0;
  @SuppressWarnings("unused")
  private double robotYawRateDegPerSec = 0.0;

  /**
   * Creates a simulated camera.
   *
   * @param truePoseSupplier supplies the robot's real pose in the simulated world, e.g. {@code
   *     drive::getPose}
   */
  public VisionIOSim(Supplier<Pose2d> truePoseSupplier) {
    this.truePoseSupplier = truePoseSupplier;
  }

  @Override
  public void definirPipeline(int pipeline) {
    if (this.pipeline != pipeline) {
      this.pipeline = pipeline;
      pipelineChanged = true;
    }
  }

  @Override
  public void definirOrientacao(double yawGraus, double yawRateGrausPorSegundo) {
    robotYawDeg = yawGraus;
    robotYawRateDegPerSec = yawRateGrausPorSegundo;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double now = Timer.getFPGATimestamp();

    if (pipelineChanged || now < lastUpdateTime) {
      pendingFrames.clear();
      nextFrameTime = now;
      inputs.temAlvo = false;
      inputs.aprilTagID = -1;
      inputs.tx = 0.0;
      inputs.ty = 0.0;
      limparMedicoes(inputs);
      pipelineChanged = false;
    }
    lastUpdateTime = now;
    inputs.pipelineAtual = pipeline;
    // Ainda não há modelo de detecção de peças na simulação.
    inputs.temAlvoGamepiece = false;
    if (pipeline != 0) return;

    // Captura a pose agora e entrega o frame apenas após a latência simulada.
    // Mantém a cadência média sem inventar capturas entre ciclos do robô.
    if (now + 1e-9 >= nextFrameTime) {
      pendingFrames.add(new Frame(now, truePoseSupplier.get()));
      nextFrameTime += (Math.floor(Math.max(0.0, now - nextFrameTime) * FRAME_RATE_HZ) + 1)
          / FRAME_RATE_HZ;
    }
    Frame frame = null;
    while (!pendingFrames.isEmpty()
        && now - pendingFrames.peek().timestamp() >= LATENCY_SECONDS) {
      frame = pendingFrames.remove();
    }
    if (frame == null) return;

    Pose2d robotPose = frame.pose();
    Pose2d cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

    int visibleCount = 0;
    double distanceSum = 0.0;
    double closestDistance = Double.POSITIVE_INFINITY;
    int closestId = -1;
    double closestTxDeg = 0.0;

    for (var tag : layout.getTags()) {
      Pose2d tagPose = tag.pose.toPose2d();
      Translation2d cameraToTag = tagPose.getTranslation().minus(cameraPose.getTranslation());
      double distance = cameraToTag.getNorm();
      if (distance > Constants.Vision.DISTANCIA_MAXIMA_VISAO_METROS || distance < 1e-6) {
        continue;
      }

      Rotation2d bearing = cameraToTag.getAngle().minus(cameraPose.getRotation());
      if (Math.abs(bearing.getDegrees()) > HORIZONTAL_FOV_DEG / 2.0) {
        continue;
      }

      Rotation2d viewAngle = cameraToTag.unaryMinus().getAngle().minus(tagPose.getRotation());
      if (Math.abs(viewAngle.getDegrees()) > MAX_TAG_VIEW_ANGLE_DEG) {
        continue;
      }

      visibleCount++;
      distanceSum += distance;
      if (distance < closestDistance) {
        closestDistance = distance;
        closestId = tag.ID;
        closestTxDeg = -bearing.getDegrees();
      }
    }

    inputs.temAlvo = visibleCount > 0;
    inputs.aprilTagID = closestId;
    inputs.tx = visibleCount > 0 ? closestTxDeg : 0.0;
    inputs.ty = 0.0;

    if (visibleCount == 0) {
      limparMedicoes(inputs);
      return;
    }

    double averageDistance = distanceSum / visibleCount;
    double ambiguity = visibleCount >= 2 ? 0.0 : SINGLE_TAG_AMBIGUITY;
    double timestamp = frame.timestamp();

    double mt2Noise = MT2_XY_NOISE_PER_METER * averageDistance;
    inputs.poseMegaTag2 =
        new Pose2d(
            robotPose.getX() + random.nextGaussian() * mt2Noise,
            robotPose.getY() + random.nextGaussian() * mt2Noise,
            robotPose.getRotation());
    inputs.timestampMegaTag2 = timestamp;
    inputs.quantidadeTagsMegaTag2 = visibleCount;
    inputs.distanciaMediaTagsMegaTag2 = averageDistance;
    inputs.ambiguidadeMinimaMegaTag2 = ambiguity;

    double mt1Noise = MT1_XY_NOISE_PER_METER * averageDistance;
    inputs.poseMegaTag1 =
        new Pose2d(
            robotPose.getX() + random.nextGaussian() * mt1Noise,
            robotPose.getY() + random.nextGaussian() * mt1Noise,
            robotPose
                .getRotation()
                .plus(Rotation2d.fromDegrees(random.nextGaussian() * MT1_HEADING_NOISE_DEG)));
    inputs.timestampMegaTag1 = timestamp;
    inputs.quantidadeTagsMegaTag1 = visibleCount;
    inputs.distanciaMediaTagsMegaTag1 = averageDistance;
    inputs.ambiguidadeMinimaMegaTag1 = ambiguity;
  }

  private void limparMedicoes(VisionIOInputs inputs) {
    inputs.quantidadeTagsMegaTag1 = 0;
    inputs.quantidadeTagsMegaTag2 = 0;
    inputs.distanciaMediaTagsMegaTag1 = 0.0;
    inputs.distanciaMediaTagsMegaTag2 = 0.0;
    inputs.timestampMegaTag1 = 0.0;
    inputs.timestampMegaTag2 = 0.0;
    inputs.ambiguidadeMinimaMegaTag1 = 1.0;
    inputs.ambiguidadeMinimaMegaTag2 = 1.0;
  }
}

