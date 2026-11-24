package frc.robot.sharks.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sharks.Constants;

/** Limelight NetworkTables adapter. Requires Limelight OS 2026+ for {@code botpose_orb_wpiblue}. */
public class VisionIOReal implements VisionIO {
  private final NetworkTable table =
      NetworkTableInstance.getDefault().getTable(Constants.Vision.NOME_LIMELIGHT);
  private final NetworkTableEntry orientation = table.getEntry("robot_orientation_set");
  private final DoubleArraySubscriber megaTag1Entry =
      table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
  private final DoubleArraySubscriber megaTag2Entry =
      table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
  private final NetworkTableEntry heartbeat = table.getEntry("hb");
  private double ultimaTrocaPipeline = 0.0;

  @Override
  public void definirPipeline(int pipeline) {
    ultimaTrocaPipeline = Timer.getFPGATimestamp();
    table.getEntry("pipeline").setDouble(pipeline);
  }

  @Override
  public void definirOrientacao(double yawGraus, double yawRateGrausPorSegundo) {

    orientation.setDoubleArray(new double[] {yawGraus, yawRateGrausPorSegundo, 0, 0, 0, 0});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.tx = table.getEntry("tx").getDouble(0.0);
    inputs.ty = table.getEntry("ty").getDouble(0.0);
    double now = Timer.getFPGATimestamp();
    double frameTime = heartbeat.getLastChange() / 1_000_000.0;
    boolean frameRecente = frameTime > 0.0 && frameTime > ultimaTrocaPipeline
        && frameTime <= now + 0.10 && now - frameTime <= 0.50;
    double pipeline = table.getEntry("getpipe").getDouble(-1.0);
    inputs.pipelineAtual = Double.isFinite(pipeline) && pipeline == Math.rint(pipeline)
        && pipeline >= 0 && pipeline <= 9 ? (int) pipeline : -1;
    inputs.temAlvo = frameRecente && table.getEntry("tv").getDouble(0.0) >= 1.0;
    inputs.temAlvoGamepiece = inputs.temAlvo && inputs.pipelineAtual == 1;

    // Leitura atômica: valores e timestamp pertencem à mesma publicação NT.
    var mt1 = megaTag1Entry.getAtomic();
    var mt2 = megaTag2Entry.getAtomic();
    preencherPose(mt1.value, mt1.timestamp, false, inputs);
    preencherPose(mt2.value, mt2.timestamp, true, inputs);

    double[] fiducials = table.getEntry("rawfiducials").getDoubleArray(new double[0]);
    inputs.aprilTagID = -1;
    for (int i = 0; i + 6 < fiducials.length; i += 7) {
      int id = (int) Math.round(fiducials[i]);
      if (inputs.aprilTagID < 0) inputs.aprilTagID = id;
    }
    if (!frameRecente || inputs.pipelineAtual != 0) {
      inputs.aprilTagID = -1;
    } else {
      inputs.temAlvo = inputs.temAlvo || fiducials.length >= 7;
    }
  }

  /**
   * Parses a Limelight pose array into the MegaTag 1 or MegaTag 2 fields of the inputs.
   *
   * @param dados raw array from NetworkTables
   * @param atualizadoEmMicros NetworkTables last-update timestamp in microseconds
   * @param megaTag2 {@code true} to fill the MegaTag 2 fields, {@code false} for MegaTag 1
   * @param inputs the inputs object to populate
   */
  private void preencherPose(
      double[] dados, long atualizadoEmMicros, boolean megaTag2, VisionIOInputs inputs) {
    // Reject data that never updated, arrived from the future, or is too old.
    double timestamp = VisionTimestampUtil.timestampCaptura(
        atualizadoEmMicros, dados.length > 6 ? dados[6] : Double.NaN,
        Timer.getFPGATimestamp(), 0.50);
    if (dados.length < 11 || timestamp <= 0.0 || timestamp <= ultimaTrocaPipeline
        || inputs.pipelineAtual != 0 || !Double.isFinite(dados[0])
        || !Double.isFinite(dados[1]) || !Double.isFinite(dados[5])
        || !Double.isFinite(dados[7]) || dados[7] < 1.0
        || dados[7] > Integer.MAX_VALUE || dados[7] != Math.rint(dados[7])) {
      if (megaTag2) {
        inputs.quantidadeTagsMegaTag2 = 0;
        inputs.distanciaMediaTagsMegaTag2 = 0.0;
        inputs.timestampMegaTag2 = 0.0;
      } else {
        inputs.quantidadeTagsMegaTag1 = 0;
        inputs.distanciaMediaTagsMegaTag1 = 0.0;
        inputs.timestampMegaTag1 = 0.0;
      }
      return;
    }
    Pose2d pose = new Pose2d(dados[0], dados[1], Rotation2d.fromDegrees(dados[5]));
    int count = (int) Math.round(dados[7]);
    double distance = dados[9];
    // Os raw fiducials anexados a cada botpose pertencem ao próprio frame.
    double ambiguity = 1.0;
    if (dados.length == 11L + 7L * count) {
      for (int i = 17; i < dados.length; i += 7) {
        if (Double.isFinite(dados[i]) && dados[i] >= 0.0) {
          ambiguity = Math.min(ambiguity, dados[i]);
        }
      }
    }
    if (megaTag2) {
      inputs.poseMegaTag2 = pose;
      inputs.timestampMegaTag2 = timestamp;
      inputs.quantidadeTagsMegaTag2 = count;
      inputs.distanciaMediaTagsMegaTag2 = distance;
      inputs.ambiguidadeMinimaMegaTag2 = ambiguity;
    } else {
      inputs.poseMegaTag1 = pose;
      inputs.timestampMegaTag1 = timestamp;
      inputs.quantidadeTagsMegaTag1 = count;
      inputs.distanciaMediaTagsMegaTag1 = distance;
      inputs.ambiguidadeMinimaMegaTag1 = ambiguity;
    }
  }
}
