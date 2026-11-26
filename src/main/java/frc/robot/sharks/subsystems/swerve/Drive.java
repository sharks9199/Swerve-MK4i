package frc.robot.sharks.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import frc.robot.sharks.subsystems.swerve.control.DriveSafety;
import frc.robot.sharks.subsystems.swerve.control.TranslationSlewLimiter;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.sharks.Constants;
import frc.robot.sharks.OffsetsConstants;
import frc.robot.sharks.subsystems.swerve.control.ChassisSpeedLimiter;
import frc.robot.sharks.subsystems.swerve.control.DriveFeedforwardUtil;
import frc.robot.sharks.subsystems.swerve.control.DriveSensorHealth;
import frc.robot.sharks.subsystems.swerve.gyro.GyroIO;
import frc.robot.sharks.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.sharks.subsystems.swerve.gyro.GyroIOReal;
import frc.robot.sharks.subsystems.swerve.module.Module;
import frc.robot.sharks.subsystems.swerve.module.ModuleIO;
import frc.robot.sharks.subsystems.swerve.module.ModuleIOReal;
import frc.robot.sharks.subsystems.swerve.module.ModuleIOSim;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve drivetrain subsystem.
 *
 * <p>Owns the four {@link Module}s, the gyro and the pose estimator. It consumes high-frequency
 * odometry samples from {@link SwerveOdometry}, fuses vision measurements, exposes chassis-level
 * control ({@link #runVelocity}) and configures PathPlanner for autonomous path following.
 */
@SuppressWarnings("unused")
public class Drive extends SubsystemBase {

  private final Field2d arena = new Field2d();
  /** Odometry sampling frequency, in Hz (250 on CAN FD, 100 on classic CAN). */
  public static final double ODOMETRY_FREQUENCY =
      Constants.Swerve.CAN_BUS.isNetworkFD() ? 250.0 : 100.0;

  /** Guards the odometry queues shared between the main loop and the odometry thread. */
  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  /**
   * Software yaw offset. Zero by default because the Pigeon 2 already applies its mount pose; see
   * {@link OffsetsConstants.Pigeon#APLICAR_OFFSET_NO_SOFTWARE}.
   */
  private final Rotation2d hardwareGyroOffset =
      OffsetsConstants.Pigeon.APLICAR_OFFSET_NO_SOFTWARE
          ? Rotation2d.fromDegrees(OffsetsConstants.Pigeon.YAW.in(Degrees))
          : Rotation2d.kZero;
  private final Module[] modules = new Module[4];
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private final Alert gyroDisconnectedAlert =
      new Alert(
          "Gyro desconectado. O chassi está usando a cinemática como reserva.", AlertType.kError);

  private Rotation2d rawGyroRotation = new Rotation2d();
  private Rotation2d driverGyroOffset =
      Rotation2d.fromDegrees(OffsetsConstants.Driver.FRENTE_GRAUS);

  private boolean modoLentoAtivo = false;

  private Rotation2d alvoHeadingAutonomo = null;

  private Consumer<Rotation2d> headingCorrectionListener = correcao -> {};

  private SwerveModulePosition[] lastModulePositions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  // Simulated ground truth integrates physical simulated wheel velocities, never fused pose.
  private final SimGroundTruth simulatedGroundTruth = new SimGroundTruth();
  private double lastSimulationTimestamp = Double.NaN;
  private boolean criticalSensorFault = Constants.MODO_ATUAL == Constants.Mode.REAL;
  private final DriveSafety safety = new DriveSafety();
  private final Alert safetyAlert = new Alert("Swerve bloqueado: verificar falha e rearmar com BACK em teleop", AlertType.kError);
  private double lastValidGyroOdometry = Double.NEGATIVE_INFINITY;
  private double powerFactor = 1.0;
  private double tipFactor = 1.0;
  private double lastOmega = 0.0;
  private final TranslationSlewLimiter centralLimiter = new TranslationSlewLimiter(
      Constants.Swerve.ACELERACAO_MAXIMA_MPS2, Constants.Swerve.PERIODO_CONTROLE_SEGUNDOS);
  private double lastOdometryTimestamp = Double.NEGATIVE_INFINITY;
  private double firstOdometryTimestamp = Double.POSITIVE_INFINITY;


  /**
   * Creates the drive subsystem.
   *
   * @param gyroIO gyro hardware abstraction
   * @param frontLeft front-left module IO
   * @param frontRight front-right module IO
   * @param backLeft back-left module IO
   * @param backRight back-right module IO
   */
  public Drive(
      GyroIO gyroIO,
      ModuleIO frontLeft,
      ModuleIO frontRight,
      ModuleIO backLeft,
      ModuleIO backRight) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(frontLeft, 0, Constants.Swerve.FRONT_LEFT);
    modules[1] = new Module(frontRight, 1, Constants.Swerve.FRONT_RIGHT);
    modules[2] = new Module(backLeft, 2, Constants.Swerve.BACK_LEFT);
    modules[3] = new Module(backRight, 3, Constants.Swerve.BACK_RIGHT);

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    SwerveOdometry.getInstance().start();
    SmartDashboard.putData("Arena", arena);
    SmartDashboard.putData("Sharks/Swerve", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        String[] nomes = {"Front Left", "Front Right", "Back Left", "Back Right"};
        for (int i = 0; i < modules.length; i++) {
          final int index = i;
          builder.addDoubleProperty(nomes[i] + " Angle",
              () -> modules[index].getAngle().getRadians(), null);
          builder.addDoubleProperty(nomes[i] + " Velocity",
              () -> modules[index].getVelocityMetersPerSec(), null);
        }
        builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
      }
    });

    try {
      configurarAutonomo();
    } catch (Exception e) {

      DriverStation.reportError(
          "Falha ao configurar o autonomo: " + e.getMessage(), e.getStackTrace());
    }

    SmartDashboard.putNumber("Chassi/Massa kg", Constants.Swerve.MASSA_COM_BATERIA_E_BUMPERS_KG);
    SmartDashboard.putNumber(
        "Chassi/Velocidade maxima configurada mps", Constants.Swerve.VELOCIDADE_MAXIMA_MPS);
    SmartDashboard.putNumber(
        "Chassi/Velocidade angular máxima radps",
        Constants.Swerve.VELOCIDADE_ANGULAR_MAXIMA_RAD_S);
  }

  /**
   * Creates a {@link Drive} with the IO implementations matching the current runtime mode.
   *
   * @return the drive subsystem for {@code REAL}, {@code SIM} or {@code REPLAY}
   */
  public static Drive criarSwerve() {
    return switch (Constants.MODO_ATUAL) {
      case REAL ->
          new Drive(
              new GyroIOReal(),
              new ModuleIOReal(TunerConstants.FrontLeft),
              new ModuleIOReal(TunerConstants.FrontRight),
              new ModuleIOReal(TunerConstants.BackLeft),
              new ModuleIOReal(TunerConstants.BackRight));
      case SIM ->
          new Drive(
              new GyroIO() {},
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      case REPLAY ->
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    };
  }

  /** Configures PathPlanner's {@link AutoBuilder} (holonomic path follower). */
  private void configurarAutonomo() throws Exception {
    // Fonte unica dos parametros fisicos do seguidor de trajetoria.
    RobotConfig config = RobotConfig.fromGUISettings();
    verificarDimensoesNavgrid();

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> runVelocityAutonomo(speeds, feedforwards),
        new PPHolonomicDriveController(
            new PIDConstants(Constants.Autonomo.KP_TRANSLACAO, 0.0, 0.0),
            new PIDConstants(Constants.Autonomo.KP_ROTACAO, 0.0, 0.0)),
        config,

        Drive::isRedAlliance,
        this);

    PathPlannerLogging.setLogActivePathCallback(
        path -> {
          Logger.recordOutput("Autonomo/Trajetoria", path.toArray(new Pose2d[0]));
          arena.getObject("Trajetoria").setPoses(path);
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> {

          alvoHeadingAutonomo = pose.getRotation();
          Logger.recordOutput("Autonomo/PoseAlvo", pose);
          arena.getObject("Alvo").setPose(pose);
        });
  }

  /** Warns if the navigation grid and the selected AprilTag field differ materially. */
  private void verificarDimensoesNavgrid() {
    try {
      var gridPath = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath()
          .resolve("pathplanner/navgrid.json");
      String grid = java.nio.file.Files.readString(gridPath);
      var field = edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(Constants.Vision.CAMPO);
      // The navgrid is generated by PathPlanner; use the same JSON parser used by WPILib.
      var dimensions = new com.fasterxml.jackson.databind.ObjectMapper().readTree(grid).path("field_size");
      double x = dimensions.path("x").asDouble(Double.NaN);
      double y = dimensions.path("y").asDouble(Double.NaN);
      boolean compatible = Double.isFinite(x) && Double.isFinite(y)
          && Math.abs(x - field.getFieldLength()) <= 0.10
          && Math.abs(y - field.getFieldWidth()) <= 0.10;
      SmartDashboard.putBoolean("Autonomo/NavgridDimensoesCompativeis", compatible);
      if (!compatible) DriverStation.reportWarning("Navgrid e AprilTags: dimensoes diferentes; revise o mapa antes de usar pathfinding.", false);
    } catch (Exception ex) {
      SmartDashboard.putBoolean("Autonomo/NavgridDimensoesCompativeis", false);
      DriverStation.reportWarning("Nao foi possivel verificar navgrid: " + ex.getMessage(), false);
    }
  }

  /** Clears the autonomous target and Field overlays (call when auto ends). */
  public void limparAlvoAutonomo() {
    alvoHeadingAutonomo = null;
    arena.getObject("Trajetoria").setPoses(java.util.List.of());
    arena.getObject("Alvo").setPoses(java.util.List.of());
  }

  /**
   * Registers a callback that receives the heading correction caused by each vision measurement,
   * so a heading goal can be shifted consistently.
   *
   * @param listener receives the rotation correction applied to the estimated pose
   */
  public void setHeadingCorrectionListener(Consumer<Rotation2d> listener) {
    this.headingCorrectionListener = listener != null ? listener : correcao -> {};
  }

  /** Applies all three robot-relative speeds from PathPlanner, including rotation. */
  private void runVelocityAutonomo(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    // Voltage closed loop: PathPlanner wheel acceleration times characterized kA.
    // Requires SysId and disabled by default until the real drivetrain is validated.
    double[] feedforwardVolts = DriveFeedforwardUtil.calcularVolts(
        feedforwards == null ? null : feedforwards.accelerationsMPSSq(),
        Constants.Autonomo.FF_KA_VOLTS_POR_MPS2,
        Constants.Autonomo.FF_VOLTS_MAX,
        Constants.Autonomo.FF_AUTONOMO_HABILITADO);
    Logger.recordOutput("Autonomo/DriveFFVolts", feedforwardVolts);

    // Nao substituir omega: uma ordem de parada deve continuar sendo zero.
    Logger.recordOutput("Autonomo/OmegaPathPlannerRadS", speeds.omegaRadiansPerSecond);
    runVelocity(speeds, feedforwardVolts);
  }

  /** Publishes the actual PathPlanner heading target without applying teleop heading control. */
  private void atualizarDashboardHeadingAutonomo() {
    Rotation2d atual = getHeading();
    Rotation2d alvo = alvoHeadingAutonomo != null ? alvoHeadingAutonomo : atual;
    double erroRad = MathUtil.angleModulus(alvo.getRadians() - atual.getRadians());
    boolean possuiAlvo = alvoHeadingAutonomo != null;
    // Preserve the topics used by the existing Elastic layout.
    SmartDashboard.putNumber("HeadingTuning/CurrentHeadingDeg", atual.getDegrees());
    SmartDashboard.putNumber("HeadingTuning/TargetHeadingDeg", alvo.getDegrees());
    SmartDashboard.putNumber("HeadingTuning/ErrorDeg", Math.toDegrees(erroRad));
    SmartDashboard.putBoolean("HeadingTuning/Active", false);
    SmartDashboard.putBoolean("HeadingTuning/AtSetpoint", possuiAlvo
        && Math.abs(erroRad) <= Constants.Heading.TOLERANCIA_POSICAO_RAD
        && Math.abs(getYawVelocityRadPerSec()) <= Constants.Heading.TOLERANCIA_VELOCIDADE_RAD_S);
    if (possuiAlvo) {
      Logger.recordOutput("Autonomo/HeadingAlvo", alvo);
    }
  }

  @Override
  public void periodic() {

    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);

      for (Module module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    atualizarOdometria();
    if (Constants.MODO_ATUAL == Constants.Mode.SIM) {
      double timestamp = Timer.getFPGATimestamp();
      if (Double.isFinite(lastSimulationTimestamp)) {
        simulatedGroundTruth.step(getChassisSpeeds(), timestamp - lastSimulationTimestamp);
      }
      lastSimulationTimestamp = timestamp;
      Logger.recordOutput("Simulation/GroundTruthPose", simulatedGroundTruth.getPose());
    }

    boolean real = Constants.MODO_ATUAL == Constants.Mode.REAL;
    double now = Timer.getFPGATimestamp();
    double volts = RobotController.getBatteryVoltage();
    double tilt = Math.max(Math.abs(gyroInputs.pitchDegrees), Math.abs(gyroInputs.rollDegrees));
    boolean stale = real && (now - lastOdometryTimestamp > Constants.Seguranca.WATCHDOG_ODOMETRIA_S
        || now - lastValidGyroOdometry > Constants.Seguranca.WATCHDOG_ODOMETRIA_S
        || lastOdometryTimestamp > now + 0.02);
    boolean driverLost = real && DriverStation.isTeleopEnabled()
        && !DriverStation.isJoystickConnected(Constants.Controles.PORTA_DRIVER);
    boolean lowPower = real && (!Double.isFinite(volts) || volts <= Constants.Seguranca.TENSAO_PARADA_V
        || RobotController.isBrownedOut());
    boolean excessiveTilt = real && (!Double.isFinite(tilt)
        || tilt >= Constants.Seguranca.INCLINACAO_PARADA_GRAUS);
    powerFactor = real ? DriveSafety.factor(volts, Constants.Seguranca.TENSAO_REDUCAO_V,
        Constants.Seguranca.TENSAO_PARADA_V) : 1.0;
    tipFactor = real ? DriveSafety.factor(tilt, Constants.Seguranca.INCLINACAO_REDUCAO_GRAUS,
        Constants.Seguranca.INCLINACAO_PARADA_GRAUS) : 1.0;
    criticalSensorFault = real && (DriveSensorHealth.criticalFault(gyroInputs.connected, getModuleHealth())
        || !Double.isFinite(gyroInputs.yawVelocityRadPerSec) || stale || driverLost || lowPower || excessiveTilt);
    safety.update(criticalSensorFault, DriverStation.isEnabled(), now);
    if (safety.blocked() || DriverStation.isDisabled()) stopImmediately();
    safetyAlert.set(real && safety.blocked());
    Logger.recordOutput("Drive/CriticalSensorFault", criticalSensorFault);
    Logger.recordOutput("Drive/SafetyLatched", safety.latched());
    Logger.recordOutput("Drive/OdometryWatchdog", stale);
    Logger.recordOutput("Drive/DriverDisconnected", driverLost);
    Logger.recordOutput("Drive/LowBatteryFault", lowPower);
    Logger.recordOutput("Drive/ExcessiveTiltFault", excessiveTilt);
    Logger.recordOutput("Drive/PowerFactor", powerFactor);
    Logger.recordOutput("Drive/TipFactor", tipFactor);
    Logger.recordOutput("Drive/TiltDegrees", tilt);
    SmartDashboard.putBoolean("Drive/SafetyLatched", safety.latched());
    SmartDashboard.putBoolean("Drive/SafetyBlocked", safety.blocked());
    arena.setRobotPose(getPose());
    ChassisSpeeds velocidadesMedidas = getChassisSpeeds();
    SmartDashboard.putNumber("Sharks/VelocidadeRealMps",
        Math.hypot(velocidadesMedidas.vxMetersPerSecond, velocidadesMedidas.vyMetersPerSecond));
    SmartDashboard.putNumber("Sharks/HeadingGraus", getHeading().getDegrees());
    if (DriverStation.isAutonomousEnabled()) {
      atualizarDashboardHeadingAutonomo();
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.MODO_ATUAL == Constants.Mode.REAL);

    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("Drive/Diagnostic/PoseX", getPose().getX());
    Logger.recordOutput("Drive/Diagnostic/PoseY", getPose().getY());
    Logger.recordOutput("Drive/Diagnostic/PoseHeading", getPose().getRotation());
    Logger.recordOutput("Drive/Diagnostic/RawGyro", rawGyroRotation);
    Logger.recordOutput("Drive/ModoLentoAtivo", modoLentoAtivo);
    Logger.recordOutput("Drive/LimiteLinearMps", getVelocidadeMaximaAtual());
    Logger.recordOutput("Drive/LimiteAngularRadS", getVelocidadeAngularMaximaAtual());
  }

  /**
   * Updates the pose estimator with every odometry sample collected since the last cycle.
   *
   * <p>Only sample indices available for every module are used. Queues can occasionally contain
   * different counts for one cycle; an early return here used to leave the odometry partially
   * updated and could corrupt the next delta calculation.
   */
  private void atualizarOdometria() {
    double[] timestamps = modules[0].getOdometryTimestamps();
    int sampleCount = timestamps.length;
    for (int modulo = 0; modulo < modules.length; modulo++) {
      sampleCount =
          Math.min(
              sampleCount,
              Math.min(
                  modules[modulo].getOdometryTimestamps().length,
                  modules[modulo].getOdometryPositions().length));
    }

    boolean gyroSamplesAvailable =
        gyroInputs.connected
            && gyroInputs.odometryYawPositions.length >= sampleCount
            && gyroInputs.odometryYawTimestamps.length >= sampleCount;

    int rejectedSamples = 0;
    for (int amostra = 0; amostra < sampleCount; amostra++) {
      double timestamp = timestamps[amostra];
      boolean valid = Double.isFinite(timestamp) && timestamp > lastOdometryTimestamp;
      for (int modulo = 0; modulo < modules.length && valid; modulo++) {
        SwerveModulePosition pos = modules[modulo].getOdometryPositions()[amostra];
        double moduleTimestamp = modules[modulo].getOdometryTimestamps()[amostra];
        valid = pos != null && pos.angle != null && Double.isFinite(pos.distanceMeters)
            && Double.isFinite(pos.angle.getRadians()) && Double.isFinite(moduleTimestamp)
            // SIM gera os timestamps dos módulos sequencialmente no mesmo loop.
            && (Constants.MODO_ATUAL != Constants.Mode.REAL
                || Math.abs(moduleTimestamp - timestamp) <= 1e-6);
      }
      if (!valid) {
        rejectedSamples++;
        continue;
      }
      SwerveModulePosition[] posicoes = new SwerveModulePosition[modules.length];
      SwerveModulePosition[] deltas = new SwerveModulePosition[modules.length];

      for (int modulo = 0; modulo < modules.length; modulo++) {
        posicoes[modulo] = modules[modulo].getOdometryPositions()[amostra];
        deltas[modulo] =
            new SwerveModulePosition(
                posicoes[modulo].distanceMeters - lastModulePositions[modulo].distanceMeters,
                posicoes[modulo].angle);
      }

      boolean validGyroSample = gyroSamplesAvailable
          && Double.isFinite(gyroInputs.odometryYawTimestamps[amostra])
          && Math.abs(gyroInputs.odometryYawTimestamps[amostra] - timestamp) <= 1e-6
          && gyroInputs.odometryYawPositions[amostra] != null
          && Double.isFinite(gyroInputs.odometryYawPositions[amostra].getRadians());
      if (Constants.MODO_ATUAL == Constants.Mode.REAL && !validGyroSample) {
        rejectedSamples++;
        continue;
      }
      if (validGyroSample) {
        rawGyroRotation = gyroInputs.odometryYawPositions[amostra].plus(hardwareGyroOffset);
        lastValidGyroOdometry = timestamp;
      } else {

        Twist2d twist = kinematics.toTwist2d(deltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(timestamp, rawGyroRotation, posicoes);
      firstOdometryTimestamp = Math.min(firstOdometryTimestamp, timestamp);
      lastOdometryTimestamp = timestamp;

      for (int modulo = 0; modulo < modules.length; modulo++) {
        lastModulePositions[modulo] = posicoes[modulo];
      }
    }

    Logger.recordOutput("Odometry/SampleCount", sampleCount);
    Logger.recordOutput("Odometry/RejectedSamples", rejectedSamples);
    if (Constants.MODO_ATUAL == Constants.Mode.REAL) {
      Logger.recordOutput("Odometry/DroppedQueueBatches",
          SwerveOdometry.getInstance().getDroppedQueueBatches());
      Logger.recordOutput("Odometry/FailedSignalRefreshes",
          SwerveOdometry.getInstance().getFailedSignalRefreshes());
    }
    Logger.recordOutput("Odometry/GyroSamplesAvailable", gyroSamplesAvailable);
  }

  /**
   * Drives the robot at the given robot-relative chassis speeds.
   *
   * <p>Speeds are clamped to the configured limits, discretized to compensate for the control loop
   * period, converted to module states and desaturated.
   *
   * @param speeds desired robot-relative chassis speeds
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, new double[4]);
  }

  private boolean[] getModuleHealth() {
    boolean[] connected = new boolean[modules.length];
    for (int i = 0; i < modules.length; i++) connected[i] = modules[i].isHealthy();
    return connected;
  }

  /** Velocity actuation with optional validated per-module additive voltage feedforward. */
  private void runVelocity(ChassisSpeeds speeds, double[] feedforwardVolts) {
    if (isSafetyBlocked() || DriverStation.isDisabled()) {
      stopImmediately();
      Logger.recordOutput("Drive/CommandBlockedSensorFault", true);
      return;
    }
    Logger.recordOutput("Drive/CommandBlockedSensorFault", false);
    if (speeds == null || !Double.isFinite(speeds.vxMetersPerSecond)
        || !Double.isFinite(speeds.vyMetersPerSecond)
        || !Double.isFinite(speeds.omegaRadiansPerSecond)) {
      stopImmediately();
      Logger.recordOutput("Drive/CommandBlockedInvalidSpeeds", true);
      return;
    }
    Logger.recordOutput("Drive/CommandBlockedInvalidSpeeds", false);
    // Ponto central de limitacao: vale para teleop, autonomo e qualquer futuro
    // comando que use runVelocity(). Diferente do clamp por eixo, respeita
    // o modulo do vetor para evitar exceder a velocidade na diagonal.
    ChassisSpeedLimiter.Resultado limitadas =
        ChassisSpeedLimiter.limitar(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            getVelocidadeMaximaAtual(),
            getVelocidadeAngularMaximaAtual());
    ChassisSpeeds velocidadesLimitadas = new ChassisSpeeds(
        limitadas.vxMetrosSegundo(),
        limitadas.vyMetrosSegundo(),
        limitadas.omegaRadSegundo());

    // Rampa central no referencial do campo, para nao limitar artificialmente curvas.
    // Uma ordem explicita de zero e uma parada imediata, sem atraso da rampa.
    if (Math.abs(speeds.vxMetersPerSecond) < 1e-9 && Math.abs(speeds.vyMetersPerSecond) < 1e-9
        && Math.abs(speeds.omegaRadiansPerSecond) < 1e-9) {
      centralLimiter.reset(Translation2d.kZero);
      lastOmega = 0.0;
    } else {
      Translation2d fieldTarget = new Translation2d(velocidadesLimitadas.vxMetersPerSecond,
          velocidadesLimitadas.vyMetersPerSecond).rotateBy(getHeading());
      Translation2d fieldLimited = centralLimiter.calculate(fieldTarget, getAceleracaoMaximaAtual());
      if (fieldLimited.getNorm() > getVelocidadeMaximaAtual()) {
        fieldLimited = fieldLimited.times(getVelocidadeMaximaAtual() / fieldLimited.getNorm());
        centralLimiter.reset(fieldLimited);
      }
      lastOmega = MathUtil.clamp(DriveSafety.step(lastOmega, velocidadesLimitadas.omegaRadiansPerSecond,
          Constants.Swerve.ACELERACAO_ANGULAR_MAXIMA_RAD_S2 * Math.min(powerFactor, tipFactor)
              * Constants.Swerve.PERIODO_CONTROLE_SEGUNDOS),
          -getVelocidadeAngularMaximaAtual(), getVelocidadeAngularMaximaAtual());
      velocidadesLimitadas = ChassisSpeeds.fromFieldRelativeSpeeds(
          fieldLimited.getX(), fieldLimited.getY(), lastOmega, getHeading());
    }
    ChassisSpeeds velocidadesDiscretas =
        ChassisSpeeds.discretize(velocidadesLimitadas, Constants.Swerve.PERIODO_CONTROLE_SEGUNDOS);

    SwerveModuleState[] estadosDesejados;
    if (Math.abs(velocidadesDiscretas.vxMetersPerSecond) < 1e-9
        && Math.abs(velocidadesDiscretas.vyMetersPerSecond) < 1e-9
        && Math.abs(velocidadesDiscretas.omegaRadiansPerSecond) < 1e-9) {

      SwerveModuleState[] estadosAtuais = getModuleStates();
      estadosDesejados = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        estadosDesejados[i] = new SwerveModuleState(0.0, estadosAtuais[i].angle);
      }
    } else {
      estadosDesejados = kinematics.toSwerveModuleStates(velocidadesDiscretas);
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
        estadosDesejados, getVelocidadeMaximaAtual());

    Logger.recordOutput("SwerveStates/Setpoints", estadosDesejados);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", velocidadesDiscretas);

    for (int i = 0; i < modules.length; i++) {
      double ffVolts = feedforwardVolts != null && feedforwardVolts.length > i
          ? feedforwardVolts[i] : 0.0;
      // Never add motor force when the desired module speed is zero.
      if (Math.abs(estadosDesejados[i].speedMetersPerSecond) < 1e-9) ffVolts = 0.0;
      modules[i].runSetpoint(estadosDesejados[i], Double.isFinite(ffVolts) ? ffVolts : 0.0);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", estadosDesejados);
  }

  /** Stops the drivetrain (zero chassis speeds). */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  private void stopImmediately() {
    centralLimiter.reset(Translation2d.kZero);
    lastOmega = 0.0;
    for (Module module : modules) module.stop();
  }

  public boolean isSafetyBlocked() {
    return Constants.MODO_ATUAL == Constants.Mode.REAL && safety.blocked();
  }

  public void rearmSafety(boolean neutral) {
    if (DriverStation.isTeleopEnabled()) safety.rearm(Timer.getFPGATimestamp(),
        Constants.Seguranca.TEMPO_RECUPERACAO_S, neutral);
  }

  /** Trava rodas em X; apenas enquanto o comando tem o requisito Drive. */
  public void xLock() {
    if (isSafetyBlocked() || !DriverStation.isTeleopEnabled()) { stopImmediately(); return; }
    centralLimiter.reset(Translation2d.kZero);
    lastOmega = 0.0;
    Translation2d[] locations = getModuleTranslations();
    for (int i = 0; i < modules.length; i++) {
      modules[i].runSetpoint(new SwerveModuleState(0.0, locations[i].getAngle()));
    }
  }

  private java.util.function.BooleanSupplier sysIdDeadman = () -> false;
  public void setSysIdDeadman(java.util.function.BooleanSupplier deadman) { sysIdDeadman = deadman; }

  private boolean sysIdAllowed() {
    return Constants.Seguranca.SYSID_HABILITADO && DriverStation.isTestEnabled()
        && sysIdDeadman.getAsBoolean() && !isSafetyBlocked() && powerFactor >= 0.99 && tipFactor >= 0.99;
  }

  private void characterize(double volts) {
    if (!sysIdAllowed()) { stopImmediately(); return; }
    boolean allAligned = java.util.Arrays.stream(modules).allMatch(Module::isAlignedForSysId);
    for (Module module : modules) module.characterize(allAligned ? volts : 0.0);
  }

  public Command sysId(boolean dynamic, SysIdRoutine.Direction direction) {
    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(3.0), Seconds.of(5.0)),
        new SysIdRoutine.Mechanism(voltage -> characterize(voltage.in(Volts)),
            log -> { for (int i = 0; i < modules.length; i++) modules[i].logSysId(log, "module" + i); },
            this, "SharksSwerve"));
    Command run = dynamic ? routine.dynamic(direction) : routine.quasistatic(direction);
    return Commands.run(() -> characterize(0.0), this).withTimeout(1.0)
        .andThen(run.onlyIf(() -> java.util.Arrays.stream(modules).allMatch(Module::isAlignedForSysId)))
        .until(() -> !sysIdAllowed())
        .onlyIf(this::sysIdAllowed).finallyDo(interrupted -> stopImmediately());
  }

  /** Alterna o modo de conducao a cada toque no botao A. */
  public void alternarModoLento() {
    modoLentoAtivo = !modoLentoAtivo;
    Logger.recordOutput("Drive/ModoLentoAtivo", modoLentoAtivo);
  }

  /** Retorna se o modo lento esta ativo. */
  public boolean isModoLentoAtivo() {
    return modoLentoAtivo;
  }

  /** Velocidade maxima do modo atual (m/s). */
  public double getVelocidadeMaximaAtual() {
    double velocidadeConfigurada = modoLentoAtivo && !DriverStation.isAutonomousEnabled()
        ? Constants.Swerve.VELOCIDADE_MAXIMA_MPS_LENTO
        : Constants.Swerve.VELOCIDADE_MAXIMA_MPS_RAPIDO;
    return Math.min(velocidadeConfigurada, Constants.Swerve.VELOCIDADE_MAXIMA_FISICA_ESTIMADA_MPS)
        * Math.min(powerFactor, tipFactor);
  }

  /** Velocidade angular maxima do modo atual (rad/s). */
  public double getVelocidadeAngularMaximaAtual() {
    return (modoLentoAtivo && !DriverStation.isAutonomousEnabled()
        ? Constants.Swerve.VELOCIDADE_ANGULAR_MAXIMA_RAD_S_LENTO
        : Constants.Swerve.VELOCIDADE_ANGULAR_MAXIMA_RAD_S_RAPIDO) * Math.min(powerFactor, tipFactor);
  }

  /** Aceleracao maxima do modo atual (m/s^2). */
  public double getAceleracaoMaximaAtual() {
    return (modoLentoAtivo && !DriverStation.isAutonomousEnabled()
        ? Constants.Swerve.ACELERACAO_MAXIMA_MPS2_LENTO
        : Constants.Swerve.ACELERACAO_MAXIMA_MPS2_RAPIDO) * Math.min(powerFactor, tipFactor);
  }

  /**
   * Resets the robot pose on the field (used by autonomous at the stDNJart of a path).
   *
   * @param pose the new pose
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
    firstOdometryTimestamp = Double.POSITIVE_INFINITY;
    if (Constants.MODO_ATUAL == Constants.Mode.SIM) {
      simulatedGroundTruth.reset(pose);
      lastSimulationTimestamp = Double.NaN;
    }
  }

  /** Independent pose source for simulated camera, never fused with vision estimates. */
  public Pose2d getSimulatedGroundTruthPose() {
    if (Constants.MODO_ATUAL != Constants.Mode.SIM) {
      throw new IllegalStateException("Pose de simulacao solicitada fora do modo SIM");
    }
    return simulatedGroundTruth.getPose();
  }

  /**
   * Resets only the driver's field-relative reference, preserving the estimated field pose.
   * Call while the robot faces the desired joystick-forward direction.
   */
  public void zerarOrientacaO() {
    zeroHeading();
  }

  /** Returns whether the robot is on the red alliance (blue when unknown). */
  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }

  /** Marks the current heading as the driver's "forward" for field-relative control. */
  public void zeroHeading() {
    driverGyroOffset = getPose().getRotation().plus(isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param poseVisao pose measured by vision
   * @param timestampSegundos capture timestamp, in seconds (FPGA time)
   * @param desvioPadraoXMetros X standard deviation, in meters
   * @param desvioPadraoYMetros Y standard deviation, in meters
   * @param desvioPadraoThetaRad heading standard deviation, in radians
   * @return the heading correction the measurement caused in the estimated pose
   */
  public Rotation2d adicionarMedicaoVisao(
      Pose2d poseVisao,
      double timestampSegundos,
      double desvioPadraoXMetros,
      double desvioPadraoYMetros,
      double desvioPadraoThetaRad) {
    Rotation2d headingAntes = getPose().getRotation();

    poseEstimator.addVisionMeasurement(
        poseVisao,
        timestampSegundos,
        VecBuilder.fill(desvioPadraoXMetros, desvioPadraoYMetros, desvioPadraoThetaRad));

    Rotation2d headingDepois = getPose().getRotation();
    Rotation2d correcaoHeading = headingDepois.minus(headingAntes);

    Logger.recordOutput("Odometry/VisionPose", poseVisao);
    Logger.recordOutput("Odometry/VisionHeadingCorrectionDeg", correcaoHeading.getDegrees());

    headingCorrectionListener.accept(correcaoHeading);

    return correcaoHeading;
  }

  /** Returns the estimated pose at the vision frame's capture time. */
  public Optional<Pose2d> getPoseAtTimestamp(double timestampSeconds) {
    if (!Double.isFinite(timestampSeconds) || timestampSeconds < firstOdometryTimestamp
        || timestampSeconds > lastOdometryTimestamp || lastOdometryTimestamp - timestampSeconds > 0.50) {
      return Optional.empty();
    }
    return poseEstimator.sampleAt(timestampSeconds);
  }

  /** Returns the estimated robot pose on the field. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the driver-relative robot rotation, used for field-relative teleop: estimated heading
   * minus the driver offset, flipped by 180 degrees on the red alliance so that "forward" on the
   * joystick always points away from the driver.
   */
  public Rotation2d getRotation() {
    Rotation2d rotation = getPose().getRotation().minus(driverGyroOffset);
    return isRedAlliance() ? rotation.plus(Rotation2d.k180deg) : rotation;
  }

  /** Returns the estimated robot heading on the field. */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /** Returns the raw gyro rotation used by the pose estimator. */
  public Rotation2d getRawGyroRotation() {
    return rawGyroRotation;
  }

  /** Returns the yaw rate measured by the gyro, in rad/s. */
  public double getYawVelocityRadPerSec() {
    return gyroInputs.connected ? gyroInputs.yawVelocityRadPerSec
        : getChassisSpeeds().omegaRadiansPerSecond;
  }

  /** Returns the measured robot-relative chassis speeds. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the measured state (speed and angle) of each module. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] estados = new SwerveModuleState[4];
    for (int i = 0; i < modules.length; i++) {
      estados[i] = modules[i].getState();
    }
    return estados;
  }

  /** Returns the position of each module relative to the robot center (FL, FR, BL, BR). */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(
          Constants.Swerve.FRONT_LEFT.LocationX, Constants.Swerve.FRONT_LEFT.LocationY),
      new Translation2d(
          Constants.Swerve.FRONT_RIGHT.LocationX, Constants.Swerve.FRONT_RIGHT.LocationY),
      new Translation2d(
          Constants.Swerve.BACK_LEFT.LocationX, Constants.Swerve.BACK_LEFT.LocationY),
      new Translation2d(
          Constants.Swerve.BACK_RIGHT.LocationX, Constants.Swerve.BACK_RIGHT.LocationY)
    };
  }
}
