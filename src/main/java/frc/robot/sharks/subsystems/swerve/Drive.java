package frc.robot.sharks.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sharks.Constants;
import frc.robot.sharks.OffsetsConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.sharks.subsystems.swerve.gyro.GyroIO;
import frc.robot.sharks.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import frc.robot.sharks.subsystems.swerve.control.HeadingController;
import frc.robot.sharks.subsystems.swerve.control.ChassisSpeedLimiter;
import frc.robot.sharks.subsystems.swerve.gyro.GyroIOReal;
import frc.robot.sharks.subsystems.swerve.module.Module;
import frc.robot.sharks.subsystems.swerve.module.ModuleIO;
import frc.robot.sharks.subsystems.swerve.module.ModuleIOReal;
import frc.robot.sharks.subsystems.swerve.module.ModuleIOSim;
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

  private HeadingController headingControllerAutonomo = null;
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
  private void configurarAutonomo() {
    DCMotor motorTracao = DCMotor.getKrakenX60Foc(1);
    double raioRoda = Constants.Swerve.FRONT_LEFT.WheelRadius;
    double reducao = Constants.Swerve.FRONT_LEFT.DriveMotorGearRatio;

    RobotConfig config =
        new RobotConfig(
            Constants.Swerve.MASSA_COM_BATERIA_E_BUMPERS_KG,
            Constants.Swerve.MOMENTO_INERCIA_ESTIMADO_KG_M2,
            new ModuleConfig(
                raioRoda,
                motorTracao.freeSpeedRadPerSec / reducao * raioRoda,
                Constants.Autonomo.COEFICIENTE_ATRITO_RODA,
                motorTracao.withReduction(reducao),
                Constants.Swerve.LIMITE_CORRENTE_TRACAO_AMPERES,
                1),
            getModuleTranslations());

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> runVelocityAutonomo(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(Constants.Autonomo.KP_TRANSLACAO, 0.0, 0.0),
            new PIDConstants(Constants.Autonomo.KP_ROTACAO, 0.0, 0.0)),
        config,

        Drive::isRedAlliance,
        this);

    PathPlannerLogging.setLogActivePathCallback(
        path -> Logger.recordOutput("Autonomo/Trajetoria", path.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> {

          alvoHeadingAutonomo = pose.getRotation();
          Logger.recordOutput("Autonomo/PoseAlvo", pose);
        });
  }

  /**
   * Makes path following use the given {@link HeadingController} for rotation, exactly like
   * teleop does while translating. PathPlanner still provides translation; its rotation output is
   * replaced by the controller's, which tracks the path's target heading.
   *
   * @param headingController controller shared with teleop (its dashboard tuning applies here too)
   */
  public void usarHeadingControllerNoAutonomo(HeadingController headingController) {
    this.headingControllerAutonomo = headingController;
  }

  /** Clears the path heading target and releases the heading controller (call when auto ends). */
  public void limparAlvoAutonomo() {
    alvoHeadingAutonomo = null;
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

  /**
   * Output used by PathPlanner. Keeps the path's translation and replaces its rotation with the
   * {@link HeadingController} output driving the robot toward the path's target heading.
   *
   * @param speeds robot-relative speeds computed by PathPlanner
   */
  private void runVelocityAutonomo(ChassisSpeeds speeds) {
    if (headingControllerAutonomo == null || alvoHeadingAutonomo == null) {

      runVelocity(speeds);
      return;
    }

    if (!headingControllerAutonomo.estaAtivo()) {

      headingControllerAutonomo.definirHeading(
          alvoHeadingAutonomo, getHeading(), getYawVelocityRadPerSec());
    } else {
      headingControllerAutonomo.definirHeading(alvoHeadingAutonomo);
    }

    double omega = headingControllerAutonomo.calcular(getHeading());
    headingControllerAutonomo.atualizarDashboard(getHeading(), omega);

    Logger.recordOutput("Autonomo/HeadingAlvo", alvoHeadingAutonomo);
    Logger.recordOutput("Autonomo/OmegaHeadingController", omega);

    runVelocity(new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega));
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
    arena.setRobotPose(getPose());

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

    for (int amostra = 0; amostra < sampleCount; amostra++) {
      SwerveModulePosition[] posicoes = new SwerveModulePosition[modules.length];
      SwerveModulePosition[] deltas = new SwerveModulePosition[modules.length];

      for (int modulo = 0; modulo < modules.length; modulo++) {
        posicoes[modulo] = modules[modulo].getOdometryPositions()[amostra];
        deltas[modulo] =
            new SwerveModulePosition(
                posicoes[modulo].distanceMeters - lastModulePositions[modulo].distanceMeters,
                posicoes[modulo].angle);
      }

      if (gyroSamplesAvailable) {
        rawGyroRotation = gyroInputs.odometryYawPositions[amostra].plus(hardwareGyroOffset);
      } else {

        Twist2d twist = kinematics.toTwist2d(deltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(timestamps[amostra], rawGyroRotation, posicoes);

      for (int modulo = 0; modulo < modules.length; modulo++) {
        lastModulePositions[modulo] = posicoes[modulo];
      }
    }

    Logger.recordOutput("Odometry/SampleCount", sampleCount);
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
      modules[i].runSetpoint(estadosDesejados[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", estadosDesejados);
  }

  /** Stops the drivetrain (zero chassis speeds). */
  public void stop() {
    runVelocity(new ChassisSpeeds());
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
    double velocidadeConfigurada = modoLentoAtivo
        ? Constants.Swerve.VELOCIDADE_MAXIMA_MPS_LENTO
        : Constants.Swerve.VELOCIDADE_MAXIMA_MPS_RAPIDO;
    return Math.min(velocidadeConfigurada, Constants.Swerve.VELOCIDADE_MAXIMA_FISICA_ESTIMADA_MPS);
  }

  /** Velocidade angular maxima do modo atual (rad/s). */
  public double getVelocidadeAngularMaximaAtual() {
    return modoLentoAtivo
        ? Constants.Swerve.VELOCIDADE_ANGULAR_MAXIMA_RAD_S_LENTO
        : Constants.Swerve.VELOCIDADE_ANGULAR_MAXIMA_RAD_S_RAPIDO;
  }

  /** Aceleracao maxima do modo atual (m/s^2). */
  public double getAceleracaoMaximaAtual() {
    return modoLentoAtivo
        ? Constants.Swerve.ACELERACAO_MAXIMA_MPS2_LENTO
        : Constants.Swerve.ACELERACAO_MAXIMA_MPS2_RAPIDO;
  }

  /**
   * Resets the robot pose on the field (used by autonomous at the stDNJart of a path).
   *
   * @param pose the new pose
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
  }

  /**
   * Resets the pose estimator's rotation, keeping the current translation. Call it while the robot
   * faces away from the driver station: the heading becomes the alliance's "forward" (0 deg on
   * blue, 180 deg on red).
   */
  public void zerarOrientacaO() {
    poseEstimator.resetRotation(isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
  }

  /** Returns whether the robot is on the red alliance (blue when unknown). */
  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }

  /** Marks the current heading as the driver's "forward" for field-relative control. */
  public void zeroHeading() {
    driverGyroOffset = getPose().getRotation();
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
    return gyroInputs.yawVelocityRadPerSec;
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
