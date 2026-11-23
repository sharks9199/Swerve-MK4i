package frc.robot.sharks.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sharks.RobotState;
import frc.robot.sharks.RobotState.PresetElevador;
import frc.robot.sharks.RobotState.PresetIntake;
import org.littletonrobotics.junction.Logger;

/**
 * Computes and publishes articulated mechanism poses for AdvantageScope.
 *
 * <p>The central elevator carriage ({@code model_0.glb}) moves within the outer stage
 * ({@code model_1.glb}). The outer stage follows only after the carriage reaches an end stop.
 * Published transforms are relative to the original CAD export, where the central carriage
 * starts at the upper stop of the retracted outer stage.
 *
 * <p>This is visualization-only kinematics: it does not control motors, detect collisions,
 * or enforce physical robot safety limits.
 */
public final class RobotComponents {

    /** Intake pivot in robot-relative meters. */
    private static final Translation3d PIVO_INTAKE =
        new Translation3d(-0.240792, -0.334370, 0.324476);

    private static final Translation3d EIXO_FEEDER = new Translation3d();

    // Provisional visualization limits; calibrate against the measured CAD end stops.
    private static final double ELEVADOR_LIMITE_BAIXO = 0.0;
    private static final double ELEVADOR_LIMITE_CIMA = 1.6;

    /** Maximum carriage travel relative to the outer elevator stage, in meters. */
    private static final double CURSO_CENTRAL_RELATIVO = 0.65;

    /** Maximum outer stage travel relative to the chassis, in meters. */
    private static final double CURSO_EXTERNO =
        ELEVADOR_LIMITE_CIMA - CURSO_CENTRAL_RELATIVO;

    /**
     * Carriage height in the unmodified CAD export. The carriage begins at the upper stop;
     * moving it downward requires a negative displacement from the exported GLB pose.
     */
    private static final double ALTURA_CENTRAL_REFERENCIA_CAD =
        CURSO_CENTRAL_RELATIVO;

    private static final double INTAKE_LIMITE_BAIXO = Math.toRadians(-80.0);
    private static final double INTAKE_LIMITE_CIMA = 0.0;

    private static final double VELOCIDADE_ELEVADOR = 0.75;
    private static final double VELOCIDADE_INTAKE = Math.toRadians(75.0);

    private static final RobotState ESTADO = RobotState.getInstance();
    private static double ultimoTempo = Double.NaN;
    private static boolean inicializado = false;

    private RobotComponents() {}

    private static double limitar(double valor, double minimo, double maximo) {
        return Math.max(minimo, Math.min(maximo, valor));
    }

    private static double aproximar(double atual, double alvo, double passoMaximo) {
        return atual + limitar(alvo - atual, -passoMaximo, passoMaximo);
    }

    /** Initializes the mechanism state to match the original CAD export pose. */
    private static void inicializarEstado() {
        if (inicializado) {
            return;
        }

        ESTADO.atualizarPosicaoElevador(ALTURA_CENTRAL_REFERENCIA_CAD, 0.0);
        if (ESTADO.getPresetElevador() == PresetElevador.MANUAL) {
            ESTADO.definirAlvoManualElevador(ALTURA_CENTRAL_REFERENCIA_CAD);
        }
        inicializado = true;
    }

    /**
     * Publishes the four component transforms in {@code config.json} model order.
     *
     * @param deslocamentoCentral central carriage offset from the exported CAD pose, in meters
     * @param deslocamentoExterno outer elevator stage offset from the exported pose, in meters
     * @param anguloIntake intake angle, in radians
     * @param anguloFeeder feeder angle, in radians
     */
    public static void publicarPosicoes(
        double deslocamentoCentral,
        double deslocamentoExterno,
        double anguloIntake,
        double anguloFeeder
    ) {
        Pose3d elevadorCentral = new Pose3d(
            0, 0, deslocamentoCentral, new Rotation3d());
        Pose3d elevadorExterno = new Pose3d(
            0, 0, deslocamentoExterno, new Rotation3d());
        Pose3d intake = new Pose3d().rotateAround(
            PIVO_INTAKE, new Rotation3d(anguloIntake, 0, 0));
        Pose3d feeder = new Pose3d().rotateAround(
            EIXO_FEEDER, new Rotation3d(0, anguloFeeder, 0));

        Logger.recordOutput(
            "Robot/PosesDosComponentes",
            new Pose3d[] {elevadorCentral, elevadorExterno, intake, feeder});
    }

    public static void selecionarPresetElevador(PresetElevador preset) {
        inicializarEstado();
        ESTADO.selecionarPresetElevador(preset);
    }

    public static void selecionarPresetIntake(PresetIntake preset) {
        ESTADO.selecionarPresetIntake(preset);
    }

    /**
     * Advances the elevator while enforcing the mechanical stage-contact model.
     *
     * <p>With carriage height {@code h}, outer stage height {@code e}, and relative extension
     * {@code r = h - e}, the model maintains {@code 0 <= e <= CURSO_EXTERNO} and
     * {@code 0 <= r <= CURSO_CENTRAL_RELATIVO}. The outer stage moves only when the
     * carriage reaches the corresponding upper or lower end stop.
     *
     * @param passoMaximo maximum carriage displacement this cycle, in meters
     */
    private static void atualizarElevador(double passoMaximo) {
        double centralAnterior = ESTADO.getAlturaCentralMetros();
        double externa = ESTADO.getAlturaExternaMetros();
        double alvo = limitar(
            ESTADO.getAlvoElevadorMetros(),
            ELEVADOR_LIMITE_BAIXO,
            ELEVADOR_LIMITE_CIMA);

        double central = aproximar(centralAnterior, alvo, passoMaximo);

        // Move the outer stage only after the carriage contacts the corresponding end stop.
        if (central > centralAnterior) {
            externa = Math.max(externa, central - CURSO_CENTRAL_RELATIVO);
        } else if (central < centralAnterior) {
            externa = Math.min(externa, central);
        }

        externa = limitar(externa, ELEVADOR_LIMITE_BAIXO, CURSO_EXTERNO);
        central = limitar(central, externa, externa + CURSO_CENTRAL_RELATIVO);
        ESTADO.atualizarPosicaoElevador(central, externa);
    }

    private static void atualizarIntake(double passoMaximo) {
        double alvo = ESTADO.getPresetIntake() == PresetIntake.ATIVO
            ? INTAKE_LIMITE_BAIXO
            : INTAKE_LIMITE_CIMA;
        double angulo = aproximar(
            ESTADO.getAnguloIntakeRadianos(), alvo, passoMaximo);
        ESTADO.atualizarPosicaoIntake(angulo);
    }

    /**
     * Updates visualization-only mechanism targets from the Xbox driver inputs.
     * RB/LB manually move the elevator and override its preset. B/X select intake positions.
     * Releasing manual input holds the current elevator height.
     *
     * @param subir whether the Xbox right bumper is held
     * @param descer whether the Xbox left bumper is held
     * @param ativarIntake whether Xbox B is held
     * @param desligarIntake whether Xbox X is held
     */
    public static void atualizarPeloControle(
        boolean subir,
        boolean descer,
        boolean ativarIntake,
        boolean desligarIntake
    ) {
        inicializarEstado();
        double agora = Timer.getFPGATimestamp();

        // Retain the last simulated pose outside enabled teleop.
        if (!DriverStation.isTeleopEnabled()) {
            ultimoTempo = agora;
            publicarEstadoAtual();
            return;
        }

        double deltaTempo = Double.isNaN(ultimoTempo)
            ? 0.02
            : limitar(agora - ultimoTempo, 0.0, 0.05);
        ultimoTempo = agora;

        double passoElevador = VELOCIDADE_ELEVADOR * deltaTempo;
        double passoIntake = VELOCIDADE_INTAKE * deltaTempo;

        if (subir != descer) {
            double incremento = subir ? passoElevador : -passoElevador;
            double alvoManual = limitar(
                ESTADO.getAlturaCentralMetros() + incremento,
                ELEVADOR_LIMITE_BAIXO,
                ELEVADOR_LIMITE_CIMA);
            ESTADO.definirAlvoManualElevador(alvoManual);
        } else if (subir && descer) {
            ESTADO.definirAlvoManualElevador(ESTADO.getAlturaCentralMetros());
        }

        if (ativarIntake != desligarIntake) {
            ESTADO.selecionarPresetIntake(
                ativarIntake ? PresetIntake.ATIVO : PresetIntake.DESLIGADO);
        }

        atualizarElevador(passoElevador);
        atualizarIntake(passoIntake);
        publicarEstadoAtual();
    }

    /** Publishes the original CAD-relative transforms and mechanism telemetry. */
    private static void publicarEstadoAtual() {
        double central = ESTADO.getAlturaCentralMetros();
        double externa = ESTADO.getAlturaExternaMetros();

        publicarPosicoes(
            central - ALTURA_CENTRAL_REFERENCIA_CAD,
            externa,
            ESTADO.getAnguloIntakeRadianos(),
            0.0);

        Logger.recordOutput("RobotState/Elevador/AlvoMetros", ESTADO.getAlvoElevadorMetros());
        Logger.recordOutput("RobotState/Elevador/CentralMetros", central);
        Logger.recordOutput("RobotState/Elevador/ExternoMetros", externa);
        Logger.recordOutput("RobotState/Elevador/CursoRelativoMetros", central - externa);
        Logger.recordOutput("RobotState/Elevador/CentralDeslocamentoCADMetros",
            central - ALTURA_CENTRAL_REFERENCIA_CAD);
        Logger.recordOutput("RobotState/Elevador/Preset", ESTADO.getPresetElevador().name());
        Logger.recordOutput("RobotState/Intake/AlvoRadianos",
            ESTADO.getPresetIntake() == PresetIntake.ATIVO
                ? INTAKE_LIMITE_BAIXO : INTAKE_LIMITE_CIMA);
        Logger.recordOutput("RobotState/Intake/AnguloRadianos", ESTADO.getAnguloIntakeRadianos());
        Logger.recordOutput("RobotState/Intake/Preset", ESTADO.getPresetIntake().name());
    }

    // Legacy demonstration helpers must not run concurrently with atualizarPeloControle().
    public static void testarZero() {
        publicarPosicoes(0, 0, 0, 0);
    }

    public static void testarElevador() {
        double progresso = (Math.sin(Timer.getFPGATimestamp()) + 1.0) / 2.0;
        publicarPosicoes(
            CURSO_CENTRAL_RELATIVO * progresso,
            ELEVADOR_LIMITE_CIMA * progresso,
            0, 0);
    }

    public static void testarElevadorEIntake() {
        double progresso = (Math.sin(Timer.getFPGATimestamp()) + 1.0) / 2.0;
        publicarPosicoes(
            CURSO_CENTRAL_RELATIVO * progresso,
            ELEVADOR_LIMITE_CIMA * progresso,
            INTAKE_LIMITE_BAIXO * progresso,
            0);
    }

    public static void testarSequenciaElevadorOrbit() {
        double tempo = Timer.getFPGATimestamp() % 8.0;
        double central;
        double externa;

        if (tempo < 2.0) {
            double progresso = tempo / 2.0;
            central = CURSO_CENTRAL_RELATIVO * progresso;
            externa = central;
        } else if (tempo < 3.0) {
            central = CURSO_CENTRAL_RELATIVO;
            externa = CURSO_CENTRAL_RELATIVO + CURSO_EXTERNO * (tempo - 2.0);
        } else if (tempo < 4.0) {
            central = CURSO_CENTRAL_RELATIVO;
            externa = ELEVADOR_LIMITE_CIMA;
        } else if (tempo < 6.0) {
            central = CURSO_CENTRAL_RELATIVO * (1.0 - (tempo - 4.0) / 2.0);
            externa = ELEVADOR_LIMITE_CIMA;
        } else {
            central = 0.0;
            externa = ELEVADOR_LIMITE_CIMA * (1.0 - (tempo - 6.0) / 2.0);
        }
        publicarPosicoes(central, externa, 0, 0);
    }
}
