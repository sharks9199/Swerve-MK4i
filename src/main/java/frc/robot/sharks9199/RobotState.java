package frc.robot.sharks9199;

/**
 * Shared state for AdvantageScope mechanism visualization.
 *
 * <p>These values represent commanded targets and simulated positions, not measured hardware
 * states. Updates currently occur sequentially on the robot loop.
 */
public final class RobotState {

    /**
     * Elevator target positions in meters, relative to the fully retracted position.
     * L1-L4 are provisional visualization heights and require CAD calibration.
     */
    public enum PresetElevador {
        MANUAL(Double.NaN),
        L1(0.0),
        L2(0.4),
        L3(0.8),
        L4(1.6);

        private final double alturaMetros;

        PresetElevador(double alturaMetros) {
            this.alturaMetros = alturaMetros;
        }

        /** Returns the target height in meters; MANUAL has no fixed height. */
        public double getAlturaMetros() {
            return alturaMetros;
        }
    }

    public enum PresetIntake {
        ATIVO,
        DESLIGADO
    }

    private static final RobotState INSTANCIA = new RobotState();

    private PresetElevador presetElevador = PresetElevador.MANUAL;
    private PresetIntake presetIntake = PresetIntake.DESLIGADO;

    private double alvoElevadorMetros = 0.0;
    private double alturaCentralMetros = 0.0;
    private double alturaExternaMetros = 0.0;

    private double anguloIntakeRadianos = 0.0;

    private RobotState() {}

    public static RobotState getInstance() {
        return INSTANCIA;
    }

    /** Sets an elevator preset target. */
    public void selecionarPresetElevador(PresetElevador preset) {
        if (preset == null || preset == PresetElevador.MANUAL) {
            throw new IllegalArgumentException("Selecione um preset entre L1 e L4.");
        }
        presetElevador = preset;
        alvoElevadorMetros = preset.getAlturaMetros();
    }

    /** Replaces the current elevator preset with a manual target. */
    public void definirAlvoManualElevador(double alturaMetros) {
        presetElevador = PresetElevador.MANUAL;
        alvoElevadorMetros = alturaMetros;
    }

    public void selecionarPresetIntake(PresetIntake preset) {
        if (preset == null) {
            throw new IllegalArgumentException("O preset do intake nao pode ser nulo.");
        }
        presetIntake = preset;
    }

    public void atualizarPosicaoElevador(double centralMetros, double externaMetros) {
        alturaCentralMetros = centralMetros;
        alturaExternaMetros = externaMetros;
    }

    public void atualizarPosicaoIntake(double anguloRadianos) {
        anguloIntakeRadianos = anguloRadianos;
    }

    public PresetElevador getPresetElevador() { return presetElevador; }
    public PresetIntake getPresetIntake() { return presetIntake; }
    public double getAlvoElevadorMetros() { return alvoElevadorMetros; }
    public double getAlturaCentralMetros() { return alturaCentralMetros; }
    public double getAlturaExternaMetros() { return alturaExternaMetros; }
    public double getAnguloIntakeRadianos() { return anguloIntakeRadianos; }
}
