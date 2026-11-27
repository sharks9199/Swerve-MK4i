
package frc.robot.sharks.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sharks.Constants;
import frc.robot.sharks.OffsetsConstants;
import frc.robot.sharks.subsystems.swerve.Drive;
import frc.robot.sharks.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/** Rastreia e segue a gamepiece continuamente enquanto o botao RB estiver pressionado. */
public class TrackGamepiece extends Command {

    private final Drive swerve;
    private final Vision limelight;

    private static final double KP = 1.00;
    private static final double KI = 0.0;
    private static final double KD = 0.45;

    private static final double MAX_SPEED = 3.5; // m/s (faixa 2-3 m/s)
    private static final double MAX_OMEGA = 3.5; // rad/s

    private static final double ANGLE_TOLERANCE = 1.5; // graus
    private static final double MAX_APPROACH_ANGLE = 25.0; // graus

    private final PIDController pid =
        new PIDController(KP, KI, KD);

    public TrackGamepiece(Drive swerve, Vision limelight) {

        this.swerve = swerve;
        this.limelight = limelight;

        pid.setTolerance(ANGLE_TOLERANCE);
        pid.setSetpoint(0.0);

        addRequirements(swerve, limelight);
    }

    @Override
    public void initialize() {
        pid.reset();
        swerve.stop();
        limelight.enableGamepieceTracking();
    }

    @Override
    public void execute() {
        turnToGamepiece();
    }

    public void turnToGamepiece() {

        if (!limelight.hasTarget()) {
            parar("Sem alvo valido");
            return;
        }

        double tx = limelight.getTx() - OffsetsConstants.Limelight.TX_ALVO_GRAUS; //coloquei isso pq a limelight tá na lateral direita do robo
        double area = limelight.getGamepieceArea();

        if (!Double.isFinite(tx) || !areaValida(area)) {
            parar("Dados invalidos");
            return;
        }

        double stopArea = Constants.Vision.AREA_PARADA_GAMEPIECE_PERCENT;
        boolean aproximacaoCalibrada = Constants.Vision.APROXIMACAO_GAMEPIECE_HABILITADA
            && areaValida(stopArea);

        Logger.recordOutput("Vision/Gamepiece/AreaPercent", area);
        Logger.recordOutput("Vision/Gamepiece/StopAreaPercent", stopArea);
        SmartDashboard.putNumber("Gamepiece/AreaPercent", area);

        if (aproximacaoCalibrada && area >= stopArea) {
            parar("Area de parada atingida (~0.5 m)");
            return;
        }

        double omega = Math.abs(tx) <= ANGLE_TOLERANCE ? 0.0
            : MathUtil.clamp(pid.calculate(tx, 0.0), -MAX_OMEGA, MAX_OMEGA);

        double alignment = MathUtil.clamp(
            1.0 - Math.abs(tx) / MAX_APPROACH_ANGLE,
            0.0,
            1.0
        );

        double vy = 0.0;
        if (aproximacaoCalibrada && Math.abs(tx) < MAX_APPROACH_ANGLE) {
            double proximidade = MathUtil.clamp(1.0 - area / stopArea, 0.0, 1.0);
            // A Limelight esta montada na lateral direita do robo (-Y no referencial WPILib).
            vy = -(MAX_SPEED * alignment * proximidade);
        }

        String estado = vy < 0.0 ? "Seguindo"
            : (aproximacaoCalibrada ? "Alinhando" : "Somente alinhamento");
        Logger.recordOutput("Vision/Gamepiece/State", estado);
        Logger.recordOutput("Vision/Gamepiece/RequestedVx", 0.0);
        Logger.recordOutput("Vision/Gamepiece/RequestedVy", vy);
        SmartDashboard.putString("Gamepiece/Estado", estado);
        SmartDashboard.putBoolean("Gamepiece/TemAlvo", true);

        swerve.runVelocity(
            new ChassisSpeeds(0.0, vy, omega)
        );
    }

    private static boolean areaValida(double area) {
        return Double.isFinite(area) && area > 0.0 && area <= 100.0;
    }

    private void parar(String motivo) {
        swerve.stop();
        pid.reset();
        Logger.recordOutput("Vision/Gamepiece/State", motivo);
        Logger.recordOutput("Vision/Gamepiece/RequestedVx", 0.0);
        Logger.recordOutput("Vision/Gamepiece/RequestedVy", 0.0);
        SmartDashboard.putString("Gamepiece/Estado", motivo);
        SmartDashboard.putBoolean("Gamepiece/TemAlvo", false);
    }

    @Override
    public void end(boolean interrupted) {
        parar("Inativo");
        limelight.disableGamepieceTracking();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
