
package frc.robot.sharks.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sharks.Constants;
import frc.robot.sharks.OffsetsConstants;
import frc.robot.sharks.subsystems.swerve.Drive;
import frc.robot.sharks.subsystems.vision.Vision;

public class TrackGamepiece extends Command {

    private final Drive swerve;
    private final Vision limelight;

    private static final double KP = 0.1;
    private static final double KI = 0.0;
    private static final double KD = 0.0;

    private static final double MAX_SPEED = 0.75;
    private static final double TIMEOUT_S = 3.0;
    private final Timer timer = new Timer();
    private static final double MAX_OMEGA = 2.0;

    private static final double ANGLE_TOLERANCE = 0.25;
    private static final double MAX_APPROACH_ANGLE = 25.0;

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
        timer.restart();

        swerve.runVelocity(new ChassisSpeeds());

        limelight.enableGamepieceTracking();
    }

    @Override
    public void execute() {
        turnToGamepiece();
    }

    public void turnToGamepiece() {

        if (!limelight.hasTarget()) {
            swerve.runVelocity(new ChassisSpeeds());
            pid.reset();
            return;
        }

        double tx = limelight.getTx() - OffsetsConstants.Limelight.TX_ALVO_GRAUS;

        if (!Double.isFinite(tx)) { //se tx não pertencer ao conjunto dos números finitos.
            swerve.runVelocity(new ChassisSpeeds());
            pid.reset();
            return;
        }

        double omega = MathUtil.clamp(
            pid.calculate(tx, 0.0),
            -MAX_OMEGA,
            MAX_OMEGA
        );

        if (pid.atSetpoint()) {
            omega = 0.0;
        }

        double alignment = MathUtil.clamp(
            1.0 - Math.abs(tx) / MAX_APPROACH_ANGLE,
            0.0,
            1.0
        );

        double vx = 0.0;
        double area = limelight.getGamepieceArea();
        double stopArea = Constants.Vision.AREA_PARADA_GAMEPIECE_PERCENT;
        if (Constants.Vision.APROXIMACAO_GAMEPIECE_HABILITADA
                && Double.isFinite(stopArea) && stopArea > 0.0 && stopArea <= 100.0
                && Double.isFinite(area) && area > 0.0 && area < stopArea) {
            vx = MAX_SPEED * alignment * MathUtil.clamp(1.0 - area / stopArea, 0.0, 1.0);
        }

        swerve.runVelocity(
            new ChassisSpeeds(vx, 0.0, omega)
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runVelocity(new ChassisSpeeds());
        pid.reset();

        timer.stop();
        limelight.disableGamepieceTracking();
    }

    @Override
    public boolean isFinished() {
        double stopArea = Constants.Vision.AREA_PARADA_GAMEPIECE_PERCENT;
        return timer.hasElapsed(TIMEOUT_S)
            || (Constants.Vision.APROXIMACAO_GAMEPIECE_HABILITADA
                && stopArea > 0.0 && stopArea <= 100.0 && limelight.hasTarget()
                && limelight.getGamepieceArea() >= stopArea);
    }
}
