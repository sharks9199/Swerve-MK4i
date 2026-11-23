
package frc.robot.sharks.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.sharks.subsystems.swerve.Drive;
import frc.robot.sharks.subsystems.vision.Vision;

public class TrackGamepiece extends Command {

    private final Drive swerve;
    private final Vision limelight;

    private static final double KP = 0.1;
    private static final double KI = 0.0;
    private static final double KD = 0.0;

    private static final double MAX_SPEED = 1.5;
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

        double tx = limelight.getTx();

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

        double vx = MAX_SPEED * alignment;

        swerve.runVelocity(
            new ChassisSpeeds(vx, 0.0, omega)
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runVelocity(new ChassisSpeeds());
        pid.reset();

        limelight.disableGamepieceTracking();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
