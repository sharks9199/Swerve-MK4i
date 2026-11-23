package frc.robot.sharks.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Simple independent simulation pose (not a full contact/slip model). */
public final class SimGroundTruth {
  private Pose2d pose = new Pose2d();

  public void reset(Pose2d initialPose) {
    pose = initialPose;
  }

  public void step(ChassisSpeeds measuredSimSpeeds, double dtSeconds) {
    if (!Double.isFinite(dtSeconds) || dtSeconds <= 0.0 ||
        !Double.isFinite(measuredSimSpeeds.vxMetersPerSecond) ||
        !Double.isFinite(measuredSimSpeeds.vyMetersPerSecond) ||
        !Double.isFinite(measuredSimSpeeds.omegaRadiansPerSecond)) return;
    // Prevent large jump after debugger pauses or simulator lag.
    double dt = Math.min(dtSeconds, 0.05);
    pose = pose.exp(new Twist2d(measuredSimSpeeds.vxMetersPerSecond * dt,
        measuredSimSpeeds.vyMetersPerSecond * dt,
        measuredSimSpeeds.omegaRadiansPerSecond * dt));
  }

  public Pose2d getPose() {
    return pose;
  }
}
