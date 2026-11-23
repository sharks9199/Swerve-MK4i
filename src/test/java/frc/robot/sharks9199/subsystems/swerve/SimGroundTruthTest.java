package frc.robot.sharks9199.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sharks.subsystems.swerve.SimGroundTruth;

import org.junit.jupiter.api.Test;

class SimGroundTruthTest {
  @Test void integratesWithoutReadingPoseEstimator() {
    SimGroundTruth groundTruth = new SimGroundTruth();
    groundTruth.step(new ChassisSpeeds(1, 0, 0), 0.02);
    assertEquals(0.02, groundTruth.getPose().getX(), 1e-8);
    groundTruth.reset(new Pose2d(2, 1, Rotation2d.kZero));
    assertEquals(2, groundTruth.getPose().getX(), 1e-8);
  }

  @Test void refusesUnboundedSimulationTimeStep() {
    SimGroundTruth groundTruth = new SimGroundTruth();
    groundTruth.step(new ChassisSpeeds(1, 0, 0), 10);
    assertEquals(0.05, groundTruth.getPose().getX(), 1e-8);
  }
}
