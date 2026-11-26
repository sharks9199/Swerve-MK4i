package frc.robot.sharks.subsystems.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.Logger;

/**
 * A single swerve module. Wraps a {@link ModuleIO} and converts between hardware units (radians)
 * and WPILib kinematics types (meters).
 */
public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;

  /** Odometry samples converted from the latest inputs. */
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /**
   * Creates a module.
   *
   * @param io the hardware IO for this module
   * @param index module index (0 = front left, 1 = front right, 2 = back left, 3 = back right)
   * @param constants the module's physical constants
   */
  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;

    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  /** Updates and logs inputs, converts odometry samples and refreshes disconnect alerts. */
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    int sampleCount =
        Math.min(
            inputs.odometryTimestamps.length,
            Math.min(inputs.odometryDrivePositionsRad.length, inputs.odometryTurnPositions.length));
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /**
   * Runs the module toward the given setpoint. The state is optimized and cosine-scaled in place.
   *
   * @param state desired module state
   */
  public void runSetpoint(SwerveModuleState state) {
    runSetpoint(state, 0.0);
  }

  /** Runs an optimized module state, correcting feedforward sign when wheel direction flips. */
  public void runSetpoint(SwerveModuleState state, double feedforwardVolts) {
    boolean flipped = Math.abs(state.angle.minus(getAngle()).getRadians()) > Math.PI / 2.0;
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);
    double signedFeedforward = flipped ? -feedforwardVolts : feedforwardVolts;
    io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius, signedFeedforward);
    io.setTurnPosition(state.angle);
  }

  public boolean isAlignedForSysId() {
    return isHealthy() && Math.abs(edu.wpi.first.math.MathUtil.angleModulus(getAngle().getRadians())) < Math.toRadians(5.0);
  }

  public void characterize(double volts) {
    io.setTurnPosition(Rotation2d.kZero);
    io.setDriveOpenLoop(isAlignedForSysId() ? volts : 0.0);
  }

  public void logSysId(edu.wpi.first.wpilibj.sysid.SysIdRoutineLog log, String name) {
    log.motor(name).voltage(edu.wpi.first.units.Units.Volts.of(inputs.driveAppliedVolts))
        .linearPosition(edu.wpi.first.units.Units.Meters.of(getPositionMeters()))
        .linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(getVelocityMetersPerSec()));
  }

  /** Stops both motors (open loop, zero output). */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  /** Signal health is debounced in the IO implementation before use. */
  public boolean isHealthy() {
    return inputs.driveConnected && inputs.turnConnected && inputs.turnEncoderConnected
        && Double.isFinite(inputs.drivePositionRad) && Double.isFinite(inputs.driveVelocityRadPerSec)
        && inputs.turnPosition != null && Double.isFinite(inputs.turnPosition.getRadians())
        && Double.isFinite(inputs.turnVelocityRadPerSec);
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the drive position of the wheel, in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  /** Returns the drive velocity of the wheel, in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  /** Returns the current module position (distance and angle). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the current module state (velocity and angle). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the high-frequency odometry positions sampled since the last cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the odometry samples, in seconds. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }
}
