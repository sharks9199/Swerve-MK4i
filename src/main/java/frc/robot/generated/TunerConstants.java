package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.sharks9199.Constants;

/**
 * Exposes the swerve configuration required by Phoenix Tuner X and the module IO layer.
 *
 * <p>Hardware values are sourced from {@link Constants.Swerve}. If this file is regenerated,
 * restore that delegation to prevent inconsistencies between motor setup and odometry.
 */
public final class TunerConstants {

  /** Pigeon 2 configuration, including its mounting orientation. */
  public static final Pigeon2Configuration CONFIGURACAO_PIGEON =
      Constants.Swerve.CONFIGURACAO_PIGEON;

  /** Approximate physical speed at 12 V, used by the module model. */
  public static final LinearVelocity VELOCIDADE_FISICA_A_12V =
      Constants.Swerve.VELOCIDADE_FISICA_A_12V;

  /** Drivetrain-level constants (CAN bus name, Pigeon 2 ID and configuration). */
  public static final SwerveDrivetrainConstants DrivetrainConstants = Constants.Swerve.DRIVETRAIN;

  /** CAN bus the drivetrain devices are connected to. */
  public static final CANBus CAN_BUS = Constants.Swerve.CAN_BUS;

  /** Front-left module (steer 2, drive 1, encoder 9). */
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft = Constants.Swerve.FRONT_LEFT;

  /** Front-right module (steer 4, drive 3, encoder 10). */
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight = Constants.Swerve.FRONT_RIGHT;

  /** Back-left module (steer 6, drive 5, encoder 11). */
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft = Constants.Swerve.BACK_LEFT;

  /** Back-right module (steer 8, drive 7, encoder 12). */
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight = Constants.Swerve.BACK_RIGHT;

  private TunerConstants() {}
}
