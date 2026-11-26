package frc.robot.sharks;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sharks.commands.swerve.SwerveCommands;
import frc.robot.sharks.commands.swerve.TrackGamepiece;
import frc.robot.sharks.simulation.RobotComponents;
import frc.robot.sharks.subsystems.swerve.Drive;
import frc.robot.sharks.subsystems.swerve.control.HeadingController;
import frc.robot.sharks.subsystems.vision.Vision;
import frc.robot.sharks.subsystems.vision.VisionIO;
import frc.robot.sharks.subsystems.vision.VisionIOReal;
import frc.robot.sharks.subsystems.vision.VisionIOSim;

/**
 * <p>As poses de elevador e intake publicadas por {@code RobotComponents} 
 */
public class RobotContainer {

    private final Drive swerve = Drive.criarSwerve();
    private final HeadingController headingController = new HeadingController();

    private final Vision vision = new Vision(
        swerve,
        switch (Constants.MODO_ATUAL) {
            case REAL -> new VisionIOReal();
            case SIM -> new VisionIOSim(swerve::getSimulatedGroundTruthPose);
            case REPLAY -> new VisionIO() {};
        }
    );

    private final TrackGamepiece trackgamepiece = new TrackGamepiece(swerve, vision);

    private final XboxController driverController =
        new XboxController(Constants.Controles.PORTA_DRIVER);

    private final SendableChooser<Command> meuSeletorAuto;

    public RobotContainer() {
        // O Heading Controller da Sharks pertence exclusivamente ao teleop.
        swerve.setHeadingCorrectionListener(correcao -> {
            if (DriverStation.isTeleopEnabled()) {
                headingController.aplicarCorrecaoReferencia(correcao);
            }
        });

        configureNamedCommands();
        meuSeletorAuto = doAutoSelector();
        SmartDashboard.putData("Autonomo/Seletor", meuSeletorAuto);

        configureBindings();
        configureButtons();
        swerve.setSysIdDeadman(() -> sticksNeutral() && driverController.getLeftBumperButton()
            && driverController.getRightBumperButton());
        SmartDashboard.putData("SysId/QuasistaticForward", swerve.sysId(false, SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysId/QuasistaticReverse", swerve.sysId(false, SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("SysId/DynamicForward", swerve.sysId(true, SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysId/DynamicReverse", swerve.sysId(true, SysIdRoutine.Direction.kReverse));
    }

    private void configureBindings() {
        Command driveCommand = SwerveCommands.movimentacaoDrive(
                swerve,
                headingController,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()
            );
        driveCommand.setName("SwerveTeleop");
        swerve.setDefaultCommand(driveCommand);
    }

    /**
     * Xbox bindings: Start resets yaw, A toggles slow mode, and the D-pad selects
     * L1 (down), L2 (left), L3 (right), or L4 (up) during enabled teleop.
     */
    public void configureButtons() {
        new Trigger(() -> DriverStation.isTeleopEnabled() && driverController.getStartButton())
            .onTrue(Commands.runOnce(this::zerarGuinada));

        new Trigger(() -> DriverStation.isTeleopEnabled() && driverController.getAButton())
            .onTrue(Commands.runOnce(swerve::alternarModoLento));

        new Trigger(() -> DriverStation.isTeleopEnabled() && driverController.getBackButton())
            .onTrue(Commands.runOnce(() -> swerve.rearmSafety(sticksNeutral()
                && !driverController.getYButton() && !driverController.getLeftStickButton()
                && !driverController.getRightBumperButton())));
        new Trigger(() -> DriverStation.isTeleopEnabled() && driverController.getYButton()
                && DriverStation.isJoystickConnected(Constants.Controles.PORTA_DRIVER) && sticksNeutral())
            .whileTrue(Commands.runEnd(swerve::xLock, swerve::stop, swerve));
        new Trigger(() -> DriverStation.isTeleopEnabled() && driverController.getLeftStickButton() && !driverController.getYButton()
                && DriverStation.isJoystickConnected(Constants.Controles.PORTA_DRIVER) && sticksNeutral())
            .whileTrue(Commands.runEnd(() -> {
                if (!vision.hasFreshAprilTag()) { swerve.stop(); return; }
                double omega = edu.wpi.first.math.MathUtil.clamp(-(vision.getTx() - OffsetsConstants.Limelight.TX_ALVO_GRAUS) * Constants.Seguranca.ALINHAMENTO_KP,
                    -Constants.Seguranca.ALINHAMENTO_MAX_RAD_S, Constants.Seguranca.ALINHAMENTO_MAX_RAD_S);
                swerve.runVelocity(new edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, omega));
            }, swerve::stop, swerve, vision).withTimeout(3.0));

        // No SIM, o Right Bumper continua reservado ao elevador visual.
        new Trigger(() -> DriverStation.isTeleopEnabled()
                && Constants.MODO_ATUAL != Constants.Mode.SIM
                && DriverStation.isJoystickConnected(Constants.Controles.PORTA_DRIVER)
                && driverController.getRightBumperButton() && !driverController.getYButton()
                && !driverController.getLeftStickButton()
                && Math.hypot(driverController.getLeftX(), driverController.getLeftY()) < 0.15
                && Math.abs(driverController.getRightX()) < 0.15)
            .whileTrue(trackgamepiece);

        // Elevator presets are only available while teleoperated control is enabled.
        Trigger teleop = new Trigger(() ->
            DriverStation.isTeleopEnabled() && Constants.MODO_ATUAL == Constants.Mode.SIM);

        teleop.and(() -> driverController.getPOV() == 180)
            .onTrue(Commands.runOnce(() ->
                RobotComponents.selecionarPresetElevador(RobotState.PresetElevador.L1)));

        teleop.and(() -> driverController.getPOV() == 270)
            .onTrue(Commands.runOnce(() ->
                RobotComponents.selecionarPresetElevador(RobotState.PresetElevador.L2)));

        teleop.and(() -> driverController.getPOV() == 90)
            .onTrue(Commands.runOnce(() ->
                RobotComponents.selecionarPresetElevador(RobotState.PresetElevador.L3)));

        teleop.and(() -> driverController.getPOV() == 0)
            .onTrue(Commands.runOnce(() ->
                RobotComponents.selecionarPresetElevador(RobotState.PresetElevador.L4)));
    }

    private boolean sticksNeutral() {
        return DriverStation.isJoystickConnected(Constants.Controles.PORTA_DRIVER)
            && Math.hypot(driverController.getLeftX(), driverController.getLeftY()) < 0.08
            && Math.abs(driverController.getRightX()) < 0.08;
    }

    public boolean isDriveSafetyBlocked() { return swerve.isSafetyBlocked(); }

    /** Updates visualization-only mechanism targets from the Xbox controller. */
    public void atualizarPosesPeloControle() {
        // Mecanismos abaixo só existem na visualização; não simular atuadores reais.
        if (Constants.MODO_ATUAL != Constants.Mode.SIM) {
            return;
        }
        boolean teleop = DriverStation.isTeleopEnabled();

        boolean subirElevador = teleop && driverController.getRightBumperButton();
        boolean descerElevador = teleop && driverController.getLeftBumperButton();
        boolean abaixarIntake = teleop && driverController.getBButton();
        boolean levantarIntake = teleop && driverController.getXButton();

        RobotComponents.atualizarPeloControle(
            subirElevador,
            descerElevador,
            abaixarIntake,
            levantarIntake
        );
    }

    /** Observation only; runs even with no dashboard connected. */
    
    private void configureNamedCommands() {
        // Register PathPlanner event commands here when needed.
    }

    private SendableChooser<Command> doAutoSelector() {
        try {
            return AutoBuilder.buildAutoChooser();
        } catch (Exception e) {
            DriverStation.reportError(
                "Erro ao criar o Autonomo: " + e.getMessage(),
                e.getStackTrace()
            );

            SendableChooser<Command> autonomoVazio = new SendableChooser<>();
            autonomoVazio.setDefaultOption("Nenhum Autonomo", Commands.none());
            return autonomoVazio;
        }
    }

    /** Disables teleop heading hold before the autonomous command is scheduled. */
    public void aoIniciarAutonomo() {
        headingController.desativar();
    }

    public Command getAutonomousCommand() {
        Command autoSelecionado = meuSeletorAuto.getSelected();
        return autoSelecionado != null ? autoSelecionado : Commands.none();
    }

    public void aoFinalizarAutonomo() {
        swerve.stop();
        swerve.limparAlvoAutonomo();
        headingController.desativar();
    }

    public void zerarGuinada() {
        swerve.zerarOrientacaO();
        headingController.desativar();
    }
}
