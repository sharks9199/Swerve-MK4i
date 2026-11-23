package frc.robot.sharks;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sharks.commands.swerve.SwerveCommands;
import frc.robot.sharks.simulation.RobotComponents;
import frc.robot.sharks.subsystems.swerve.Drive;
import frc.robot.sharks.subsystems.swerve.control.HeadingController;
import frc.robot.sharks.subsystems.vision.Vision;
import frc.robot.sharks.subsystems.vision.VisionIO;
import frc.robot.sharks.subsystems.vision.VisionIOReal;
import frc.robot.sharks.subsystems.vision.VisionIOSim;

/**
 * Composicao do robo Sharks: cria subsistemas, conecta comandos e associa controles.
 *
 * <p>Evite implementar controladores de motores aqui. Toda logica de atuacao
 * pertence aos subsistemas; esta classe configura apenas a integracao.
 *
 * <p>As poses de elevador e intake publicadas por {@code RobotComponents}
 * sao APENAS visuais. Nao existe elevador real implementado nesta base.
 */
public class RobotContainer {

    private final Drive swerve = Drive.criarSwerve();
    private final HeadingController headingController = new HeadingController();

    @SuppressWarnings("unused")
    private final Vision vision = new Vision(
        swerve,
        switch (Constants.MODO_ATUAL) {
            case REAL -> new VisionIOReal();
            case SIM -> new VisionIOSim(swerve::getSimulatedGroundTruthPose);
            case REPLAY -> new VisionIO() {};
        }
    );

    private final XboxController driverController =
        new XboxController(Constants.Controles.PORTA_DRIVER);

    private final SendableChooser<Command> meuSeletorAuto;

    public RobotContainer() {
        swerve.usarHeadingControllerNoAutonomo(headingController);
        swerve.setHeadingCorrectionListener(headingController::aplicarCorrecaoReferencia);

        configureNamedCommands();
        meuSeletorAuto = doAutoSelector();
        SmartDashboard.putData("Autonomo/Seletor", meuSeletorAuto);

        configureBindings();
        configureButtons();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(
            SwerveCommands.movimentacaoDrive(
                swerve,
                headingController,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()
            )
        );
    }

    /**
     * Xbox bindings: Start resets yaw, A toggles slow mode, and the D-pad selects
     * L1 (down), L2 (left), L3 (right), or L4 (up) during enabled teleop.
     */
    public void configureButtons() {
        new Trigger(driverController::getStartButton)
            .onTrue(Commands.runOnce(this::zerarGuinada));

        new Trigger(driverController::getAButton)
            .onTrue(Commands.runOnce(swerve::alternarModoLento));

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

    public Command getAutonomousCommand() {
        Command autoSelecionado = meuSeletorAuto.getSelected();
        return autoSelecionado != null ? autoSelecionado : Commands.none();
    }

    public void aoFinalizarAutonomo() {
        swerve.limparAlvoAutonomo();
        headingController.desativar();
    }

    public void zerarGuinada() {
        swerve.zerarOrientacaO();
        headingController.desativar();
    }
}
