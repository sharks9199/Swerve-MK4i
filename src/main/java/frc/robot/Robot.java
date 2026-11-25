// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sharks.Constants;
import frc.robot.sharks.RobotContainer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.LoggedRobot;
import org.ironmaple.simulation.SimulatedArena;
import static edu.wpi.first.units.Units.Seconds;

   /**
   * Main robot class. Configures logging, owns the {@link RobotContainer} and drives the command
   * scheduler for every robot mode.
   */
  public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /** Starts AdvantageKit logging and builds the {@link RobotContainer}. */
    public Robot() {
      RobotController.setBrownoutVoltage(Constants.Energia.TENSAO_BROWNOUT_VOLTS);

      if (Constants.MODO_ATUAL == Constants.Mode.REPLAY) {

        // Process replay logs without waiting for real-time cycle timing.
        setUseTiming(false);

        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        Logger.addDataReceiver(new NT4Publisher());
      } else {

        Logger.addDataReceiver(new NT4Publisher());
        // REAL: /U/logs em pendrive FAT32 no roboRIO; SIM: pasta logs local.
        Logger.addDataReceiver(new WPILOGWriter());
      }
      Logger.start();

      m_robotContainer = new RobotContainer();

    }

    /** Runs the command scheduler. Must be called every loop, regardless of mode. */
    @Override
    public void robotPeriodic() {
      CommandScheduler.getInstance().run();
      m_robotContainer.atualizarPosesPeloControle();
      // Telemetria somente para o dashboard; não interfere nos comandos.
      SmartDashboard.putNumber("Sharks/TempoPartida", DriverStation.getMatchTime());
      SmartDashboard.putNumber("Sharks/BateriaVolts", RobotController.getBatteryVoltage());
      SmartDashboard.putString("Sharks/Modo", DriverStation.isDisabled() ? "DISABLED"
          : DriverStation.isAutonomous() ? "AUTO"
          : DriverStation.isTest() ? "TESTE" : "TELEOP");
      SmartDashboard.putBoolean("Sharks/AutoEmExecucao", DriverStation.isAutonomousEnabled()
          && m_autonomousCommand != null && m_autonomousCommand.isScheduled());
    }

    @Override
    public void simulationPeriodic() {
        SimulatedArena.overrideSimulationTimings(Seconds.of(0.01), 3);
        SimulatedArena.getInstance().simulationPeriodic();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    /** Schedules the autonomous command selected on the dashboard, if any. */
    @Override
    public void autonomousInit() {
      m_robotContainer.aoIniciarAutonomo();
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      if (m_autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(m_autonomousCommand);
      }
    }

    @Override
    public void autonomousPeriodic() {}

    /** Releases the heading controller and path target so teleop starts from a clean state. */
    @Override
    public void autonomousExit() {
      m_robotContainer.aoFinalizarAutonomo();
    }

    /** Cancels the autonomous command so that the driver takes over immediately. */
    @Override
    public void teleopInit() {
      if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
      }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /** Cancels all running commands when entering test mode. */
    @Override
    public void testInit() {
      CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

  }
