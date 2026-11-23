package frc.robot.sharks.subsystems.swerve.control;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import frc.robot.sharks.subsystems.swerve.control.DriveSensorHealth;

class DriveSensorHealthTest {
  @Test void blocksFaultyModuleAndGyro() {
    assertFalse(DriveSensorHealth.criticalFault(true, new boolean[]{true,true,true,true}));
    assertTrue(DriveSensorHealth.criticalFault(false, new boolean[]{true,true,true,true}));
    assertTrue(DriveSensorHealth.criticalFault(true, new boolean[]{true,false,true,true}));
    assertTrue(DriveSensorHealth.criticalFault(true, new boolean[]{true,true}));
    assertTrue(DriveSensorHealth.criticalFault(true, null));
  }
}
