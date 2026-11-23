# Migração do ZIP original para Sharks Base 2026

## Arquivos movidos

- `frc.robot.Constants` → `frc.robot.sharks.Constants`
- `frc.robot.RobotContainer` → `frc.robot.sharks.RobotContainer`
- `frc.robot.RobotState` → `frc.robot.sharks.RobotState`
- `frc.robot.OffsetsConstants` → `frc.robot.sharks.OffsetsConstants`
- `frc.robot.commands.*` → `frc.robot.sharks.commands.*`
- `frc.robot.subsystems.*` → `frc.robot.sharks.subsystems.*`
- `frc.robot.util.RobotComponents` → `frc.robot.sharks.simulation.RobotComponents`
- `frc.robot.util.PhoenixUtil` → `frc.robot.sharks.util.PhoenixUtil`

`Main.java`, `Robot.java` e `generated/TunerConstants.java` foram mantidos em seus pacotes originais. Seus imports foram corrigidos.

## Correções e melhorias

1. Substituído limite rápido de 9,20 m/s por valor inicial de 3,00 m/s, abaixo da referência física teórica de 4,20 m/s a 12 V.
2. Definidos limites separados de translação, aceleração teleop e giro para modos lento/rápido.
3. Limitação central de velocidade vetorial e angular em `Drive.runVelocity` para teleop/autônomo; proteção da velocidade diagonal.
4. `SwerveCommands` passa a usar o limite angular do modo e sincroniza estado de limitador quando ocorre clamp.
5. Corrigida histerese de snap de eixo: limite para entrar é menor do que para sair.
6. Simulação de módulo usa o período configurado em `Constants.Swerve`.
7. Alerta de gyro desconectado limitado ao modo REAL, e dados de diagnóstico antes `TESTE/` passaram a `Drive/Diagnostic/`.
8. Removidos stubs inacabados de `subsystems/arm` (`Arm.java`, `ArmIO.java`, `ArmIOReal.java`, `ArmIOSim.java`); implementação requer especificação de hardware.
9. Adicionados Javadoc, README, guias e testes de limitador.

## Instruções de migração segura

1. Faça cópia integral do projeto anterior, incluindo `build.gradle`, `settings.gradle`, `gradle/`, `gradlew.bat`, `vendordeps/`, `.wpilib/` e seus CADs/arquivos visuais.
2. Crie uma branch separada e **substitua** a pasta `src/` original pela disponibilizada neste pacote.
3. Não mantenha as classes antigas nos pacotes `frc.robot.commands`, `frc.robot.subsystems`, `frc.robot.util`; elas não devem coexistir com os pacotes Sharks.
4. Ajuste imports de qualquer código externo que faça referência às classes movidas (se existir fora de `src/`).
5. Rode `gradlew test build` usando o ambiente WPILib correto, depois teste em simulação e em robô físico.
6. Não migre isoladamente para outro projeto FRC sem compatibilizar configurações de motores/encoders e dependências.
