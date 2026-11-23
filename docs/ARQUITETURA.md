# Arquitetura da base Sharks

## Árvore resumida

```text
src/
├── main/
│   ├── deploy/pathplanner/              # Arquivos de trajetórias do PathPlanner
│   └── java/frc/robot/
│       ├── Main.java                    # Inicialização padrão WPILib
│       ├── Robot.java                   # Ciclo de vida / modos / logging
│       ├── generated/TunerConstants.java # Delegação CTRE; gerado/compatibilidade
│       └── sharks/
│           ├── Constants.java           # Valores globais + SEÇÃO DE VELOCIDADES
│           ├── OffsetsConstants.java    # Offsets de hardware calibrados
│           ├── RobotContainer.java      # Instanciação, controles e comandos
│           ├── RobotState.java          # Estado da simulação visual dos mecanismos
│           ├── commands/swerve/         # Direção teleoperada
│           ├── subsystems/swerve/       # Chassi, módulos, odometria e controle
│           ├── subsystems/vision/       # Câmera e fusão de AprilTags
│           ├── simulation/              # Poses 3D para AdvantageScope
│           └── util/                    # Utilitários técnicos
└── test/java/frc/robot/sharks/          # Testes automatizados
```

## Responsabilidades e fluxo

1. `Main` inicia `Robot`.
2. `Robot` configura AdvantageKit e chama `CommandScheduler` periodicamente.
3. `RobotContainer` cria `Drive` e `Vision`, associa o Xbox aos comandos e publica o seletor autônomo.
4. `SwerveCommands` lê o controle, trata deadband/snap/heading e **limita aceleração de teleop**.
5. `Drive.runVelocity` aplica os limites de **velocidade linear e angular** tanto em teleop como em autônomo. `ChassisSpeedLimiter` limita a norma do vetor, incluindo na diagonal.
6. `ModuleIOReal`/`GyroIOReal` falam com hardware; `ModuleIOSim` simula módulo; `VisionIOSim` gera medições simuladas.
7. `Drive` atualiza a pose usando a odometria e a visão. PathPlanner pode comandar `Drive.runVelocity` por meio do AutoBuilder.
8. `RobotComponents` publica poses articuladas para visualização; não controla mecanismos reais.

## Convenções de pacote

- O caminho físico da classe deve coincidir com o `package` declarado.
- Classes de aplicação: `frc.robot.sharks.*`.
- Entrypoints: `frc.robot.Main` e `frc.robot.Robot`.
- Configuração Phoenix: `frc.robot.generated.TunerConstants`.
- Não duplicar arquivos antigos de subsistemas em `frc.robot.subsystems`; isso pode confundir imports e compilar classes obsoletas.

## O que é REAL, SIM e REPLAY

- **REAL:** usa interfaces reais de giroscópio, motores e visão.
- **SIM:** usa `ModuleIOSim`, `VisionIOSim` e giroscópio vazio com heading obtido por integração cinemática.
- **REPLAY:** consome registros do AdvantageKit; exige um arquivo de log disponível.

A escolha de modo está em `Constants.MODO_SIMULACAO` / `Constants.MODO_ATUAL`. Em roboRIO, o código seleciona `REAL` automaticamente.

## Restrições importantes

- Os limites de velocidade não são um sistema antitombamento. **Physical Instinct ainda é uma proposta**, não um módulo instalado.
- A lógica de braço fornecida no ZIP eram arquivos vazios/placeholder e foi retirada; não há controle de braço implementado.
- Elevação/intake da visualização não equivalem a subsistemas reais.
- O callback do PathPlanner original ignora valores `feedforwards` enviados pela biblioteca. Isso pode afetar a qualidade do seguimento de trajetória; revisar tuning e implementar suporte quando necessário.
- O ponto de partida visual da simulação depende do CAD e precisa de calibração para novos robôs.
