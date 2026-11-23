# Revisão de confiabilidade Sharks 2026

Este conjunto de mudanças corrige problemas identificados por análise estática. **NÃO significa homologação**, e nenhuma mudança de controle de movimento deve ser utilizada em competição sem `gradlew test build`, simulação e testes de campo.

## Alterações

- `Robot.java`: grava WPILOG em REAL e SIM; use pendrive FAT32 no roboRIO para `/U/logs`.
- `VisionIOSim` recebe `Drive.getSimulatedGroundTruthPose()`; essa posição integra as velocidades simuladas das rodas, sem consultar a pose fusionada pela visão. Foi acrescentado ruído pequeno ao MegaTag2. **Não é uma simulação física de escorregamento, colisão ou dinâmica completa.**
- `Vision.java`: verifica a ambiguidade do método de visão correspondente e rejeita ângulos inválidos.
- `VisionIOReal.java`: calcula tempo de captura a partir de `getLastChange()` do NetworkTables, rejeita quadros atrasados (>0,50 s) e impede que uma mesma pose antiga seja aceita repetidamente em ciclos de 20 ms.
- `Drive.java`: propaga as estimativas de aceleração do PathPlanner ao módulo até o TalonFX `VelocityVoltage.withFeedForward`; inverte o sinal se o módulo girar 180°. **Fica DESATIVADO por padrão** (`Autonomo.FF_AUTONOMO_HABILITADO=false`) e `FF_KA_VOLTS_POR_MPS2=0.0` até caracterização SysId/tuning em hardware real. A simulação também o recebe.
- `Drive.java`: se um módulo ou giroscópio reportar desconexão em `REAL`, a atuação do swerve é bloqueada. É proteção para falhas de comunicação, **não** o sistema antitombamento.
- `RobotContainer.java`: elevador e intake **visuais** só mudam no modo SIM. Para mecanismos reais será preciso especificar motores, sensores e criar subsistemas IOReal/IOSim quando esse hardware existir.
- Testes adicionais: limitador de velocidade, feedforward, timestamp da câmera, intertravamento de sensores, integração simplificada de pose simulada, snap e limitador de aceleração; execução prevista pela CI do GitHub Actions.
- Pacote `sharks9199`: corrigidos três imports antigos `frc.robot.sharks` e realocado o teste `ChassisSpeedLimiterTest` para a estrutura atual.

## O que a Sharks precisa verificar

1. `git diff` para revisar as alterações e `git status` para confirmar arquivos esperados.
2. `./gradlew test build` (ou `.\gradlew.bat test build` no Windows) — este pacote **não foi compilado integralmente com as dependências WPILib** no ambiente de criação; utilitários sem bibliotecas externas foram compilados e testados isoladamente.
3. Testar SIM, REPLAY e REAL separadamente; inspecionar a posição no AdvantageScope e medir a diferença da visão em relação à verdade simulada.
4. Confirmar que o intertravamento de sensor não bloqueie o robô durante a partida por falhas intermitentes — revisando os debouncers e o comportamento em campo.
5. Calibrar motor e controlador de velocidade antes de habilitar o feedforward do PathPlanner. Verificar saturação e sentido do sinal em cada módulo.
6. Confirmar gravação em `/U/logs`, espaço livre no USB e arquivos abrindo no AdvantageScope.

## Sharks Predict Instintict (futuro)

Nenhuma proteção física antitombamento foi implementada neste patch, conforme solicitado.
A futura aplicação WEB pode observar a telemetria NetworkTables, comparar cenários e exibir decisões explicáveis. **O bloqueio de extensão do braço deve ser validado e aplicado pelo código que roda no roboRIO**, não por uma página web, por causa de atraso, desconexão e segurança. NetworkTables deve publicar pedidos, decisões, parâmetros e motivos; solicitações externas não devem ser tratadas como autorizações de segurança.

## Limites da simulação atual

O modelo de verdade simulada integra velocidade medida dos motores simulados (não o estimador) e aplica ruído visual, mas partilha a mesma cinemática e não modela derrapagem, atrito variável, colisões ou topografia. Para validação dinâmica de tombamento será necessária simulação física adicional.

## Observações adicionais

- Para usar `src(3)_corrigido.zip`, substitua o diretório `src/` em uma cópia limpa do projeto WPILib completo; os arquivos `.github/workflows/ci.yml` e `docs/` são opcionais, mas recomendados. Não use o ZIP sozinho como projeto Gradle, porque o upload de origem contém apenas `src/`.
- A fonte de verdade simulada integra velocidades simuladas e por isso ainda não produz uma simulação realmente independente dos modelos de roda; é melhor que usar diretamente a pose fusionada, mas ainda exige validação com modelo de campo e atrito reais.
- O bloqueio por sensor é conservador: a perda do giroscópio (mesmo havendo fallback de odometria) ou de qualquer módulo impede movimento comandado em REAL. Validar debounce e a transição de modo no robô suspenso, depois em campo com velocidade baixa.
- `VisionTimestampUtil` exige alinhamento do relógio NetworkTables com FPGA e rejeita poses de câmera mais antigas que 0,5 s; conferir no hardware o comportamento da Limelight.
