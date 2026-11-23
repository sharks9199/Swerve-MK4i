# SHARKS | FRC 2026 — Base Swerve MK4i

> **Status:** base de código reorganizada e revisada, **não homologada em hardware**. Consulte `docs/VALIDACAO.md` antes de afirmar que o robô está pronto para competição.

Esta entrega foi construída a partir do ZIP `src(2).zip` disponibilizado pela equipe. Inclui **código-fonte e arquivos de deploy**, não inclui Gradle Wrapper, `build.gradle`, `settings.gradle` e `vendordeps`: esses arquivos não estavam no ZIP enviado. **Copie o `src/` desta entrega para a raiz de um projeto WPILib 2026 existente**, com as mesmas bibliotecas do projeto original. Não abra esta pasta isoladamente como se fosse um projeto Gradle autossuficiente.

## Principais funcionalidades existentes

- Chassi Swerve (4 módulos) com abstração de hardware e simulação.
- Comando de direção teleoperada com deadband radial, curva exponencial, snap de eixo, limite de aceleração e controle de heading.
- Modos **rápido e lento** via botão **A** do controle Xbox; ambos limitados no ponto central de atuação (`Drive.runVelocity`).
- Odometria de módulos/giroscópio, estimativa de pose, câmera Limelight com AprilTags.
- PathPlanner para seleção de autônomos (trajetórias originais preservadas).
- Publicação de poses 3D de elevador e intake **apenas para visualização** no AdvantageScope.
- Logs via AdvantageKit e dashboard para configuração/diagnóstico.

## Onde ajustar a velocidade do robô

**Arquivo:** `src/main/java/frc/robot/sharks/Constants.java`, classe interna `Constants.Swerve`, seção **SHARKS | AJUSTES PRINCIPAIS DE VELOCIDADE**.

| Constante | Padrão | Unidade | Efeito |
| --- | ---: | --- | --- |
| `VELOCIDADE_MAXIMA_MPS_LENTO` | 1,50 | m/s | Velocidade linear máxima no modo lento |
| `ACELERACAO_MAXIMA_MPS2_LENTO` | 1,50 | m/s² | Rampa de translação no modo lento, no comando de teleop |
| `VELOCIDADE_MAXIMA_MPS_RAPIDO` | 3,00 | m/s | Velocidade linear máxima no modo rápido |
| `ACELERACAO_MAXIMA_MPS2_RAPIDO` | 3,00 | m/s² | Rampa de translação no modo rápido, no comando de teleop |
| `VELOCIDADE_ANGULAR_MAXIMA_RAD_S_LENTO` | 120°/s | rad/s | Giro máximo no modo lento |
| `VELOCIDADE_ANGULAR_MAXIMA_RAD_S_RAPIDO` | 300°/s | rad/s | Giro máximo no modo rápido |
| `ACELERACAO_ANGULAR_MAXIMA_RAD_S2` | 360°/s² | rad/s² | Rampa de giro aplicada pelo comando de teleop |
| `VELOCIDADE_MAXIMA_FISICA_ESTIMADA_MPS` | 4,20 | m/s | Referência **teórica** de velocidade a 12 V; verificar hardware |

As velocidades máximas limitam o **módulo do vetor de translação**, não cada eixo isoladamente. O limite angular também vale em `Drive.runVelocity`. O comando de teleop aplica a rampa linear; o PathPlanner tem seu próprio perfil de trajetória e o chassi impõe somente os limites instantâneos de velocidade. **Não há limitador central de aceleração para todas as fontes de comando**, nem prevenção de tombamento; não confundir com o SPI.

Valores deliberadamente conservadores como ponto de partida, não números calibrados no robô. Consulte `docs/CALIBRACAO.md`.

## Controles Xbox atualmente configurados

| Ação | Botão | Comportamento |
|---|---|---|
| Alternar velocidade lenta/rápida | A | Troca perfil de translação/giro |
| Redefinir orientação do estimador | Start | Ajusta referência de guinada do robô |
| Presets visuais do elevador | D-Pad baixo/esquerda/direita/cima | L1/L2/L3/L4 (somente teleop) |
| Mover elevador visualmente | RB / LB | Sobe / desce simulação de pose |
| Alterar posição visual do intake | B / X | Abaixa / levanta a representação |

**Atenção:** esses comandos de elevador/intake movem somente a **representação visual**, não motores reais.

## Preparação e execução

1. Faça backup do projeto WPILib completo e trabalhe em branch separada: `git switch -c refactor/sharks-base`.
2. Substitua somente a pasta `src/` do projeto original pelo diretório `src/` deste ZIP, evitando deixar classes antigas duplicadas (especialmente pacotes `frc.robot.subsystems` e `frc.robot.sharks.subsystems`).
3. Confirme que o `build.gradle` do projeto original usa `frc.robot.Main` como classe principal e inclua as vendor dependencies necessárias (AdvantageKit, Phoenix 6, PathPlanner, REV, MapleSim, etc. conforme o projeto).
4. Rode **`./gradlew test build`** no Linux/macOS ou **`.\gradlew.bat test build`** no PowerShell do Windows (veja comandos abaixo).
5. Abra a simulação com **WPILib: Simulate Robot Code** no VS Code, observe Swerve, PathPlanner e logs.
6. Realize os testes físicos em ambiente controlado; só depois faça o deploy para o roboRIO.

PowerShell (Windows):

```powershell
.\gradlew.bat test build
```

Shell (Linux/macOS):

```bash
./gradlew test build
```

Caso falhem, **não faça deploy** sem revisar os erros e a configuração de versões.

## Documentação detalhada

- [`docs/ARQUITETURA.md`](docs/ARQUITETURA.md) — estrutura e responsabilidades.
- [`docs/CALIBRACAO.md`](docs/CALIBRACAO.md) — checklist de velocidades e hardware.
- [`docs/VALIDACAO.md`](docs/VALIDACAO.md) — evidências de teste e limitações.
- [`docs/MIGRACAO.md`](docs/MIGRACAO.md) — histórico de alterações e migração segura.

## Convenções

- Nome do pacote da equipe: `frc.robot.sharks`.
- `Main` e `Robot` continuam no pacote `frc.robot` para manter a integração WPILib.
- Arquivos gerados de Phoenix Tuner permanecem em `frc.robot.generated`.
- Lógica de hardware em `subsystems/*`, comandos em `commands/*`, visualização em `simulation/*`.
- Não adicionar novas classes vazias representando mecanismos inexistentes.
- O sistema de segurança física **Sharks Physical Instinct** ainda não foi implementado nesta entrega.
