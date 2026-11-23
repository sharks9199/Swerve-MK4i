# Registro de validação — Sharks Base 2026

**Estado da entrega:** revisão estática e testes isolados; homologação completa pendente.

## O que foi verificado nesta entrega

- [x] Arquivos ZIP do usuário extraídos e reorganizados sem apagar trajetórias `pathplanner`.
- [x] Pacotes Java reestruturados em `frc.robot.sharks` com `Main` e `Robot` intactos no `frc.robot`.
- [x] Referências de import e diretórios revisadas por verificador estático (relatório atualizado em `docs/RESULTADOS_TESTES.md`).
- [x] Matemática do limite vetorial e angular testada isoladamente com compilação Java 17.
- [x] Velocidades padrão ajustadas abaixo da velocidade teórica do projeto.
- [x] Histerese do snap corrigida: entrada 0,12, saída 0,20.
- [x] Testes JUnit 5 adicionados (a execução via Gradle depende do ambiente WPILib original).

## O que NÃO foi verificado

- [ ] `gradlew build` e `gradlew test` de ponta a ponta com dependências WPILib/CTRE/PathPlanner (ZIP enviado continha apenas `src`).
- [ ] Compilação de todas as classes com bibliotecas externas reais.
- [ ] Simulação gráfica em VS Code/AdvantageScope e funcionalidades de PathPlanner.
- [ ] Operação no roboRIO, hardware CAN, giroscópio, câmera e ajuste de odometria.
- [ ] Segurança física em campo, risco de tombamento e calibração de massas/velocidades.

Portanto, **não há base técnica para afirmar que toda a programação está 100% correta ou pronta para competição**. Esta versão foi preparada para ser submetida aos testes indicados e para servir de base organizada.

## Comandos a executar no projeto WPILib completo

```powershell
.\gradlew.bat test build
```

Para concluir a validação, incluir a saída dos comandos, logs de simulação, resultados de testes físicos e aprovação da equipe neste documento.
