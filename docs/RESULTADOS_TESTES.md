# Resultados de verificacao — Sharks Base 2026

Data da revisao: 23/11/2026.

## Executado neste ambiente

| Verificacao | Resultado |
|---|---|
| Pacotes Java correspondem ao caminho dos arquivos | PASSOU (26 arquivos de producao) |
| Imports internos Java sem referencia quebrada | PASSOU (annotation processor gerara `*InputsAutoLogged`) |
| Compilacao Java 17 do limitador (`ChassisSpeedLimiter.java`) | PASSOU |
| Cenarios de limitacao: vetor diagonal 3-4-5, rotacao positiva/negativa, NaN, infinito, limites zero e limites invalidos | PASSOU |
| Arquivos `src/main/deploy` preservados byte a byte | PASSOU |
| Revisao dos novos valores de velocidade e dos limiares de snap | PASSOU |

## Nao foi possivel executar

A tentativa de compilar todo o projeto por `javac` **sem as dependencias externas** encontrou erros esperados de simbolos nao resolvidos (WPILib, CTRE Phoenix, PathPlanner, AdvantageKit etc.). Essa tentativa **nao e** um build validado da aplicacao completa.

O ZIP original trazia somente `src/`. Nao havia `build.gradle`, `vendordeps/`, wrapper Gradle, instalacao WPILib nem ambiente roboRIO. Portanto **`gradlew test build` nao foi executado aqui** e os testes JUnit incluidos ainda precisam ser executados no projeto WPILib completo.

## Evidencia necessaria para homologacao

- Compilar com `gradlew test build` no ambiente WPILib 2026 da equipe.
- Executar a simulacao completa e testar comando do piloto, odometria, camera e trajetorias.
- Validar IDs CAN, inversoes, offsets, tempos, limites e calibracoes no robo fisico com seguranca.
- Repetir testes com elevador/intake reais apos implementacao dos subsistemas.
