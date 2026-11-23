# Calibração, segurança e velocidades

## 1. Antes de qualquer teste com o robô real

- [ ] Confirmar ano WPILib e versões exatas das vendor dependencies.
- [ ] Confirmar IDs CAN, tipo de motor, redução, diâmetro das rodas e sentido de inversão.
- [ ] Conferir offsets CANcoder e orientação do Pigeon 2 com as medições atuais.
- [ ] Verificar massa, momento de inércia e limites elétricos; parâmetros do ZIP são estimativas.
- [ ] Elevar o chassi sobre cavaletes adequados durante primeiro teste de direção; manter parada de emergência acessível.
- [ ] Validar que a direção de cada módulo, a orientação do joystick e o gyro correspondem ao esperado.
- [ ] Calibrar sob supervisão dos responsáveis pela segurança da equipe.

## 2. Ajustar velocidade

Editar `src/main/java/frc/robot/sharks/Constants.java`, seção `SHARKS | AJUSTES PRINCIPAIS DE VELOCIDADE`.

Sugestão para primeiro teste: 1,50 m/s lento, 3,00 m/s rápido, giro 120°/s lento e 300°/s rápido. Esses **não são valores homologados**.

- Testar lento e rápido individualmente em linha reta e em diagonal.
- Testar alternância de modo em movimento e a parada do comando de teleop.
- Testar giro horário/anti-horário e resposta do heading hold.
- Verificar logs de setpoints versus velocidades medidas e corrente dos motores.
- Ajustar rampas de aceleração apenas após verificar aderência e estabilidade física.
- Repetir em diferentes níveis de bateria e com mecanismos erguidos, se houver.
- **Nunca** aumentar limites com base apenas na simulação. A velocidade teórica de 4,20 m/s não é uma medição validada.

### O que cada limite protege (e o que não protege)

- Os limites de velocidade centralizados (`Drive.runVelocity`) se aplicam também ao autônomo.
- Os limites de aceleração neste projeto são aplicados principalmente pelo comando **teleoperado**. Um autônomo utiliza as restrições de perfil próprias do PathPlanner; não há limitador de aceleração central e universal.
- O limitador de velocidade não mede o centro de gravidade e não evita tombamento sozinho.
- Limites de tensão/corrente continuam dependendo das configurações do hardware e da eletrônica.

## 3. Check final antes de competição

- [ ] `test build` executado e aprovado na configuração WPILib do projeto.
- [ ] Trajetórias PathPlanner verificadas em simulação **e** campo.
- [ ] Autônomo termina e entrega controle ao piloto.
- [ ] Modo lento/rápido, gyro, visão e parada respondem conforme esperado.
- [ ] Sem warnings de hardware, perda de odometria ou falhas de câmera persistentes.
- [ ] Revisão de código aprovada por outro integrante.
