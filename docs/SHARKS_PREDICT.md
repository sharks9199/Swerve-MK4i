# Sharks Predict Instintict — proposta, não implementada

## Objetivo

Um painel web para entender, visualizar e simular decisões do robô da Sharks e analisar se uma manobra, como estender um braço enquanto se move, respeita limites físicos definidos e validados.

## Arquitetura que eu recomendo

1. **Robô (roboRIO)** publica via NetworkTables v4: velocidades e acelerações medidas, inclinação, posição real do braço/elevador, estado dos sensores, pedidos de movimento e motivos de bloqueio.
2. **Motor de regras físicas no próprio robô** avalia pedidos e limites com dados recentes, respeitando o período de controle. A segurança não pode depender do navegador nem da conexão Wi-Fi.
3. **Backend / bridge opcional** observa NetworkTables para agregar histórico, cálculos mais caros e comparações hipotéticas. Não deve ser a autoridade final para liberar mecanismos físicos.
4. **Aplicação web** mostra gráficos, modelo 3D, centro de massa estimado, cenários, alertas, explicações e logs. Permite comparar o efeito de velocidades e posições do braço, sem assumir controle irrestrito.
5. **Modo SIM** é o primeiro lugar para validar cenários; medições e limites reais dependem de CAD, massa, centro de massa, geometria do apoio, sensores calibrados e ensaios.

## Exemplo de fluxo

- Piloto solicita estender o braço.
- O roboRIO valida sensores e calcula o limite com o estado atual, segundo o modelo testado.
- Se não houver margem aceitável, o **roboRIO** bloqueia o mecanismo, publica a decisão e o motivo.
- O painel **observa e explica** o bloqueio, mesmo que a conexão com a web caia depois.

A hipótese "10 m/s" é útil como cenário de estudo, mas a base atual limita o modo rápido a 3,0 m/s e declara um teto físico teórico de 4,2 m/s; não considere 10 m/s uma capacidade comprovada do robô.

## Antes de construir

Precisamos de: dimensões e massa do robô, CAD dos mecanismos, distribuição de massa, localização de sensores, tipo e alcance do braço, estado da alimentação elétrica, controle e simulação disponíveis. Segurança real exige tolerâncias, detecção de dados desatualizados, proteções locais independentes e testes de falha.
