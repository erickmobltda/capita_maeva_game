# Capitã Maeva

Jogo de navegador em pixel art para crianças de 4 a 6 anos, que roda no navegador da TV, do computador ou do celular. O jogo inteiro é um arquivo só: [`index.html`](index.html).

Ele também pode ser jogado **com o corpo**: um controle feito com ESP32 e um sensor de movimento vira um teclado Bluetooth. Quando a criança pula, o personagem pula; quando ela agacha, ele abaixa.

## Como jogar

| Ação | Teclado | Celular | Controle de movimento |
|---|---|---|---|
| Pular | `Espaço` | botão na tela | pular |
| Abaixar | `↓` (segurar) | botão na tela | agachar (e levantar para soltar) |

Para jogar, abra o `index.html` no navegador. Se o GitHub Pages estiver ativado neste repositório, o jogo fica em `https://erickmobltda.github.io/capita_maeva_game/`.

## Controle de movimento (ESP32 + MPU6050)

O controle não precisa de nada instalado na TV. Ele aparece como um teclado Bluetooth chamado **Controle Pulo** e aperta `Espaço` e `↓` sozinho.

```
MPU6050 ──I²C──> ESP32 ──Bluetooth (teclado)──> TV / computador ──> index.html
```

O passo a passo completo está no [guia de montagem](controle/guia-controle-pulo.html): componentes, ligação, gravação, pareamento, calibração e leitura dos logs. Abra o arquivo no navegador.

### Peças

- Placa ESP32 DevKit (ESP32 clássico, 30 pinos)
- Módulo GY-521 (MPU6050)
- 4 jumpers fêmea-fêmea
- Cabo USB de dados (para gravar)
- Power bank pequeno e um case firme

### Ligação

| GY-521 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SCL | D22 |
| SDA | D21 |

Prenda o sensor firme dentro do case. Se ele balançar, mede o próprio chacoalho.

### Gravar o firmware

1. Na Arduino IDE, instale o pacote **esp32 by Espressif Systems**, versão 3.3.8 ou mais nova. A URL do Boards Manager é `https://espressif.github.io/arduino-esp32/package_esp32_index.json`.
2. Abra [`controle/controle_pulo/controle_pulo.ino`](controle/controle_pulo/controle_pulo.ino). Ele usa só bibliotecas do pacote oficial, nada para instalar.
3. Escolha a placa **ESP32 Dev Module**, a porta e clique em Upload. Se travar em `Connecting…`, segure o botão BOOT.
4. Abra o Serial Monitor em 115200, com final de linha **New Line**.

### Primeira vez

| Comando | Quando |
|---|---|
| `sensor` | Uma vez só. Corrige o erro de fábrica do sensor: deixe o controle parado em 6 posições, como as faces de um dado. |
| `teclas` | Testa o Bluetooth: aperta `Espaço` 2 vezes e segura `↓` por 2 s. Clique antes na janela do jogo. |
| `cal` | Com a criança: ela fica parada, agacha 3 vezes e pula 3 vezes. Os ajustes são aplicados e salvos sozinhos. |

Outros comandos: `ver` (estado e parâmetros), `desfazer` (volta a última `cal`), `padrao` (valores de fábrica), `log 0|1|2` e `<parametro> <valor>`, como `desce_vel 0.15`. Digite `ajuda` para ver a lista.

> As teclas vão para a **janela em foco**. Depois de digitar um comando no Serial Monitor, clique na janela do jogo antes de se mexer.

### Como funciona

- O giroscópio e o acelerômetro passam por um filtro Mahony a 200 Hz, que diz onde é "para cima" mesmo com o controle inclinado.
- **Pulo:** impulso para cima seguido de corpo "sem peso". O `Espaço` sai cerca de 35 ms depois que a criança sai do chão.
- **Agachar:** a `↓` é apertada depois de alguns centímetros de descida e solta assim que a criança começa a subir. O mergulho que ela faz antes de pular é ignorado.
- Sacudir o controle para os lados não conta.

Com um único sensor na mão, levantar o controle rápido com os braços equivale a pular.

## Estrutura

```
index.html                              o jogo
controle/
  guia-controle-pulo.html               guia de montagem e calibração
  controle_pulo/controle_pulo.ino       firmware do controle (Arduino IDE)
```
