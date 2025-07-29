# Projeto: Datalogger de Movimento com MPU6050 e Raspberry Pi Pico W

Este projeto implementa um datalogger portátil capaz de capturar dados de movimento (aceleração e giroscópio) utilizando o sensor MPU6050. Os dados são armazenados em formato `.csv` em um cartão microSD. O sistema inclui display OLED, LED RGB, buzzer e botões físicos para interação local.

## 📦 Funcionalidades

- **Leitura de dados do sensor MPU6050**
- **Armazenamento dos dados em cartão SD no formato `.csv`**
- **Exibição de informações em tempo real no display OLED SSD1306**
- **Sinalização de estados do sistema com LED RGB e buzzer**
- **Controle por comandos via terminal serial e botões físicos**
- **Sistema de arquivos baseado em FatFs**
- **Exportação dos dados para análise posterior em Python**

## ⚙️ Hardware Utilizado

- Raspberry Pi Pico W
- Sensor MPU6050 (I2C)
- Display OLED SSD1306 (I2C)
- Cartão microSD com módulo SPI
- LED RGB (3 GPIOs)
- Buzzer (PWM)
- 2 botões físicos (GPIO com pull-up)

## 🖥️ Interface do Terminal Serial

Comandos disponíveis:

| Comando | Função |
|--------|--------|
| `a` | Montar o cartão SD |
| `b` | Desmontar o cartão SD |
| `c` | Listar arquivos no SD |
| `d` | Mostrar conteúdo do arquivo `mpu6050.csv` |
| `e` | Obter espaço livre no SD |
| `f` | Iniciar/parar captura de dados do IMU |
| `g` | Formatar o cartão SD |
| `h` | Mostrar ajuda |

## 🔧 Pinos Utilizados

| Componente | Pinos |
|-----------|-------|
| MPU6050 (I2C0) | SDA: GP0, SCL: GP1 |
| Display OLED (I2C1) | SDA: GP14, SCL: GP15 |
| LED RGB | R: GP13, G: GP11, B: GP12 |
| Buzzer (PWM) | GP21 |
| Botão Captura | GP5 |
| Botão SD | GP6 |

## 🎨 Feedback Visual e Auditivo

- **LED RGB**:
  - Amarelo: Inicializando / Montando cartão
  - Verde: Cartão SD pronto para uso
  - Vermelho: Captura em andamento
  - Azul piscando: Acessando SD
  - Roxo piscando: Erro
  - Branco: SD desmontado

- **Display OLED**:
  - Exibe status do sistema
  - Exibe "Gravando..." durante captura
  - Exibe mensagens como "Montado", "Erro", "Concluído"

- **Buzzer**:
  - 1 beep: Início da captura
  - 2 beeps: Fim da captura

## 📊 Estrutura do Arquivo CSV

O arquivo gerado (`mpu6050.csv`) tem a seguinte estrutura:

```csv
amostra,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
1,0.01,-0.02,1.00,0.50,-0.12,0.75
2,...
...
```

- Aceleração normalizada em `g` (gravidade)
- Giroscópio normalizado em `°/s` (graus por segundo)

## 📈 Análise dos Dados

Use o script Python incluso no repositório para gerar gráficos com os dados do arquivo `.csv` salvo:

```bash
cd ArquivoDados
python .\PlotaDados.py
```

## 🚀 Como Usar

1. Grave o firmware na Raspberry Pi Pico
2. Conecte os sensores e periféricos conforme os pinos
3. Acesse o terminal serial via `PuTTY` ou similar
4. Use os comandos ou botões para operar o sistema

## 👨‍💻 Autor

**Leonam Rabelo** – Projeto de Sistemas Embarcados com a placa BitDogLab
