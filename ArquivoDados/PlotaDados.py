import pandas as pd
import matplotlib.pyplot as plt

# Lê o arquivo CSV gerado pelo datalogger
arquivo = "mpu6050.csv"
df = pd.read_csv(arquivo)

# Verifica as primeiras linhas para garantir leitura correta
print("Primeiras amostras:\n", df.head())

# Cria subplots: 2 linhas, 1 coluna
fig, axs = plt.subplots(2, 1, figsize=(10, 10))

# === Gráfico da Aceleração ===
axs[0].plot(df['amostra'], df['accel_x'], label='Accel X')
axs[0].plot(df['amostra'], df['accel_y'], label='Accel Y')
axs[0].plot(df['amostra'], df['accel_z'], label='Accel Z')
axs[0].set_title("Aceleração - MPU6050")
axs[0].set_xlabel("Tempo (amostras)")
axs[0].set_ylabel("Aceleração (g)")
axs[0].legend()
axs[0].grid(True)

# === Gráfico do Giroscópio ===
axs[1].plot(df['amostra'], df['gyro_x'], label='Gyro X')
axs[1].plot(df['amostra'], df['gyro_y'], label='Gyro Y')
axs[1].plot(df['amostra'], df['gyro_z'], label='Gyro Z')
axs[1].set_title("Velocidade Angular - MPU6050")
axs[1].set_xlabel("Tempo (amostras)")
axs[1].set_ylabel("Velocidade Angular (°/s)")
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show()