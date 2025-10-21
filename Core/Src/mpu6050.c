#include "mpu6050.h"
#include "main.h" // Precisamos incluir o main.h para ter acesso ao HAL e ao handle hi2c1

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */

// Handle I2C global definido em main.c
extern I2C_HandleTypeDef hi2c1;

// Endereços de 8 bits (7 bits shiftados) para as funções HAL
#define MPU6050_ADDR_WRITE ((MPU6050_ADDR) << 1)
#define MPU6050_ADDR_READ  (((MPU6050_ADDR) << 1) | 0x01)

// O valor esperado do registrador WHO_AM_I
#define MPU_WHO_AM_I_VAL   0x68 

/**
  * @brief  Inicializa o MPU6050, agora com um teste WHO_AM_I.
  * @retval 0 se OK.
  * @retval -1 se falha de comunicação I2C ao ler WHO_AM_I.
  * @retval -2 se ID do WHO_AM_I estiver incorreto (não é um MPU6050).
  * @retval -3 a -7 se falha ao escrever configurações.
  */
int mpu6050_init(void) {
    uint8_t data;
    HAL_StatusTypeDef status;

    // --- NOVO: Teste "WHO_AM_I" ---
    // Tenta ler o registrador WHO_AM_I (0x75)
    // Isso confirma a fiação E se o chip está respondendo.
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_READ, MPU6050_REG_WHOAMI, 1, &data, 1, HAL_MAX_DELAY);
    
    if (status != HAL_OK) {
        return -1; // Falha de comunicação (Timeout, NACK, etc.)
    }
    if (data != MPU_WHO_AM_I_VAL) {
        return -2; // Comunicação OK, mas não é o MPU6050 (ID errado)
    }
    // --- FIM DO NOVO TESTE ---


    // 1. Acorda o MPU6050 (escreve 0x00 no registrador PWR1)
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_WRITE, MPU6050_REG_PWR1, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return -3;

    // 2. Configura o Digital Low Pass Filter (LPF) ~42 Hz (escreve 0x03 no CONFIG)
    data = 0x03;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_WRITE, MPU6050_REG_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return -4;

    // 3. Configura a escala do Giroscópio: ±250 dps (escreve 0x00 no GYROCFG)
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_WRITE, MPU6050_REG_GYROCFG, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return -5;

    // 4. Configura a escala do Acelerômetro: ±2g (escreve 0x00 no ACCELCFG)
    data = 0x00;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_WRITE, MPU6050_REG_ACCELCFG, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return -6;

    // 5. Configura o Sample Rate: 100 Hz (SampleRate = 1k / (1 + SMPLRT_DIV))
    data = 9;
    status = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_WRITE, MPU6050_REG_SMPLRT, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return -7;

    return 0; // Sucesso
}

/**
  * @brief  Lê todos os 14 bytes de dados (Accel, Temp, Gyro) usando HAL.
  * @param  out: Ponteiro para a struct de saída dos dados crus.
  * @retval 0 se OK, -1 em caso de erro.
  */
int mpu6050_read_all(mpu6050_raw_t *out) {
    uint8_t buf[14];
    HAL_StatusTypeDef status;

    // Lê 14 bytes começando do registrador MPU6050_REG_ACCEL (0x3B)
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_READ, MPU6050_REG_ACCEL, 1, buf, 14, HAL_MAX_DELAY);
    
    if (status != HAL_OK) {
        // Erro na leitura I2C
        return -1;
    }

    // Organiza os bytes (Big Endian) na struct
    out->ax = (int16_t)((buf[0] << 8) | buf[1]);
    out->ay = (int16_t)((buf[2] << 8) | buf[3]);
    out->az = (int16_t)((buf[4] << 8) | buf[5]);
    out->temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    out->gx = (int16_t)((buf[8] << 8) | buf[9]);
    out->gy = (int16_t)((buf[10] << 8) | buf[11]);
    out->gz = (int16_t)((buf[12] << 8) | buf[13]);

    return 0; // Sucesso
}