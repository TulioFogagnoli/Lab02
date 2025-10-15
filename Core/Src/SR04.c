//
// Created by ashkore on 23-9-2.
//

#include "sr04.h"
#include "stm32f4xx_hal.h" // Adicionado para HAL_RCC_GetPCLK1Freq()
#define DISTANCE_LIMIT 5000

void sr04_init(sr04_t *sr04_struct){
  // Enable trigger pin
  HAL_GPIO_WritePin(sr04_struct->trig_port, sr04_struct->trig_pin, GPIO_PIN_RESET);
  // Set input capture edge to rising
  __HAL_TIM_SET_CAPTUREPOLARITY(sr04_struct->echo_htim, sr04_struct->echo_channel, TIM_INPUTCHANNELPOLARITY_RISING);
  // Set capture flag to 0
  sr04_struct->capture_flag = 0;
  // Enable echo pin
  HAL_TIM_IC_Start_IT(sr04_struct->echo_htim, sr04_struct->echo_channel);
  HAL_TIM_Base_Start_IT(sr04_struct->echo_htim);
}

void sr04_trigger(sr04_t *sr04_struct){
  // Send pulse to trigger pin
  HAL_GPIO_WritePin(sr04_struct->trig_port, sr04_struct->trig_pin, GPIO_PIN_SET);
  for (uint8_t i=0; i<15; i++) { __NOP(); } // Pequeno delay de ~10us
  HAL_GPIO_WritePin(sr04_struct->trig_port, sr04_struct->trig_pin, GPIO_PIN_RESET);
}

// SUBSTITUA A FUNÇÃO ANTIGA POR ESTA
void sr04_read_distance(sr04_t *sr04_struct){
  // This function should be called in the timer input capture callback
  switch (sr04_struct->capture_flag){
    case 0:
      // Primeira borda (subida)
      sr04_struct->start_counter = HAL_TIM_ReadCapturedValue(sr04_struct->echo_htim, sr04_struct->echo_channel);
      sr04_struct->capture_flag = 1;
      sr04_struct->tim_update_count = 0;
      // Muda a polaridade para detectar a borda de descida
      __HAL_TIM_SET_CAPTUREPOLARITY(sr04_struct->echo_htim, sr04_struct->echo_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
      break;
    case 1:
      // Segunda borda (descida)
      sr04_struct->end_counter = HAL_TIM_ReadCapturedValue(sr04_struct->echo_htim, sr04_struct->echo_channel);
      sr04_struct->capture_flag = 0;

      // Calcula a diferença de ticks, considerando o overflow do timer
      uint32_t delta_ticks;
      if (sr04_struct->end_counter > sr04_struct->start_counter) {
          delta_ticks = (sr04_struct->end_counter - sr04_struct->start_counter) + sr04_struct->tim_update_count * (sr04_struct->echo_htim->Init.Period + 1);
      } else { // Em caso de overflow entre a captura de subida e descida
          delta_ticks = (sr04_struct->echo_htim->Init.Period - sr04_struct->start_counter + sr04_struct->end_counter) + (sr04_struct->tim_update_count -1) * (sr04_struct->echo_htim->Init.Period + 1);
      }
      
      // Fórmula de cálculo de distância corrigida e robusta
      // distance (mm) = (delta_ticks * (Prescaler + 1) * speed_of_sound_mm_s) / (timer_clock_hz * 2)
      // Usando 343m/s como velocidade do som
      uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
      if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
          timer_clock *= 2; // O clock do timer é dobrado se o prescaler APB1 > 1
      }
      
      // Usamos uint64_t para evitar overflow durante o cálculo intermediário
      uint64_t numerator = (uint64_t)delta_ticks * (sr04_struct->echo_htim->Init.Prescaler + 1) * 343000;
      sr04_struct->distance = numerator / (timer_clock * 2);

      // Limite de distância
      if(sr04_struct->distance > DISTANCE_LIMIT){
        sr04_struct->distance = sr04_struct->last_distance;
      }
      sr04_struct->last_distance = sr04_struct->distance;

      // Volta a polaridade para detectar a borda de subida do próximo pulso
      __HAL_TIM_SET_CAPTUREPOLARITY(sr04_struct->echo_htim, sr04_struct->echo_channel, TIM_INPUTCHANNELPOLARITY_RISING);
      break;
  }
}