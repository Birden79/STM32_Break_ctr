#pragma once

void Encoder_check(int16_t enc_value, uint16_t ppr_value, uint16_t freq);
int16_t GetCurrentRPM(void);
int16_t GetCurrentAccel(void);
void SetEncPPR(uint32_t ppr);
void Encoder_config(void);
