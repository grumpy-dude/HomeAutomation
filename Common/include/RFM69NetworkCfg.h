#include "RFM69/RFM69.h"

const uint8_t Frequency_Band      = RF69_915MHZ;
const uint8_t The_Network_ID      = 101;
const uint8_t The_Gateway_Node_ID = 1;

#define USE_ENCRYPTION
const char The_Encryption_Key[RF69_ENCRYPTION_KEY_SIZE + 1] = "xxxxxxxxxxxxxxxx";
