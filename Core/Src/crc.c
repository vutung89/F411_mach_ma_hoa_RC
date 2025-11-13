#include "main.h"
#include "crc.h"

uint16_t calculate_crc16(uint8_t *data, size_t length) {
    uint8_t ucCRCHi = 0xFF;
    uint8_t ucCRCLo = 0xFF;
    uint8_t iIndex;

    for (size_t i = 0; i < length; i++) {
        iIndex = ucCRCLo ^ data[i];
        ucCRCLo = ucCRCHi ^ aucCRCHi[iIndex];
        ucCRCHi = aucCRCLo[iIndex];
    }

    return (ucCRCHi << 8) | ucCRCLo;
}
