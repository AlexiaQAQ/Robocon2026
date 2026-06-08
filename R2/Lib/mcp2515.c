#include "mcp2515.h"
#include "mcp2515_consts.h"
#include "main.h"

#define SPI_TIMEOUT 10

void MCP2515_Init(MCP2515_HandleTypeDef* hcan, SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin) {
    hcan->hspi = hspi;
    hcan->cs_port = cs_port;
    hcan->cs_pin = cs_pin;
}

uint8_t MCP2515_SPI_transfer(MCP2515_HandleTypeDef* hcan, uint8_t txByte) {
    uint8_t rxByte;
    HAL_SPI_TransmitReceive(hcan->hspi, &txByte, &rxByte, 1, SPI_TIMEOUT);
    return rxByte;
}

void MCP2515_Set_Register(MCP2515_HandleTypeDef* hcan, uint8_t reg, uint8_t value) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
    MCP2515_SPI_transfer(hcan, INSTRUCTION_WRITE);
    MCP2515_SPI_transfer(hcan, reg);
    MCP2515_SPI_transfer(hcan, value);
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);
}

void MCP2515_Set_Registers(MCP2515_HandleTypeDef* hcan, uint8_t reg, uint8_t values[], uint8_t n) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
    MCP2515_SPI_transfer(hcan, INSTRUCTION_WRITE);
    MCP2515_SPI_transfer(hcan, reg);
    for (uint8_t i = 0; i < n; i++) {
        MCP2515_SPI_transfer(hcan, values[i]);
    }

    //  HAL_SPI_Transmit(SPI_CAN, values, n, SPI_TIMEOUT);
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);
}

void MCP2515_Load_Tx(MCP2515_HandleTypeDef* hcan, uint8_t reg, uint8_t values[], uint8_t n) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
    // MCP2515_SPI_transfer(hcan, INSTRUCTION_WRITE);
    MCP2515_SPI_transfer(hcan, reg);
    for (uint8_t i = 0; i < n; i++) {
        MCP2515_SPI_transfer(hcan, values[i]);
    }

    //  HAL_SPI_Transmit(SPI_CAN, values, n, SPI_TIMEOUT);
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);
}

void MCP2515_Modify_Register(MCP2515_HandleTypeDef* hcan, uint8_t reg, uint8_t mask, uint8_t data) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
    MCP2515_SPI_transfer(hcan, INSTRUCTION_BITMOD);
    MCP2515_SPI_transfer(hcan, reg);
    MCP2515_SPI_transfer(hcan, mask);
    MCP2515_SPI_transfer(hcan, data);
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);
}

uint8_t MCP2515_Read_Register(MCP2515_HandleTypeDef* hcan, REGISTER reg) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
    MCP2515_SPI_transfer(hcan, INSTRUCTION_READ);
    MCP2515_SPI_transfer(hcan, reg);
    uint8_t ret = MCP2515_SPI_transfer(hcan, 0x00);
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);

    return ret;
}

void MCP2515_Read_Registers(MCP2515_HandleTypeDef* hcan, REGISTER reg, uint8_t values[], uint8_t n) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
    MCP2515_SPI_transfer(hcan, INSTRUCTION_READ);
    MCP2515_SPI_transfer(hcan, reg);
    // mcp2515 has auto-increment of address-pointer
    for (uint8_t i = 0; i < n; i++) {
        values[i] = MCP2515_SPI_transfer(hcan, 0x00);
        // HAL_SPI_Receive(&hspi4, values, n, SPI_TIMEOUT);  //Todo, check if the 0x00 from above is needed
    }
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);
}

void MCP2515_Read_Rx(MCP2515_HandleTypeDef* hcan, REGISTER reg, uint8_t values[], uint8_t n) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
    MCP2515_SPI_transfer(hcan, reg);
    // mcp2515 has auto-increment of address-pointer
    for (uint8_t i = 0; i < n; i++) {
        values[i] = MCP2515_SPI_transfer(hcan, 0x00);
        // HAL_SPI_Receive(&hspi4, values, n, SPI_TIMEOUT);  //Todo, check if the 0x00 from above is needed
    }
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);
}

CAN_Error MCP2515_Set_Mode(MCP2515_HandleTypeDef* hcan, MCP2515_Mode_t mode) {
    unsigned long endTime = HAL_GetTick() + 10;
    uint8_t modeMatch = 0;
    while (HAL_GetTick() < endTime) {
        MCP2515_Modify_Register(hcan, MCP_CANCTRL, CANCTRL_REQOP, mode);
        uint8_t newmode = MCP2515_Read_Register(hcan, MCP_CANSTAT);
        newmode &= CANSTAT_OPMOD;

        modeMatch = newmode == mode;

        if (modeMatch) {
            break;
        }
    }

    return modeMatch ? ERROR_OK : ERROR_FAIL;
}

CAN_Error MCP2515_Set_Config_Mode(MCP2515_HandleTypeDef* hcan) {
    return MCP2515_Set_Mode(hcan, (MCP2515_Mode_t)CANCTRL_REQOP_CONFIG);
}

void MCP2515_Prepare_Id(uint8_t* buffer, uint8_t ext, uint32_t id) {
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext) {
        buffer[MCP_EID0] = (uint8_t)(canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t)(canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t)(canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t)(canid >> 5);
    } else {
        buffer[MCP_SIDH] = (uint8_t)(canid >> 3);
        buffer[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

uint8_t MCP2515_Get_Status(MCP2515_HandleTypeDef* hcan) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
    MCP2515_SPI_transfer(hcan, INSTRUCTION_READ_STATUS);
    uint8_t i = MCP2515_SPI_transfer(hcan, 0x00);
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);
    return i;
}

CAN_Error MCP2515_Set_Filter_Mask(MCP2515_HandleTypeDef* hcan, MASK mask, uint8_t ext, uint32_t ulData) {
    CAN_Error res = MCP2515_Set_Config_Mode(hcan);
    if (res != ERROR_OK) {
        return res;
    }

    uint8_t tbufdata[4];
    MCP2515_Prepare_Id(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask) {
        case MASK0:
            reg = MCP_RXM0SIDH;
            break;
        case MASK1:
            reg = MCP_RXM1SIDH;
            break;
        default:
            return ERROR_FAIL;
    }

    MCP2515_Set_Registers(hcan, reg, tbufdata, 4);

    return ERROR_OK;
}

CAN_Error MCP2515_Set_Filter(MCP2515_HandleTypeDef* hcan, RXF num, uint8_t ext, uint32_t ulData) {
    CAN_Error res = MCP2515_Set_Config_Mode(hcan);
    if (res != ERROR_OK) {
        return res;
    }

    REGISTER reg;

    switch (num) {
        case RXF0:
            reg = MCP_RXF0SIDH;
            break;
        case RXF1:
            reg = MCP_RXF1SIDH;
            break;
        case RXF2:
            reg = MCP_RXF2SIDH;
            break;
        case RXF3:
            reg = MCP_RXF3SIDH;
            break;
        case RXF4:
            reg = MCP_RXF4SIDH;
            break;
        case RXF5:
            reg = MCP_RXF5SIDH;
            break;
        default:
            return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    MCP2515_Prepare_Id(tbufdata, ext, ulData);
    MCP2515_Set_Registers(hcan, reg, tbufdata, 4);

    return ERROR_OK;
}

CAN_Error MCP2515_Reset(MCP2515_HandleTypeDef* hcan) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
    MCP2515_SPI_transfer(hcan, INSTRUCTION_RESET);
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);

    HAL_Delay(10);

    uint8_t zeros[14] = {0};

    MCP2515_Set_Registers(hcan, MCP_TXB0CTRL, zeros, 14);
    MCP2515_Set_Registers(hcan, MCP_TXB1CTRL, zeros, 14);
    MCP2515_Set_Registers(hcan, MCP_TXB2CTRL, zeros, 14);

    MCP2515_Set_Register(hcan, MCP_RXB0CTRL, 0);
    MCP2515_Set_Register(hcan, MCP_RXB1CTRL, 0);

    MCP2515_Set_Register(hcan, MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    MCP2515_Modify_Register(hcan, MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                            RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
    MCP2515_Modify_Register(hcan, MCP_RXB1CTRL, RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                            RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t ext = (i == 1);
        CAN_Error result = MCP2515_Set_Filter(hcan, filters[i], ext, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    MASK masks[] = {MASK0, MASK1};
    for (int i = 0; i < 2; i++) {
        CAN_Error result = MCP2515_Set_Filter_Mask(hcan, masks[i], 1, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    return ERROR_OK;
}

CAN_Error MCP2515_Set_Pin_Control(MCP2515_HandleTypeDef* hcan, bool B1BFS, bool B0BFS, bool B1BFE, bool B0BFE, bool B1BFM, bool B0BFM) {
    CAN_Error error = MCP2515_Set_Config_Mode(hcan);
    if (error != ERROR_OK) {
        return error;
    }
    MCP2515_Set_Register(hcan, MCP_BFPCTRL, (B1BFS << 5) | (B0BFS << 4) | (B1BFE << 3) | (B0BFE << 2) | (B1BFM << 1) | B0BFM);
    return ERROR_OK;
}

CAN_Error MCP2515_Set_Interrupt(MCP2515_HandleTypeDef* hcan, bool MERRIE, bool WAKIE, bool ERRIE, bool TX2IE, bool TX1IE, bool TX0IE, bool RX1IE, bool RX0IE) {
    CAN_Error error = MCP2515_Set_Config_Mode(hcan);
    if (error != ERROR_OK) {
        return error;
    }
    MCP2515_Set_Register(hcan, MCP_CANINTE, (MERRIE << 7) | (WAKIE << 6) | (ERRIE << 5) | (TX2IE << 4) | (TX1IE << 3) | (TX0IE << 2) | (RX1IE << 1) | RX0IE);
    return ERROR_OK;
}

CAN_Error MCP2515_Set_Bitrate_Clock(MCP2515_HandleTypeDef* hcan, CAN_SPEED canSpeed, CAN_CLOCK canClock) {
    CAN_Error error = MCP2515_Set_Config_Mode(hcan);
    if (error != ERROR_OK) {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock) {
        case (MCP_8MHZ):
            switch (canSpeed) {
                case (CAN_5KBPS): //   5KBPS
                    cfg1 = MCP_8MHz_5kBPS_CFG1;
                    cfg2 = MCP_8MHz_5kBPS_CFG2;
                    cfg3 = MCP_8MHz_5kBPS_CFG3;
                    break;

                case (CAN_10KBPS): //  10KBPS
                    cfg1 = MCP_8MHz_10kBPS_CFG1;
                    cfg2 = MCP_8MHz_10kBPS_CFG2;
                    cfg3 = MCP_8MHz_10kBPS_CFG3;
                    break;

                case (CAN_20KBPS): //  20KBPS
                    cfg1 = MCP_8MHz_20kBPS_CFG1;
                    cfg2 = MCP_8MHz_20kBPS_CFG2;
                    cfg3 = MCP_8MHz_20kBPS_CFG3;
                    break;

                case (CAN_31K25BPS): //  31.25KBPS
                    cfg1 = MCP_8MHz_31k25BPS_CFG1;
                    cfg2 = MCP_8MHz_31k25BPS_CFG2;
                    cfg3 = MCP_8MHz_31k25BPS_CFG3;
                    break;

                case (CAN_33KBPS): //  33.333KBPS
                    cfg1 = MCP_8MHz_33k3BPS_CFG1;
                    cfg2 = MCP_8MHz_33k3BPS_CFG2;
                    cfg3 = MCP_8MHz_33k3BPS_CFG3;
                    break;

                case (CAN_40KBPS): //  40Kbps
                    cfg1 = MCP_8MHz_40kBPS_CFG1;
                    cfg2 = MCP_8MHz_40kBPS_CFG2;
                    cfg3 = MCP_8MHz_40kBPS_CFG3;
                    break;

                case (CAN_50KBPS): //  50Kbps
                    cfg1 = MCP_8MHz_50kBPS_CFG1;
                    cfg2 = MCP_8MHz_50kBPS_CFG2;
                    cfg3 = MCP_8MHz_50kBPS_CFG3;
                    break;

                case (CAN_80KBPS): //  80Kbps
                    cfg1 = MCP_8MHz_80kBPS_CFG1;
                    cfg2 = MCP_8MHz_80kBPS_CFG2;
                    cfg3 = MCP_8MHz_80kBPS_CFG3;
                    break;

                case (CAN_100KBPS): // 100Kbps
                    cfg1 = MCP_8MHz_100kBPS_CFG1;
                    cfg2 = MCP_8MHz_100kBPS_CFG2;
                    cfg3 = MCP_8MHz_100kBPS_CFG3;
                    break;

                case (CAN_125KBPS): // 125Kbps
                    cfg1 = MCP_8MHz_125kBPS_CFG1;
                    cfg2 = MCP_8MHz_125kBPS_CFG2;
                    cfg3 = MCP_8MHz_125kBPS_CFG3;
                    break;

                case (CAN_200KBPS): // 200Kbps
                    cfg1 = MCP_8MHz_200kBPS_CFG1;
                    cfg2 = MCP_8MHz_200kBPS_CFG2;
                    cfg3 = MCP_8MHz_200kBPS_CFG3;
                    break;

                case (CAN_250KBPS): // 250Kbps
                    cfg1 = MCP_8MHz_250kBPS_CFG1;
                    cfg2 = MCP_8MHz_250kBPS_CFG2;
                    cfg3 = MCP_8MHz_250kBPS_CFG3;
                    break;

                case (CAN_500KBPS): // 500Kbps
                    cfg1 = MCP_8MHz_500kBPS_CFG1;
                    cfg2 = MCP_8MHz_500kBPS_CFG2;
                    cfg3 = MCP_8MHz_500kBPS_CFG3;
                    break;

                case (CAN_1000KBPS): //   1Mbps
                    cfg1 = MCP_8MHz_1000kBPS_CFG1;
                    cfg2 = MCP_8MHz_1000kBPS_CFG2;
                    cfg3 = MCP_8MHz_1000kBPS_CFG3;
                    break;

                default:
                    set = 0;
                    break;
            }
            break;

        case (MCP_16MHZ):
            switch (canSpeed) {
                case (CAN_5KBPS): //   5Kbps
                    cfg1 = MCP_16MHz_5kBPS_CFG1;
                    cfg2 = MCP_16MHz_5kBPS_CFG2;
                    cfg3 = MCP_16MHz_5kBPS_CFG3;
                    break;

                case (CAN_10KBPS): //  10Kbps
                    cfg1 = MCP_16MHz_10kBPS_CFG1;
                    cfg2 = MCP_16MHz_10kBPS_CFG2;
                    cfg3 = MCP_16MHz_10kBPS_CFG3;
                    break;

                case (CAN_20KBPS): //  20Kbps
                    cfg1 = MCP_16MHz_20kBPS_CFG1;
                    cfg2 = MCP_16MHz_20kBPS_CFG2;
                    cfg3 = MCP_16MHz_20kBPS_CFG3;
                    break;

                case (CAN_33KBPS): //  33.333Kbps
                    cfg1 = MCP_16MHz_33k3BPS_CFG1;
                    cfg2 = MCP_16MHz_33k3BPS_CFG2;
                    cfg3 = MCP_16MHz_33k3BPS_CFG3;
                    break;

                case (CAN_40KBPS): //  40Kbps
                    cfg1 = MCP_16MHz_40kBPS_CFG1;
                    cfg2 = MCP_16MHz_40kBPS_CFG2;
                    cfg3 = MCP_16MHz_40kBPS_CFG3;
                    break;

                case (CAN_50KBPS): //  50Kbps
                    cfg1 = MCP_16MHz_50kBPS_CFG1;
                    cfg2 = MCP_16MHz_50kBPS_CFG2;
                    cfg3 = MCP_16MHz_50kBPS_CFG3;
                    break;

                case (CAN_80KBPS): //  80Kbps
                    cfg1 = MCP_16MHz_80kBPS_CFG1;
                    cfg2 = MCP_16MHz_80kBPS_CFG2;
                    cfg3 = MCP_16MHz_80kBPS_CFG3;
                    break;

                case (CAN_83K3BPS): //  83.333Kbps
                    cfg1 = MCP_16MHz_83k3BPS_CFG1;
                    cfg2 = MCP_16MHz_83k3BPS_CFG2;
                    cfg3 = MCP_16MHz_83k3BPS_CFG3;
                    break;

                case (CAN_100KBPS): // 100Kbps
                    cfg1 = MCP_16MHz_100kBPS_CFG1;
                    cfg2 = MCP_16MHz_100kBPS_CFG2;
                    cfg3 = MCP_16MHz_100kBPS_CFG3;
                    break;

                case (CAN_125KBPS): // 125Kbps
                    cfg1 = MCP_16MHz_125kBPS_CFG1;
                    cfg2 = MCP_16MHz_125kBPS_CFG2;
                    cfg3 = MCP_16MHz_125kBPS_CFG3;
                    break;

                case (CAN_200KBPS): // 200Kbps
                    cfg1 = MCP_16MHz_200kBPS_CFG1;
                    cfg2 = MCP_16MHz_200kBPS_CFG2;
                    cfg3 = MCP_16MHz_200kBPS_CFG3;
                    break;

                case (CAN_250KBPS): // 250Kbps
                    cfg1 = MCP_16MHz_250kBPS_CFG1;
                    cfg2 = MCP_16MHz_250kBPS_CFG2;
                    cfg3 = MCP_16MHz_250kBPS_CFG3;
                    break;

                case (CAN_500KBPS): // 500Kbps
                    cfg1 = MCP_16MHz_500kBPS_CFG1;
                    cfg2 = MCP_16MHz_500kBPS_CFG2;
                    cfg3 = MCP_16MHz_500kBPS_CFG3;
                    break;

                case (CAN_1000KBPS): //   1Mbps
                    cfg1 = MCP_16MHz_1000kBPS_CFG1;
                    cfg2 = MCP_16MHz_1000kBPS_CFG2;
                    cfg3 = MCP_16MHz_1000kBPS_CFG3;
                    break;

                default:
                    set = 0;
                    break;
            }
            break;

        case (MCP_20MHZ):
            switch (canSpeed) {
                case (CAN_33KBPS): //  33.333Kbps
                    cfg1 = MCP_20MHz_33k3BPS_CFG1;
                    cfg2 = MCP_20MHz_33k3BPS_CFG2;
                    cfg3 = MCP_20MHz_33k3BPS_CFG3;
                    break;

                case (CAN_40KBPS): //  40Kbps
                    cfg1 = MCP_20MHz_40kBPS_CFG1;
                    cfg2 = MCP_20MHz_40kBPS_CFG2;
                    cfg3 = MCP_20MHz_40kBPS_CFG3;
                    break;

                case (CAN_50KBPS): //  50Kbps
                    cfg1 = MCP_20MHz_50kBPS_CFG1;
                    cfg2 = MCP_20MHz_50kBPS_CFG2;
                    cfg3 = MCP_20MHz_50kBPS_CFG3;
                    break;

                case (CAN_80KBPS): //  80Kbps
                    cfg1 = MCP_20MHz_80kBPS_CFG1;
                    cfg2 = MCP_20MHz_80kBPS_CFG2;
                    cfg3 = MCP_20MHz_80kBPS_CFG3;
                    break;

                case (CAN_83K3BPS): //  83.333Kbps
                    cfg1 = MCP_20MHz_83k3BPS_CFG1;
                    cfg2 = MCP_20MHz_83k3BPS_CFG2;
                    cfg3 = MCP_20MHz_83k3BPS_CFG3;
                    break;

                case (CAN_100KBPS): // 100Kbps
                    cfg1 = MCP_20MHz_100kBPS_CFG1;
                    cfg2 = MCP_20MHz_100kBPS_CFG2;
                    cfg3 = MCP_20MHz_100kBPS_CFG3;
                    break;

                case (CAN_125KBPS): // 125Kbps
                    cfg1 = MCP_20MHz_125kBPS_CFG1;
                    cfg2 = MCP_20MHz_125kBPS_CFG2;
                    cfg3 = MCP_20MHz_125kBPS_CFG3;
                    break;

                case (CAN_200KBPS): // 200Kbps
                    cfg1 = MCP_20MHz_200kBPS_CFG1;
                    cfg2 = MCP_20MHz_200kBPS_CFG2;
                    cfg3 = MCP_20MHz_200kBPS_CFG3;
                    break;

                case (CAN_250KBPS): // 250Kbps
                    cfg1 = MCP_20MHz_250kBPS_CFG1;
                    cfg2 = MCP_20MHz_250kBPS_CFG2;
                    cfg3 = MCP_20MHz_250kBPS_CFG3;
                    break;

                case (CAN_500KBPS): // 500Kbps
                    cfg1 = MCP_20MHz_500kBPS_CFG1;
                    cfg2 = MCP_20MHz_500kBPS_CFG2;
                    cfg3 = MCP_20MHz_500kBPS_CFG3;
                    break;

                case (CAN_1000KBPS): //   1Mbps
                    cfg1 = MCP_20MHz_1000kBPS_CFG1;
                    cfg2 = MCP_20MHz_1000kBPS_CFG2;
                    cfg3 = MCP_20MHz_1000kBPS_CFG3;
                    break;

                default:
                    set = 0;
                    break;
            }
            break;

        default:
            set = 0;
            break;
    }

    if (set) {
        MCP2515_Set_Register(hcan, MCP_CNF1, cfg1);
        MCP2515_Set_Register(hcan, MCP_CNF2, cfg2);
        MCP2515_Set_Register(hcan, MCP_CNF3, cfg3);
        return ERROR_OK;
    }

    return ERROR_FAIL;
}

CAN_Error MCP2515_Set_Transmit_Priority(MCP2515_HandleTypeDef* hcan, MCP2515_Priority_t txb0, MCP2515_Priority_t txb1, MCP2515_Priority_t txb2)
{
    MCP2515_Set_Register(hcan, MCP_TXB0CTRL, txb0);
    MCP2515_Set_Register(hcan, MCP_TXB1CTRL, txb1);
    MCP2515_Set_Register(hcan, MCP_TXB2CTRL, txb2);
		return ERROR_OK;
}

void MCP2515_Request_To_Send(MCP2515_HandleTypeDef* hcan, uint8_t instruction) {
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_RESET);
    MCP2515_SPI_transfer(hcan, instruction);
    HAL_GPIO_WritePin(hcan->cs_port, hcan->cs_pin, GPIO_PIN_SET);
}

CAN_Error MCP2515_Send_Message_To(MCP2515_HandleTypeDef* hcan, TXBn txbn, can_frame_t* frame)
// TXBm is just 0,1,2 for txbox number
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

    // Todo, fix these magic numbers, but not with something as awful as the og arduino library
    uint8_t load_addr = (2 * txbn) | 0x40;

    uint8_t rts_addr = (1 << txbn) | 0x80;

    uint8_t data[13];

    uint8_t ext = !!(frame->can_id & CAN_EFF_FLAG);
    uint8_t rtr = !!(frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    MCP2515_Prepare_Id(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    for (int i = 0; i < frame->can_dlc; i++) {
        data[MCP_DATA + i] = frame->data[i];
    }

    // memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    MCP2515_Load_Tx(hcan, load_addr, data, 5 + frame->can_dlc);
    // MCP2515_Set_Registers(hcan, load_addr, data, 5 + frame->can_dlc);

    // MCP2515_Modify_Register(hcan, txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);
    // MCP2515_Modify_Register(hcan, rts_addr, TXB_TXREQ, TXB_TXREQ);
    MCP2515_Request_To_Send(hcan, rts_addr);
    // MCP2515_Set_Register(hcan, rts_addr, TXB_TXREQ);

    uint8_t ctrl = MCP2515_Read_Register(hcan, (REGISTER)rts_addr);
    if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0) {
        return ERROR_FAILTX;
    }
    return ERROR_OK;
}

CAN_Error MCP2515_Send_Message(MCP2515_HandleTypeDef* hcan, can_frame_t* frame) 
{
		uint8_t ctrlval,i;
	
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

    for (i = 0; i < N_TXBUFFERS; i++) 
		{
        ctrlval = MCP2515_Read_Register(hcan, (REGISTER)(i + 3) << 4);
        if ((ctrlval & TXB_TXREQ) == 0) 
				{
            return MCP2515_Send_Message_To(hcan, i, frame);
        }
    }

    return ERROR_ALLTXBUSY;
}

CAN_Error MCP2515_Read_Message_From(MCP2515_HandleTypeDef* hcan, RXBn rxbn, can_frame_t* frame) {
    uint8_t readCommand = (rxbn << 2) | 0x90;
    rx_reg_t rxReg;

    MCP2515_Read_Rx(hcan, readCommand, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));

    uint32_t id = (rxReg.rx_reg_array[MCP_SIDH] << 3) + (rxReg.rx_reg_array[MCP_SIDL] >> 5);

    if ((rxReg.rx_reg_array[MCP_SIDL] & TXB_EXIDE_MASK) == TXB_EXIDE_MASK) {
        id = (id << 2) + (rxReg.rx_reg_array[MCP_SIDL] & 0x03);
        id = (id << 8) + rxReg.rx_reg_array[MCP_EID8];
        id = (id << 8) + rxReg.rx_reg_array[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (rxReg.rx_reg_array[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN) {
        return ERROR_FAIL;
    }

    // 0x60 or 0x70
    uint8_t ctrl = MCP2515_Read_Register(hcan, (rxbn + 6) << 4);
    if (ctrl & RXBnCTRL_RTR) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    frame->data[0] = rxReg.RXBnD0;
    frame->data[1] = rxReg.RXBnD1;
    frame->data[2] = rxReg.RXBnD2;
    frame->data[3] = rxReg.RXBnD3;
    frame->data[4] = rxReg.RXBnD4;
    frame->data[5] = rxReg.RXBnD5;
    frame->data[6] = rxReg.RXBnD6;
    frame->data[7] = rxReg.RXBnD7;

    // Clear the inbox interrupt, 0x1 or 0x2
    MCP2515_Modify_Register(hcan, MCP_CANINTF, rxbn + 1, 0);

    return ERROR_OK;
}

CAN_Error MCP2515_Read_Message(MCP2515_HandleTypeDef* hcan, can_frame_t* frame) {
    CAN_Error rc;
    uint8_t stat = MCP2515_Get_Status(hcan);

    if (stat & STAT_RX0IF) {
        rc = MCP2515_Read_Message_From(hcan, RXB0, frame);
    } else if (stat & STAT_RX1IF) {
        rc = MCP2515_Read_Message_From(hcan, RXB1, frame);
    } else {
        rc = ERROR_NOMSG;
    }

    return rc;
}

uint8_t MCP2515_Check_Receive(MCP2515_HandleTypeDef* hcan) {
    uint8_t res = MCP2515_Get_Status(hcan);
    if (res & STAT_RXIF_MASK) {
        return 1;
    }

    return 0;
}

uint8_t MCP2515_Get_Error_Flags(MCP2515_HandleTypeDef* hcan) {
    return MCP2515_Read_Register(hcan, MCP_EFLG);
}

uint8_t MCP2515_Check_Error(MCP2515_HandleTypeDef* hcan) {
    uint8_t eflg = MCP2515_Get_Error_Flags(hcan);

    if (eflg & EFLG_ERRORMASK) {
        return 1;
    } else {
        return 0;
    }
}

void MCP2515_Clear_RXn_OVR_Flags(MCP2515_HandleTypeDef* hcan) {
    MCP2515_Modify_Register(hcan, MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t MCP2515_Get_Interrupts(MCP2515_HandleTypeDef* hcan) {
    return MCP2515_Read_Register(hcan, MCP_CANINTF);
}

void MCP2515_Clear_Interrupts(MCP2515_HandleTypeDef* hcan) {
    MCP2515_Set_Register(hcan, MCP_CANINTF, 0);
}

uint8_t MCP2515_Get_Interrupt_Mask(MCP2515_HandleTypeDef* hcan) {
    return MCP2515_Read_Register(hcan, MCP_CANINTE);
}

void MCP2515_Clear_TX_Interrupts(MCP2515_HandleTypeDef* hcan) {
    MCP2515_Modify_Register(hcan, MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void MCP2515_Clear_RXn_OVR(MCP2515_HandleTypeDef* hcan) {
    uint8_t eflg = MCP2515_Get_Error_Flags(hcan);
    if (eflg != 0) {
        MCP2515_Clear_RXn_OVR_Flags(hcan);
        MCP2515_Clear_Interrupts(hcan);
        // MCP2515_Modify_Register(hcan, MCP_CANINTF, CANINTF_ERRIF, 0);
    }
}

void MCP2515_Clear_MERR(MCP2515_HandleTypeDef* hcan) {
    // MCP2515_Modify_Register(hcan, MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    // clearInterrupts();
    MCP2515_Modify_Register(hcan, MCP_CANINTF, CANINTF_MERRF, 0);
}

void MCP2515_Clear_ERRIF(MCP2515_HandleTypeDef* hcan) {
    // MCP2515_Modify_Register(hcan, MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    // clearInterrupts();
    MCP2515_Modify_Register(hcan, MCP_CANINTF, CANINTF_ERRIF, 0);
}


CAN_Error mcp_error;

MCP2515_HandleTypeDef hcan3,hcan4,hcan5;


void mcp2515_sys_init(MCP2515_HandleTypeDef *can,SPI_HandleTypeDef* hspi ,GPIO_TypeDef* cs_port, uint16_t cs_pin)
{
	
	MCP2515_Init(can,hspi,cs_port,cs_pin);
	mcp_error = MCP2515_Reset(can);
	HAL_Delay(50);
	mcp_error = MCP2515_Set_Bitrate_Clock(can, CAN_1000KBPS, MCP_8MHZ);
	// 设置 MASK0 和 MASK1 为 0，表示所有位“不关心”
	MCP2515_Set_Filter_Mask(can, MASK0, 0, 0x00000000);  // 不关心任何位
	MCP2515_Set_Filter_Mask(can, MASK1, 0, 0x00000000);

	// 设置所有 6 个滤波器为 0（任意 ID 都能通过）
	MCP2515_Set_Filter(can, RXF0, 0, 0x00000000);
	MCP2515_Set_Filter(can, RXF1, 0, 0x00000000);
	MCP2515_Set_Filter(can, RXF2, 0, 0x00000000);
	MCP2515_Set_Filter(can, RXF3, 0, 0x00000000);
	MCP2515_Set_Filter(can, RXF4, 0, 0x00000000);
	MCP2515_Set_Filter(can, RXF5, 0, 0x00000000);
		
	mcp_error = MCP2515_Set_Mode(can, MODE_NORMAL);
	
}


//	MCP2515	CAN 发送函数
void mcp2515_can_tx_data(MCP2515_HandleTypeDef *can, uint16_t id, uint8_t *data, uint32_t len)
{
	can->Can_Tx.can_id = id;
	can->Can_Tx.can_dlc = len;
	
	can->Can_Tx.data[0] = data[0];
	can->Can_Tx.data[1] = data[1];
	can->Can_Tx.data[2] = data[2];
	can->Can_Tx.data[3] = data[3];
	can->Can_Tx.data[4] = data[4];
	can->Can_Tx.data[5] = data[5];
	can->Can_Tx.data[6] = data[6];
	can->Can_Tx.data[7] = data[7];
	
	MCP2515_Send_Message(can,&can->Can_Tx);
}


//	can伪中断回调函数
void mcp2515_can3_rx_data(MCP2515_HandleTypeDef *can)
{
	mcp_error = MCP2515_Read_Message(can,&can->Can_Rx);
	if (mcp_error == ERROR_OK) 
	{
		//	接收数据回调函数
		//	调用示例 	motor_control中的can帧解析函数
		//	can3_rx_callback(can->Can_Rx.data);
		
	}
}

//	can伪中断回调函数
void mcp2515_can4_rx_data(MCP2515_HandleTypeDef *can)
{
	mcp_error = MCP2515_Read_Message(can,&can->Can_Rx);
	if (mcp_error == ERROR_OK) 
	{
		//	接收数据回调函数
		//	调用示例 	motor_control中的can帧解析函数
		//	can3_rx_callback(can->Can_Rx.data);
		
	}
}

//	can伪中断回调函数
void mcp2515_can5_rx_data(MCP2515_HandleTypeDef *can)
{
	mcp_error = MCP2515_Read_Message(can,&can->Can_Rx);
	if (mcp_error == ERROR_OK) 
	{
		//	接收数据回调函数
		//	调用示例 	motor_control中的can帧解析函数
		//	can3_rx_callback(can->Can_Rx.data);
		
	}
}
