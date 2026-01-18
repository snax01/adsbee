
#include "MCP251XFD.h"

// -----------------------------------------------------------------------------
// FIFO/TEF RAM bookkeeping (driver fills these in during ConfigureFIFOList)
// -----------------------------------------------------------------------------
static MCP251XFD_RAMInfos Ext1_TEF_RAMInfos;
static MCP251XFD_RAMInfos Ext1_FIFO1_RAMInfos;
static MCP251XFD_RAMInfos Ext1_FIFO2_RAMInfos;

// -----------------------------------------------------------------------------
// FIFO list: TEF (8) + RX FIFO1 (10) + TX FIFO2 (16)
// IMPORTANT: Order matters for RAM packing: TEF then TXQ then FIFO1..FIFO31. :contentReference[oaicite:1]{index=1}
// -----------------------------------------------------------------------------
#define EXT1_FIFO_COUNT 3
static MCP251XFD_FIFO Ext1_FIFOlist[EXT1_FIFO_COUNT] =
{
    // --- TEF: Transmit Event FIFO (logs completed transmits) ---
    {
        .Name = MCP251XFD_TEF,
        .Size = MCP251XFD_FIFO_8_MESSAGE_DEEP,
        .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_OBJ, // timestamp entries
        .InterruptFlags = (eMCP251XFD_FIFOIntFlags)(MCP251XFD_FIFO_OVERFLOW_INT
                        | MCP251XFD_FIFO_EVENT_FIFO_NOT_EMPTY_INT),
        .RAMInfos = &Ext1_TEF_RAMInfos,
    },

    // --- RX FIFO1: 10 deep, 64-byte payload ---
    {
        .Name = MCP251XFD_FIFO1,
        .Size = MCP251XFD_FIFO_10_MESSAGE_DEEP,
        .Payload = MCP251XFD_PAYLOAD_64BYTE,
        .Direction = MCP251XFD_RECEIVE_FIFO,
        .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_RX, // timestamp RX frames
        .InterruptFlags = (eMCP251XFD_FIFOIntFlags)(MCP251XFD_FIFO_OVERFLOW_INT
                        | MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT),
        .RAMInfos = &Ext1_FIFO1_RAMInfos,
    },

    // --- TX FIFO2: 16 deep, 64-byte payload ---
    {
        .Name = MCP251XFD_FIFO2,
        .Size = MCP251XFD_FIFO_16_MESSAGE_DEEP,
        .Payload = MCP251XFD_PAYLOAD_64BYTE,
        .Direction = MCP251XFD_TRANSMIT_FIFO,
        .Attempts = MCP251XFD_THREE_ATTEMPTS,
        .Priority = MCP251XFD_MESSAGE_TX_PRIORITY16,
        .ControlFlags = MCP251XFD_FIFO_NO_RTR_RESPONSE,
        .InterruptFlags = (eMCP251XFD_FIFOIntFlags)(MCP251XFD_FIFO_TX_ATTEMPTS_EXHAUSTED_INT
                        | MCP251XFD_FIFO_TRANSMIT_FIFO_NOT_FULL_INT),
        .RAMInfos = &Ext1_FIFO2_RAMInfos,
    },
};