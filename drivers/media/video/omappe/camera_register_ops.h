/*
 * Register Programming Lists
 *
 * This driver uses lists of "register operations" to perform many of its
 * camera module programming duties. These lists ultimately come from .ini
 * files generated from the Micron DevWare utility.
 *
 * Register programming lists are arrays of register operations. Each
 * register operation (REGISTER_OPERATION) consists of an opcode, the
 * register address, and a value to be used for the operation. Reg ops
 * can set registers to specified values, set or clear individual bits
 * within registers, or delay during a register programming sequence.
 *
 * Register operations are simply evaluated in order from the beginning of
 * the list to the end. Lists are terminated by a register operation with a
 * ROP_EOL opcode.
 *
 */
#define REG_MCUADDR         0x338c
#define REG_MCUDATA         0x3390

typedef enum
{
    ROP_EOL = 0,    /* End-of-list marker. */
    ROP_DELAY,      /* Delay for specified milliseconds. */
    ROP_SET,        /* Set the entire register to the specified value. */
    ROP_AND,        /* Clear bits (by AND'ing with the specified value). */
    ROP_OR,         /* Set bits (by OR'ing with the specified value). */
    ROP_WAIT        /* Wait for specified flag to clear. */
} REGISTER_OPCODE;

typedef struct
{
    REGISTER_OPCODE opcode;
    uint32_t reg;
    uint32_t value;
} REGISTER_OPERATION;

/* For transferring new arrays to driver via firmware download mechanism. */
typedef enum
{
    ROA_INIT,
    ROA_PLL_INIT,
    ROA_PROGRAM_SNAPSHOT,
    ROA_PROGRAM_PREVIEW,
    ROA_SYNC_HOLD,
    ROA_SYNC_RELEASE,
    ROA_CTXA_INIT,
    ROA_CTXA_WIDTH,
    ROA_CTXA_HEIGHT,
    ROA_CTXA_RGB,
    ROA_CTXA_YUV,
    ROA_CTXB_INIT,
    ROA_CTXB_WIDTH,
    ROA_CTXB_HEIGHT,
    ROA_CTXB_RGB,
    ROA_CTXB_YUV,
} REGISTER_OPERATION_ARRAY_TYPE;

typedef struct
{
    REGISTER_OPERATION_ARRAY_TYPE type;
    unsigned int count;
    REGISTER_OPERATION rops[0];
} REGISTER_OPERATION_ARRAY;

/* For MCU address and control at 0x33c8. */
#define MCUA_8BITS          (1 << 15)
#define MCUA_LOGADDR        (0x01 << 13)
#define MCUA_DEVID(_devid)  ((_devid & 0x1F) << 8)
#define MCUA_REG(_reg)      (_reg & 0xff)
#define MCU_VAR(_devid, _reg, _val) \
    {ROP_SET, REG_MCUADDR, MCUA_LOGADDR|MCUA_DEVID(_devid)|MCUA_REG(_reg)},\
    {ROP_SET, REG_MCUDATA, _val}
#define MCU_VAR8(_devid, _reg, _val) \
    {ROP_SET, REG_MCUADDR, MCUA_8BITS|MCUA_LOGADDR|MCUA_DEVID(_devid)|MCUA_REG(_reg)},\
    {ROP_SET, REG_MCUDATA, _val}

