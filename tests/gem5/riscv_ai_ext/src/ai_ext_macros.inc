.option norvc

.set CUSTOM_0_OPCODE, 0x0b

.macro AI_MAC rd, rs1, rs2
    .4byte ((0x00 << 25) | (\rs2 << 20) | (\rs1 << 15) | (0x0 << 12) | (\rd << 7) | CUSTOM_0_OPCODE)
.endm

.macro AI_DOT4_ACC rd, rs1, rs2
    .4byte ((0x01 << 25) | (\rs2 << 20) | (\rs1 << 15) | (0x0 << 12) | (\rd << 7) | CUSTOM_0_OPCODE)
.endm

.macro AI_RELU rd, rs1
    .4byte ((0x02 << 25) | (0x00 << 20) | (\rs1 << 15) | (0x0 << 12) | (\rd << 7) | CUSTOM_0_OPCODE)
.endm

.macro AI_CLAMP rd, rs1, rs2
    .4byte ((\rs2 << 20) | (\rs1 << 15) | (0x1 << 12) | (\rd << 7) | CUSTOM_0_OPCODE)
.endm

.macro AI_LP_SETUP rs1, raw_imm
    .4byte (((\raw_imm & 0xfff) << 20) | (\rs1 << 15) | (0x2 << 12) | (0x0 << 7) | CUSTOM_0_OPCODE)
.endm

.macro AI_P_LW rd, rs1, imm
    .4byte (((\imm & 0xfff) << 20) | (\rs1 << 15) | (0x3 << 12) | (\rd << 7) | CUSTOM_0_OPCODE)
.endm
