# Input File Format (With Hardware Customization)

The input file should have the following format:

## Section 1: Hardware Configuration (Optional)
If this section is omitted, default values are used.
```
CONFIG
ROB_ENTRIES <number>
LOAD_RS <number>
STORE_RS <number>
BEQ_RS <number>
CALL_RET_RS <number>
ADDSUB_RS <number>
NAND_RS <number>
MUL_RS <number>
LOAD_CYCLES <number>
STORE_CYCLES <number>
BEQ_CYCLES <number>
CALL_CYCLES <number>
RET_CYCLES <number>
ADD_CYCLES <number>
SUB_CYCLES <number>
NAND_CYCLES <number>
MUL_CYCLES <number>
END_CONFIG
```

## Section 2: Program Start Address
```
<starting_address>
```

## Section 3: Instructions
```
<instruction_1>
<instruction_2>
...
END
```

## Section 4: Memory Initialization
```
<address_1> <value_1>
<address_2> <value_2>
...
-1 -1
```

---

## Example 1: With Custom Configuration
```
CONFIG
ROB_ENTRIES 16
LOAD_RS 4
STORE_RS 2
BEQ_RS 4
CALL_RET_RS 2
ADDSUB_RS 8
NAND_RS 4
MUL_RS 2
LOAD_CYCLES 4
STORE_CYCLES 4
BEQ_CYCLES 2
CALL_CYCLES 1
RET_CYCLES 1
ADD_CYCLES 1
SUB_CYCLES 1
NAND_CYCLES 1
MUL_CYCLES 8
END_CONFIG
0
LOAD R1, 0(R0)
LOAD R2, 1(R0)
ADD R3, R1, R2
MUL R4, R3, R2
STORE R4, 2(R0)
END
0 10
1 20
-1 -1
```

## Example 2: Using Default Configuration
```
0
LOAD R1, 0(R0)
LOAD R2, 1(R0)
ADD R3, R1, R2
STORE R3, 2(R0)
END
0 10
1 20
-1 -1
```

---

## Configuration Parameters

### Reservation Stations (Default values from project):
- `LOAD_RS`: Number of LOAD reservation stations (default: 2)
- `STORE_RS`: Number of STORE reservation stations (default: 1)
- `BEQ_RS`: Number of BEQ reservation stations (default: 2)
- `CALL_RET_RS`: Number of CALL/RET reservation stations (default: 1)
- `ADDSUB_RS`: Number of ADD/SUB reservation stations (default: 4)
- `NAND_RS`: Number of NAND reservation stations (default: 2)
- `MUL_RS`: Number of MUL reservation stations (default: 1)

### ROB Configuration:
- `ROB_ENTRIES`: Number of ROB entries (default: 8)

### Execution Cycles (Default values from project):
- `LOAD_CYCLES`: Cycles for LOAD execution (default: 6)
- `STORE_CYCLES`: Cycles for STORE execution (default: 6)
- `BEQ_CYCLES`: Cycles for BEQ execution (default: 1)
- `CALL_CYCLES`: Cycles for CALL execution (default: 1)
- `RET_CYCLES`: Cycles for RET execution (default: 1)
- `ADD_CYCLES`: Cycles for ADD execution (default: 2)
- `SUB_CYCLES`: Cycles for SUB execution (default: 2)
- `NAND_CYCLES`: Cycles for NAND execution (default: 1)
- `MUL_CYCLES`: Cycles for MUL execution (default: 12)

### Notes:
- All configuration parameters are optional
- If CONFIG section is missing, all defaults are used
- If only some parameters are specified, others use defaults
- Configuration must come BEFORE the starting address