# femTomas - Tomasulo Algorithm Simulator

## Team Information
- **Student 1:** Mohamed Ouail Slama (ID: [Your ID])
- **Student 2:** Yahia Kilany (ID: [Your ID])
- **Course:** CSCE 3301 – Computer Architecture
- **Instructor:** Cherif Salama
- **Submission Date:** December 09, 2025

## Project Overview
femTomas is a simulator that models a 16-bit RISC processor using **Tomasulo's algorithm with speculation**. It supports a simplified RISC ISA, a single-issue pipeline, and configurable hardware parameters. The simulator tracks instruction progress through reservation stations, a reorder buffer (ROB), and outputs detailed performance metrics.

## Release Notes

### What Works
- ✅ Full implementation of Tomasulo's algorithm with speculation
- ✅ Support for all required instructions: LOAD, STORE, BEQ, CALL, RET, ADD, SUB, NAND, MUL
- ✅ Four-stage pipeline simulation (Issue → Execute → Write → Commit)
- ✅ Always-not-taken branch prediction with correct flush/recovery
- ✅ Memory disambiguation for load-store ordering
- ✅ Customizable hardware configuration via input file (Bonus Feature)
- ✅ Input parsing from structured text files
- ✅ Cycle-accurate timing tracking for all instructions
- ✅ Performance metrics: IPC, total cycles, branch misprediction rate
- ✅ Console-based interface with manual and file input modes
- ✅ Register file (R0-R7) with R0 hardwired to 0
- ✅ Word-addressable memory (128KB capacity)

### What Partially Works / Limitations
- ⚠️ The simulator assumes all instructions are pre-fetched and decoded (front-end takes 0 cycles)
- ⚠️ No cache or memory hierarchy simulation (unless implemented as bonus)
- ⚠️ No multiple-issue support (single-issue only)
- ⚠️ No GUI interface (console-based only)
- ⚠️ Limited to the provided instruction set; no floating-point or I/O instructions

### What Doesn't Work
- ❌ Dynamic branch prediction algorithms (1-bit/2-bit) not implemented
- ❌ Cache hierarchy simulation not implemented
- ❌ Assembly parser does not support labels (must use direct addresses)
- ❌ No interrupt or exception handling

### Known Issues / Assumptions
1. Memory is initialized only at specified addresses; uninitialized addresses return 0
2. CALL instruction stores return address in R1 (as per ISA specification)
3. BEQ offset is PC-relative (PC + 1 + offset)
4. MUL stores only lower 16 bits of 32-bit result
5. Pipeline width is fixed at 1 (single-issue)
6. One-to-one mapping between reservation stations and functional units
7. No data forwarding beyond the CDB broadcast mechanism

## Hardware Configuration Defaults
- **ROB Entries:** 8
- **Reservation Stations:**
  - LOAD: 2
  - STORE: 1
  - BEQ: 2
  - CALL/RET: 1
  - ADD/SUB: 4
  - NAND: 2
  - MUL: 1
- **Execution Latencies (cycles):**
  - LOAD: 6
  - STORE: 6
  - BEQ: 1
  - CALL: 1
  - RET: 1
  - ADD: 2
  - SUB: 2
  - NAND: 1
  - MUL: 12

## Bonus Features Implemented
1. **Hardware Configuration Customization:** Users can specify custom hardware parameters via CONFIG section in input files, including:
   - Number of reservation stations per functional unit
   - ROB size
   - Execution latencies for all instruction types

## Input Methods
**Manual Input:** Enter instructions and memory values interactively  
**File Input:** Load from a structured text file (see format below)

## Input File Format
[Optional CONFIG section]
<starting_address>
<instruction_1>
<instruction_2>
...
END
<address_1> <value_1>
<address_2> <value_2>
...
-1 -1
For detailed information about the output format and complete documentation, see [input_format.md](input_format.md).