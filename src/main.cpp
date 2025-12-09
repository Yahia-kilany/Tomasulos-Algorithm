#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

// Instruction types
enum InstructionType { LOAD, STORE, BEQ, CALL, RET, ADD, SUB, NAND, MUL };

// Static instruction structure (the instruction definition)
struct Instruction {
  InstructionType type;
  int rA, rB, rC;
  int offset;
  int label;
  int pc;
  string original;

  Instruction() : rA(0), rB(0), rC(0), offset(0), label(0), pc(0) {}
};

// Issued instruction instance (tracks timing for each issued instance)
struct IssuedInstruction {
  int pc;           // Which instruction (PC)
  int issue_number; // Which issue of this PC (for loops)
  int issue_cycle;
  int exec_start_cycle;
  int exec_end_cycle;
  int write_cycle;
  int commit_cycle;
  bool flushed;

  IssuedInstruction(int pc_val, int issue_num)
      : pc(pc_val), issue_number(issue_num), issue_cycle(-1),
        exec_start_cycle(-1), exec_end_cycle(-1), write_cycle(-1),
        commit_cycle(-1), flushed(false) {}
};

// Reservation Station
struct ReservationStation {
  bool busy;
  InstructionType op;
  int Vj, Vk;
  int Qj, Qk;
  int dest;
  int address;
  int rob_entry;
  int cycles_remaining;
  bool executing;
  bool exec_done; // Flag to delay write by one cycle after execution ends

  ReservationStation()
      : busy(false), Vj(0), Vk(0), Qj(-1), Qk(-1), dest(-1), address(0),
        rob_entry(-1), cycles_remaining(0), executing(false), exec_done(false) {
  }
};

// ROB Entry
struct ROBEntry {
  bool busy;
  InstructionType type;
  int dest;
  int value;
  bool ready;
  int instruction_pc;
  bool is_speculative;
  int branch_target;     // For branches, the computed target PC
  int issued_inst_index; // Index into issued_instructions vector

  ROBEntry()
      : busy(false), dest(-1), value(0), ready(false), instruction_pc(-1),
        is_speculative(false), branch_target(-1), issued_inst_index(-1) {}
};



// Hardware configuration structure
struct HardwareConfig {
  // Reservation stations
  int load_rs = 2;
  int store_rs = 1;
  int beq_rs = 2;
  int call_ret_rs = 1;
  int addsub_rs = 4;
  int nand_rs = 2;
  int mul_rs = 1;
  
  // ROB
  int rob_entries = 8;
  
  // Execution cycles
  int load_cycles = 6;
  int store_cycles = 6;
  int beq_cycles = 1;
  int call_cycles = 1;
  int ret_cycles = 1;
  int add_cycles = 2;
  int sub_cycles = 2;
  int nand_cycles = 1;
  int mul_cycles = 12;
};

// Parsed input data
struct ParsedInput {
  HardwareConfig config;
  int start_address;
  map<int, Instruction> instructions;
  map<int, int> memory;
  bool success;
  string error_message;
  
  ParsedInput() : start_address(0), success(false) {}
};

// ===========================    SIMULATION LOGIC    ================================== //

class TomasuroSimulator {
private:
  // Hardware components
  vector<ReservationStation> load_rs;
  vector<ReservationStation> store_rs;
  vector<ReservationStation> beq_rs;
  vector<ReservationStation> call_ret_rs;
  vector<ReservationStation> addsub_rs;
  vector<ReservationStation> nand_rs;
  vector<ReservationStation> mul_rs;

  vector<ROBEntry> rob;
  vector<int> registers;
  vector<int> register_status;
  map<int, int> memory;

  map<int, Instruction> instructions;            // PC -> Instruction definition 
  vector<IssuedInstruction> issued_instructions; // All issued instances
  map<int, int> instruction_issue_count; // Track how many times each PC issued
  int cycle;
  int fetch_pc;
  int rob_head;
  int rob_tail;
  int total_instructions_issued;
  int total_instructions_committed;
  int branches_encountered;
  int branch_mispredictions;
  bool speculating;
  int speculative_branch_rob;
  int max_instructions; // Safety limit

  // Execution latencies
  map<InstructionType, int> execution_cycles;

public:
  TomasuroSimulator()
      : cycle(0), fetch_pc(0), rob_head(0), rob_tail(0),
        total_instructions_issued(0), total_instructions_committed(0),
        branches_encountered(0), branch_mispredictions(0), speculating(false),
        speculative_branch_rob(-1), max_instructions(1000) {
    // Initialize reservation stations
    load_rs.resize(2);
    store_rs.resize(1);
    beq_rs.resize(2);
    call_ret_rs.resize(1);
    addsub_rs.resize(4);
    nand_rs.resize(2);
    mul_rs.resize(1);

    // Initialize ROB
    rob.resize(8);

    // Initialize registers (R0 always 0)
    registers.resize(8, 0);
    register_status.resize(8, -1);

    // Set execution cycles
    execution_cycles[LOAD] = 6;
    execution_cycles[STORE] = 6;
    execution_cycles[BEQ] = 1;
    execution_cycles[CALL] = 1;
    execution_cycles[RET] = 1;
    execution_cycles[ADD] = 2;
    execution_cycles[SUB] = 2;
    execution_cycles[NAND] = 1;
    execution_cycles[MUL] = 12;
  }

  TomasuroSimulator(const HardwareConfig& config)
      : cycle(0), fetch_pc(0), rob_head(0), rob_tail(0),
        total_instructions_issued(0), total_instructions_committed(0),
        branches_encountered(0), branch_mispredictions(0), speculating(false),
        speculative_branch_rob(-1), max_instructions(1000) {
    
    // Initialize reservation stations with custom sizes
    load_rs.resize(config.load_rs);
    store_rs.resize(config.store_rs);
    beq_rs.resize(config.beq_rs);
    call_ret_rs.resize(config.call_ret_rs);
    addsub_rs.resize(config.addsub_rs);
    nand_rs.resize(config.nand_rs);
    mul_rs.resize(config.mul_rs);

    // Initialize ROB with custom size
    rob.resize(config.rob_entries);

    // Initialize registers (R0 always 0)
    registers.resize(8, 0);
    register_status.resize(8, -1);

    // Set execution cycles from config
    execution_cycles[LOAD] = config.load_cycles;
    execution_cycles[STORE] = config.store_cycles;
    execution_cycles[BEQ] = config.beq_cycles;
    execution_cycles[CALL] = config.call_cycles;
    execution_cycles[RET] = config.ret_cycles;
    execution_cycles[ADD] = config.add_cycles;
    execution_cycles[SUB] = config.sub_cycles;
    execution_cycles[NAND] = config.nand_cycles;
    execution_cycles[MUL] = config.mul_cycles;
  }

  void addInstruction(int pc, const Instruction &inst) {
    instructions[pc] = inst;
  }

  void setMemory(int address, int value) { memory[address] = value; }

  void setStartPC(int pc) { fetch_pc = pc; }

  void run() {
    while (total_instructions_committed < max_instructions) {
      cycle++;

      // Check if we're done (no more work in pipeline)
      bool pipeline_empty = true;
      for (int i = 0; i < 8; i++) {
        if (rob[i].busy) {
          pipeline_empty = false;
          break;
        }
      }

      if (pipeline_empty && instructions.find(fetch_pc) == instructions.end()) {
        break; // All done
      }

      commit();
      write();
      execute();
      issue();

      if (cycle > 10000) {
        cout << "\nSimulation stopped: exceeded 10000 cycles (possible "
                "infinite loop)\n";
        break;
      }
    }
  }

  void issue() {
    // Check if we have an instruction at current PC
    if (instructions.find(fetch_pc) == instructions.end())
      return;

    // Check if ROB is full
    if (rob[rob_tail].busy)
      return;

    Instruction inst = instructions[fetch_pc];
    vector<ReservationStation> *rs_bank = nullptr;

    // Select reservation station bank
    switch (inst.type) {
    case LOAD:
      rs_bank = &load_rs;
      break;
    case STORE:
      rs_bank = &store_rs;
      break;
    case BEQ:
      rs_bank = &beq_rs;
      break;
    case CALL:
    case RET:
      rs_bank = &call_ret_rs;
      break;
    case ADD:
      rs_bank = &addsub_rs;
      break; 
    case SUB:
      rs_bank = &addsub_rs;
      break;
    case NAND:
      rs_bank = &nand_rs;
      break;
    case MUL:
      rs_bank = &mul_rs;
      break;
    }

    // Find free RS entry
    int free_rs = -1;
    for (int i = 0; i < rs_bank->size(); i++) {
      if (!(*rs_bank)[i].busy) {
        free_rs = i;
        break;
      }
    }
    if (free_rs == -1)
      return;

    // Issue instruction to RS
    ReservationStation &rs = (*rs_bank)[free_rs];
    rs.busy = true;
    rs.op = inst.type;
    rs.rob_entry = rob_tail;
    rs.executing = false;
    rs.cycles_remaining = execution_cycles[inst.type];

    // Helper to setup operands with dependency tracking
    auto setupOperand = [&](int reg, int &V, int &Q) {
      if (register_status[reg] != -1) {
        int prod = register_status[reg];
        // If producing ROB is already ready, take the value immediately
        if (rob[prod].ready) {
          V = rob[prod].value;
          Q = -1;
        } else {
          Q = prod;
        }
      } else {
        V = registers[reg];
        Q = -1;
      }
    };

    // Setup operands based on instruction type
    if (inst.type == LOAD) {
      setupOperand(inst.rB, rs.Vj, rs.Qj);
      rs.address = inst.offset;
    } else if (inst.type == STORE) {
      setupOperand(inst.rA, rs.Vj, rs.Qj);
      setupOperand(inst.rB, rs.Vk, rs.Qk);
      rs.address = inst.offset;
    } else if (inst.type == BEQ) {
      setupOperand(inst.rA, rs.Vj, rs.Qj);
      setupOperand(inst.rB, rs.Vk, rs.Qk);
      rs.address = inst.offset;
      branches_encountered++;
    } else if (inst.type == CALL) {
      rs.address = inst.label;
    } else if (inst.type == RET) {
      setupOperand(1, rs.Vj, rs.Qj);
    } else {
      // ALU operations (ADD, SUB, NAND, MUL)
      setupOperand(inst.rB, rs.Vj, rs.Qj);
      setupOperand(inst.rC, rs.Vk, rs.Qk);
    }

    // Update ROB entry
    rob[rob_tail].busy = true;
    rob[rob_tail].type = inst.type;
    rob[rob_tail].instruction_pc = fetch_pc;
    rob[rob_tail].ready = false;
    rob[rob_tail].is_speculative = false; // We predict not taken

    // Register destination tracking
    if (inst.type == LOAD || inst.type == ADD || inst.type == SUB ||
        inst.type == NAND || inst.type == MUL) {
      rob[rob_tail].dest = inst.rA;
      if (inst.rA != 0)
        register_status[inst.rA] = rob_tail;
    } else if (inst.type == CALL) {
      rob[rob_tail].dest = 1;
      register_status[1] = rob_tail;
    } else {
      rob[rob_tail].dest = -1;
    }

    // Create issued instruction instance for timing
    int issue_num = instruction_issue_count[fetch_pc];
    IssuedInstruction issued_inst(fetch_pc, issue_num);
    issued_inst.issue_cycle = cycle;
    issued_instructions.push_back(issued_inst);
    rob[rob_tail].issued_inst_index = issued_instructions.size() - 1;

    instruction_issue_count[fetch_pc]++;
    total_instructions_issued++;
    rob_tail = (rob_tail + 1) % rob.size();

    // Predict not taken: fetch next sequential PC
    fetch_pc++;
  }

  void execute() {
    executeRS(load_rs);
    executeRS(store_rs);
    executeRS(beq_rs);
    executeRS(call_ret_rs);
    executeRS(addsub_rs);
    executeRS(nand_rs);
    executeRS(mul_rs);
  }

  void executeRS(vector<ReservationStation> &rs_bank) {
    for (auto &rs : rs_bank) {
      if (!rs.busy)
        continue;

      int issued_idx = rob[rs.rob_entry].issued_inst_index;
      if (issued_idx < 0 || issued_instructions[issued_idx].flushed) {
        continue;
      }

      if (!rs.executing) {
        if (rs.Qj == -1 && rs.Qk == -1) {
          // Loads must also wait for older stores with same/unknown address
          if (rs.op == LOAD) {
            int eff_addr = rs.Vj + rs.address;
            if (hasOlderStoreConflict(rs.rob_entry, eff_addr)) {
              continue;
            }
          }

          rs.executing = true;
          if (issued_instructions[issued_idx].exec_start_cycle == -1) {
            issued_instructions[issued_idx].exec_start_cycle = cycle;
          }
        }
      }

      if (rs.executing) {
        rs.cycles_remaining--;
        if (rs.cycles_remaining == 0) {
          int result = 0;
          bool branch_taken = false;

          switch (rs.op) {
          case LOAD:
            // Memory read happens once load-store ordering check passes
            result = memory[rs.Vj + rs.address];
            break;
          case STORE:
            result = rs.Vj;
            break;
          case BEQ:
            branch_taken = (rs.Vj == rs.Vk);
            result = branch_taken;
            // Calculate and store branch target
            rob[rs.rob_entry].branch_target =
                rob[rs.rob_entry].instruction_pc + 1 + rs.address;
            break;
          case CALL:
            result = rob[rs.rob_entry].instruction_pc + 1;
            break;
          case RET:
            result = rs.Vj;
            break;
          case ADD:
            result = (rs.Vj + rs.Vk) & 0xFFFF;
            break;
          case SUB:
            result = (rs.Vj - rs.Vk) & 0xFFFF;
            break;
          case NAND:
            result = ~(rs.Vj & rs.Vk) & 0xFFFF;
            break;
          case MUL:
            result = (rs.Vj * rs.Vk) & 0xFFFF;
            break;
          }

          rob[rs.rob_entry].value = result;

          if (rs.op == STORE) {
            rob[rs.rob_entry].dest = rs.Vk + rs.address;
          }

          int issued_idx = rob[rs.rob_entry].issued_inst_index;
          if (issued_instructions[issued_idx].exec_end_cycle == -1) {
            issued_instructions[issued_idx].exec_end_cycle = cycle;
          }

          // Mark execution done, delay write to next cycle
          rs.exec_done = true;
        }
      }
    }
  }

  void write() {
    writeRS(load_rs);
    writeRS(store_rs);
    writeRS(beq_rs);
    writeRS(call_ret_rs);
    writeRS(addsub_rs);
    writeRS(nand_rs);
    writeRS(mul_rs);
  }

  void writeRS(vector<ReservationStation> &rs_bank) {
    for (auto &rs : rs_bank) {
      if (!rs.busy || !rs.executing || rs.cycles_remaining != 0)
        continue;
      // Ready flag is asserted at write stage (end of CDB/broadcast)

      int issued_idx = rob[rs.rob_entry].issued_inst_index;
      if (issued_idx < 0 || issued_instructions[issued_idx].flushed) {
        rs.busy = false;
        rs.executing = false;
        continue;
      }

      int rob_num = rs.rob_entry;
      int value = rob[rob_num].value;

      // Mark result ready now
      rob[rob_num].ready = true;

      if (issued_instructions[issued_idx].write_cycle == -1) {
        issued_instructions[issued_idx].write_cycle = cycle;
      }

      // Stores do not broadcast on the CDB; others do
      if (rs.op != STORE) {
        broadcastResult(rob_num, value);
      }

      rs.busy = false;
      rs.executing = false;
    }
  }

  bool hasOlderStoreConflict(int rob_index, int address) {
    int idx = rob_head;
    while (idx != rob_index) {
      if (rob[idx].busy && rob[idx].type == STORE) {
        // Wait if older store address/value not ready or same address
        if (!rob[idx].ready)
          return true;
        if (rob[idx].dest == address)
          return true;
      }
      idx = (idx + 1) % rob.size();
    }
    return false;
  }

  void broadcastResult(int rob_num, int value) {
    auto update = [&](vector<ReservationStation> &rs_bank) {
      for (auto &rs : rs_bank) {
        if (rs.Qj == rob_num) {
          rs.Vj = value;
          rs.Qj = -1;
        }
        if (rs.Qk == rob_num) {
          rs.Vk = value;
          rs.Qk = -1;
        }
      }
    };

    update(load_rs);
    update(store_rs);
    update(beq_rs);
    update(call_ret_rs);
    update(addsub_rs);
    update(nand_rs);
    update(mul_rs);
  }

  void flush(int from_rob_index, int correct_pc) {
    cout << "  [Cycle " << cycle
         << "] Misprediction! Flushing, redirecting to PC " << correct_pc
         << endl;

    // Mark instructions as flushed in ROB
    int current = (from_rob_index + 1) % 8;
    while (current != rob_tail) {
      if (rob[current].busy) {
        int issued_idx = rob[current].issued_inst_index;
        if (issued_idx >= 0) {
          issued_instructions[issued_idx].flushed = true;
        }

        // Clear register status
        if (rob[current].dest >= 0 && rob[current].dest < 8) {
          if (register_status[rob[current].dest] == current) {
            register_status[rob[current].dest] = -1;
          }
        }

        rob[current].busy = false;
        rob[current].ready = false;
      }
      current = (current + 1) % 8;
    }

    // Clear reservation stations
    auto clearRS = [&](vector<ReservationStation> &rs_bank) {
      for (auto &rs : rs_bank) {
        if (rs.busy) {
          int issued_idx = rob[rs.rob_entry].issued_inst_index;
          if (issued_idx >= 0 && issued_instructions[issued_idx].flushed) {
            rs.busy = false;
            rs.executing = false;
          }
        }
      }
    };

    clearRS(load_rs);
    clearRS(store_rs);
    clearRS(beq_rs);
    clearRS(call_ret_rs);
    clearRS(addsub_rs);
    clearRS(nand_rs);
    clearRS(mul_rs);

    // Reset ROB tail
    rob_tail = (from_rob_index + 1) % 8;

    // Update fetch PC to correct target
    fetch_pc = correct_pc;

    // No longer speculating
    speculating = false;
    speculative_branch_rob = -1;
  }

  void commit() {
    if (!rob[rob_head].busy)
      return;
    if (!rob[rob_head].ready)
      return;

    ROBEntry &entry = rob[rob_head];
    int inst_pc = entry.instruction_pc;
    Instruction &inst = instructions[inst_pc];
    int issued_idx = entry.issued_inst_index;

    // Check if instruction was flushed
    if (issued_idx < 0 || issued_instructions[issued_idx].flushed) {
      entry.busy = false;
      rob_head = (rob_head + 1) % 8;
      return;
    }

    // Handle branch
    if (entry.type == BEQ) {
      bool actually_taken = (entry.value != 1);
      bool predicted_not_taken = true;

      if (actually_taken != predicted_not_taken) {
        // Misprediction! Branch was taken but we predicted not taken
        branch_mispredictions++;
        int correct_pc = entry.branch_target; // Use pre-computed target
        flush(rob_head, correct_pc);
      } else {
        // Correct prediction
        if (speculative_branch_rob == rob_head) {
          speculating = false;
          speculative_branch_rob = -1;
        }
      }
    }

    // Commit the instruction (write results to memory/registers)
    if (entry.type == STORE) {
      memory[entry.dest] = entry.value;
    } else if (entry.dest > 0 && entry.dest < 8) {
      // R0 is always zero, never write to it
      registers[entry.dest] = entry.value;
      if (register_status[entry.dest] == rob_head) {
        register_status[entry.dest] = -1;
      }
    }

    // Handle control flow instructions (CALL and RET flush pipeline)
    if (entry.type == CALL) {
      // CALL is a control flow change - flush subsequent instructions
      // and jump to the called function
      flush(rob_head, inst.label);
    } else if (entry.type == RET) {
      // RET returns control to caller - flush subsequent instructions
      flush(rob_head, entry.value);
    }

    if (issued_instructions[issued_idx].commit_cycle == -1) {
      issued_instructions[issued_idx].commit_cycle = cycle;
    }
    total_instructions_committed++;

    entry.busy = false;
    rob_head = (rob_head + 1) % 8;
  }

  void printResults() {
    cout << "\n=== SIMULATION RESULTS ===\n\n";
    cout << "Instruction Timing Table (All Issued Instances):\n";
    cout << setw(4) << "PC" << setw(30) << "Instruction" << setw(6) << "#"
         << setw(8) << "Issue" << setw(8) << "ExecS" << setw(8) << "ExecE"
         << setw(8) << "Write" << setw(8) << "Commit" << setw(10) << "Status"
         << endl;
    cout << string(96, '-') << endl;

    for (auto &issued : issued_instructions) {
      int pc = issued.pc;
      Instruction &inst = instructions[pc];

      cout << setw(4) << pc << setw(30) << inst.original << setw(6)
           << issued.issue_number << setw(8) << issued.issue_cycle << setw(8)
           << issued.exec_start_cycle << setw(8) << issued.exec_end_cycle
           << setw(8) << issued.write_cycle << setw(8) << issued.commit_cycle
           << setw(10) << (issued.flushed ? "FLUSHED" : "OK") << endl;
    }

    cout << "\n=== PERFORMANCE METRICS ===\n";
    cout << "Total execution time: " << cycle << " cycles\n";
    cout << "Instructions issued: " << total_instructions_issued << endl;
    cout << "Instructions committed: " << total_instructions_committed << endl;
    cout << "IPC: " << fixed << setprecision(3)
         << (double)total_instructions_committed / cycle << endl;
    cout << "Branches encountered: " << branches_encountered << endl;
    cout << "Branch mispredictions: " << branch_mispredictions << endl;
    if (branches_encountered > 0) {
      cout << "Branch misprediction rate: " << fixed << setprecision(2)
           << (100.0 * branch_mispredictions / branches_encountered) << "%\n";
    }

    cout << "\n=== FINAL REGISTER STATE ===\n";
    for (int i = 0; i < 8; i++) {
      cout << "R" << i << ": " << registers[i] << "  ";
      if ((i + 1) % 4 == 0)
        cout << endl;
    }

    cout << "\n=== FINAL MEMORY STATE ===\n";
    for (auto &pair : memory) {
      cout << "Mem[" << pair.first << "]: " << pair.second << endl;
    }
  }
};

Instruction parseInstruction(const string &line, int pc) {
  Instruction inst;
  inst.pc = pc;
  inst.original = line;

  stringstream ss(line);
  string opcode;
  ss >> opcode;

  transform(opcode.begin(), opcode.end(), opcode.begin(), ::toupper);

  if (opcode == "LOAD") {
    inst.type = LOAD;
    string rA, offset_rB;
    ss >> rA >> offset_rB;
    inst.rA = rA[1] - '0';

    size_t paren = offset_rB.find('(');
    inst.offset = stoi(offset_rB.substr(0, paren));
    inst.rB = offset_rB[paren + 2] - '0';
  } else if (opcode == "STORE") {
    inst.type = STORE;
    string rA, offset_rB;
    ss >> rA >> offset_rB;
    inst.rA = rA[1] - '0';

    size_t paren = offset_rB.find('(');
    inst.offset = stoi(offset_rB.substr(0, paren));
    inst.rB = offset_rB[paren + 2] - '0';
  } else if (opcode == "BEQ") {
    inst.type = BEQ;
    string rA, rB, offset;
    ss >> rA >> rB >> offset;
    inst.rA = rA[1] - '0';
    inst.rB = rB[1] - '0';
    inst.offset = stoi(offset);
  } else if (opcode == "CALL") {
    inst.type = CALL;
    ss >> inst.label;
  } else if (opcode == "RET") {
    inst.type = RET;
  } else if (opcode == "ADD") {
    inst.type = ADD;
    string rA, rB, rC;
    ss >> rA >> rB >> rC;
    inst.rA = rA[1] - '0';
    inst.rB = rB[1] - '0';
    inst.rC = rC[1] - '0';
  } else if (opcode == "SUB") {
    inst.type = SUB;
    string rA, rB, rC;
    ss >> rA >> rB >> rC;
    inst.rA = rA[1] - '0';
    inst.rB = rB[1] - '0';
    inst.rC = rC[1] - '0';
  } else if (opcode == "NAND") {
    inst.type = NAND;
    string rA, rB, rC;
    ss >> rA >> rB >> rC;
    inst.rA = rA[1] - '0';
    inst.rB = rB[1] - '0';
    inst.rC = rC[1] - '0';
  } else if (opcode == "MUL") {
    inst.type = MUL;
    string rA, rB, rC;
    ss >> rA >> rB >> rC;
    inst.rA = rA[1] - '0';
    inst.rB = rB[1] - '0';
    inst.rC = rC[1] - '0';
  }

  return inst;
}
// ===========================    PARSING LOGIC    ================================== //

ParsedInput parse(const string& filename) {
  ParsedInput result;
  ifstream infile(filename);
  
  if (!infile.is_open()) {
    result.success = false;
    result.error_message = "Could not open file '" + filename + "'";
    return result;
  }
  
  string line;
  enum ParseState { 
    LOOKING_FOR_CONFIG_OR_ADDRESS, 
    IN_CONFIG, 
    READ_ADDRESS, 
    READING_INSTRUCTIONS, 
    READING_MEMORY 
  };
  
  ParseState state = LOOKING_FOR_CONFIG_OR_ADDRESS;
  int pc = 0;
  
  while (getline(infile, line)) {
    // Trim whitespace
    line.erase(0, line.find_first_not_of(" \t\r\n"));
    line.erase(line.find_last_not_of(" \t\r\n") + 1);
    
    // Skip empty lines
    if (line.empty()) continue;
    
    // State machine for parsing
    if (state == LOOKING_FOR_CONFIG_OR_ADDRESS) {
      if (line == "CONFIG") {
        state = IN_CONFIG;
      } else {
        // It's the starting address
        try {
          result.start_address = stoi(line);
          pc = result.start_address;
          state = READING_INSTRUCTIONS;
        } catch (...) {
          result.success = false;
          result.error_message = "Invalid starting address: " + line;
          return result;
        }
      }
    }
    else if (state == IN_CONFIG) {
      if (line == "END_CONFIG") {
        state = LOOKING_FOR_CONFIG_OR_ADDRESS;
      } else {
        // Parse configuration parameter
        stringstream ss(line);
        string param;
        int value;
        ss >> param >> value;
        
        transform(param.begin(), param.end(), param.begin(), ::toupper);
        
        // Reservation stations
        if (param == "LOAD_RS") result.config.load_rs = value;
        else if (param == "STORE_RS") result.config.store_rs = value;
        else if (param == "BEQ_RS") result.config.beq_rs = value;
        else if (param == "CALL_RET_RS") result.config.call_ret_rs = value;
        else if (param == "ADDSUB_RS") result.config.addsub_rs = value;
        else if (param == "NAND_RS") result.config.nand_rs = value;
        else if (param == "MUL_RS") result.config.mul_rs = value;
        
        // ROB
        else if (param == "ROB_ENTRIES") result.config.rob_entries = value;
        
        // Execution cycles
        else if (param == "LOAD_CYCLES") result.config.load_cycles = value;
        else if (param == "STORE_CYCLES") result.config.store_cycles = value;
        else if (param == "BEQ_CYCLES") result.config.beq_cycles = value;
        else if (param == "CALL_CYCLES") result.config.call_cycles = value;
        else if (param == "RET_CYCLES") result.config.ret_cycles = value;
        else if (param == "ADD_CYCLES") result.config.add_cycles = value;
        else if (param == "SUB_CYCLES") result.config.sub_cycles = value;
        else if (param == "NAND_CYCLES") result.config.nand_cycles = value;
        else if (param == "MUL_CYCLES") result.config.mul_cycles = value;
        
        else {
          cout << "Warning: Unknown config parameter '" << param << "'\n";
        }
      }
    }
    else if (state == READING_INSTRUCTIONS) {
      if (line == "END" || line == "end") {
        state = READING_MEMORY;
      } else {
        Instruction inst = parseInstruction(line, pc);
        result.instructions[pc] = inst;
        pc++;
      }
    }
    else if (state == READING_MEMORY) {
      stringstream ss(line);
      int addr, value;
      ss >> addr >> value;
      
      if (addr == -1) {
        break; // Done reading memory
      }
      
      result.memory[addr] = value;
    }
  }
  
  infile.close();
  result.success = true;
  return result;
}



int main() {
  cout << "=== TOMASULO ALGORITHM SIMULATOR ===\n\n";

  // Choose input method
  cout << "Choose input method:\n";
  cout << "1. Manual input\n";
  cout << "2. Read from file\n";
  cout << "Enter choice (1 or 2): ";
  int choice;
  cin >> choice;
  cin.ignore();

  if (choice == 2) {
    // File input
    cout << "Enter filename: ";
    string filename;
    getline(cin, filename);

    ParsedInput parsed = parse(filename);
    
    if (!parsed.success) {
      cerr << "Error: " << parsed.error_message << "\n";
      return 1;
    }
    
    // Create simulator with custom configuration
    TomasuroSimulator sim(parsed.config);
    
    // Set starting PC
    sim.setStartPC(parsed.start_address);
    
    // Add instructions
    for (auto& pair : parsed.instructions) {
      sim.addInstruction(pair.first, pair.second);
    }
    
    // Set memory
    for (auto& pair : parsed.memory) {
      sim.setMemory(pair.first, pair.second);
    }
    
    cout << "Successfully loaded " << parsed.instructions.size()
         << " instructions from file.\n";
    
    cout << "\n=== Hardware Configuration ===\n";
    cout << "ROB Entries: " << parsed.config.rob_entries << "\n";
    cout << "Reservation Stations:\n";
    cout << "  LOAD: " << parsed.config.load_rs << "\n";
    cout << "  STORE: " << parsed.config.store_rs << "\n";
    cout << "  BEQ: " << parsed.config.beq_rs << "\n";
    cout << "  CALL/RET: " << parsed.config.call_ret_rs << "\n";
    cout << "  ADD/SUB: " << parsed.config.addsub_rs << "\n";
    cout << "  NAND: " << parsed.config.nand_rs << "\n";
    cout << "  MUL: " << parsed.config.mul_rs << "\n";
    
    cout << "\n=== RUNNING SIMULATION ===\n";
    sim.run();
    sim.printResults();

  } else {
    // Manual input
    TomasuroSimulator sim;  // ← Create sim HERE for manual mode
    
    int start_addr;
    cout << "\nEnter starting address for program: ";
    cin >> start_addr;
    cin.ignore();
    sim.setStartPC(start_addr);

    // Input instructions
    cout << "\nEnter instructions (one per line, 'END' to finish):\n";
    cout << "Format examples:\n";
    cout << "  LOAD R1, 0(R2)\n";
    cout << "  ADD R3, R1, R2\n";
    cout << "  BEQ R1, R2, -5  (negative offset for backward branch/loop)\n\n";

    int pc = start_addr;
    string line;
    while (true) {
      cout << "Inst[" << pc << "]: ";
      getline(cin, line);
      if (line == "END" || line == "end")
        break;
      if (line.empty())
        continue;

      Instruction inst = parseInstruction(line, pc);
      sim.addInstruction(pc, inst);
      pc++;
    }

    // Input initial memory values
    cout << "\nEnter initial memory values (address value, -1 -1 to finish):\n";
    while (true) {
      int addr, value;
      cout << "Memory: ";
      cin >> addr >> value;
      if (addr == -1)
        break;
      sim.setMemory(addr, value);
    }
    
    cout << "\n=== RUNNING SIMULATION ===\n";
    sim.run();
    sim.printResults();
  }

  return 0;
}