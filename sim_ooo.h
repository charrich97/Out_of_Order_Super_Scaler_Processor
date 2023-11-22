#ifndef SIM_OO_H_
#define SIM_OO_H_

#include <stdio.h>
#include <stdbool.h>
#include <sstream>
#include <iostream>
#include <inttypes.h>
#include <cassert>
#include <cstdlib>
#include <stdlib.h>
#include <fstream>
#include <cstring>
#include <string>
#include <iomanip>
#include <map>
#include <vector>
#include <algorithm>

using namespace std;

#define UNDEFINED 0xFFFFFFFF //constant used for initialization
#define NUM_GP_REGISTERS 32
#define NUM_FP_REGISTERS 32
#define NUM_OPCODES 28
#define NUM_STAGES 4
#define ASSERT( condition, statement, ... ) \
   if( !(condition) ) { \
      printf( "[ASSERT] In File: %s, Line: %d => " #statement "\n", __FILE__, __LINE__, ##__VA_ARGS__ ); \
      abort(); \
   }

typedef enum {LW, SW, ADD, SUB, XOR, OR, AND, MULT, DIV, ADDI, SUBI, XORI, ORI, ANDI, BEQZ, BNEZ, BLTZ, BGTZ, BLEZ, BGEZ, JUMP, EOP, LWS, SWS, ADDS, SUBS, MULTS, DIVS} opcode_t;

typedef enum {INTEGER_RS, LOAD_B, ADD_RS, MULT_RS, RS_TOTAL} res_station_t;

typedef enum {INTEGER, ADDER, MULTIPLIER, DIVIDER, MEMORY, EX_TOTAL} exe_unit_t;

typedef enum{ISSUE, EXECUTE, WRITE_RESULT, COMMIT} stage_t;

const string opcode_str[] = {"LW", "SW", "ADD", "SUB", "XOR", "OR", "AND", "MULT", "DIV", "ADDI", "SUBI", "XORI", "ORI", "ANDI", "BEQZ", "BNEZ", "BLTZ", "BGTZ", "BLEZ", "BGEZ", "JUMP", "EOP", "LWS", "SWS", "ADDS", "SUBS", "MULTS", "DIVS"};


//-------------------------------------------------------------------------------//
typedef struct dynInstructT* dynamic_instruct_pointer;
typedef struct instructT* instructPT;

struct instructT{
   opcode_t           opcode;
   uint32_t           pc;
   uint32_t           dst;
   uint32_t           src1;
   uint32_t           src2;
   uint32_t           imm;
   bool               dstValid;
   bool               src1Valid;
   bool               src2Valid;
   bool               dstF;
   bool               src1F;
   bool               src2F;
   bool               is_stall;
   bool               is_branch;
   bool               is_taken;
   bool               is_load;
   bool               is_store;

   instructT(){
      nop();
   }

   void copy( instructT input ){
      opcode     = input.opcode; 
      pc         = input.pc; 
      dst        = input.dst; 
      src1       = input.src1; 
      src2       = input.src2; 
      imm        = input.imm; 
      dstValid   = input.dstValid; 
      src1Valid  = input.src1Valid;
      src2Valid  = input.src2Valid;
      is_stall   = input.is_stall; 
      is_branch  = input.is_branch;
      is_taken   = input.is_taken;
      is_store   = input.is_store;
      is_load    = input.is_load;
      dstF       = input.dstF;
      src1F      = input.src1F;    
      src2F      = input.src2F;    
   }

   void print(){
      cout << "Opcode: " << opcode_str[opcode] << ", dst: " << dst << ", src1: " << src1 << ", src2: " << src2 << ", imm: " << imm << ", dstValid: " << dstValid << ", src1Valid: " << src1Valid << ", src2Valid: " << src2Valid << ", dstF: " << dstF << ", src1F: " << src1F << ", src2F: " << src2F << ", is_stall: " << is_stall << ", is_branch: " << is_branch << "PC" << pc << endl;
   }

   void nop(){
      opcode     = EOP;
      dst        = UNDEFINED;
      src1       = UNDEFINED;
      src2       = UNDEFINED;
      imm        = UNDEFINED;
      dstValid   = false;
      src1Valid  = false;
      src2Valid  = false;
      is_stall   = false;
      is_branch  = false;
      is_taken   = false;
      is_store   = false;
      is_load    = false;
      dstF       = false;
      src1F      = false;
      src2F      = false;
   }

   void stall(){
      nop();
      is_stall   = true;
   }
};

struct instStatT{
   unsigned           t_issue;
   unsigned           t_execute;
   unsigned           t_wr;
   unsigned           t_commit;
   stage_t            state;
   unsigned           pc;
   instStatT(){
      t_issue          = UNDEFINED;
      t_execute        = UNDEFINED;
      t_wr             = UNDEFINED;
      t_commit         = UNDEFINED;
      state            = ISSUE;
      pc               = UNDEFINED;
   }
};

struct dynInstructT : public instructT{
   instStatT stat;
   dynInstructT( instructT input ){
      copy(input);
   }
};

//Integer General-Purpose Registers
struct int_fileT{
   int            value;
   int            busy;
   int            tag;
};

//Floating-Point General-Purpose Registers
struct fp_fileT{
   float          value;
   int            busy;
   int            tag;
};

//Reservation Station Data Structure
struct reservation_station_t{
   dynamic_instruct_pointer   dyn_instruction_p            ;
   unsigned                   vj                           ;
   bool                       vj_ready                     ;
   unsigned                   vk                           ;
   bool                       vk_ready                     ;
   unsigned                   qj                           ;
   unsigned                   qk                           ;
   unsigned                   tag_res_stat                 ;
   unsigned                   addr                         ;
   int                        id                           ;
   bool                       pushed_2_exec                ;

   reservation_station_t(){
      vj_ready        = true      ;
      vk_ready        = true      ;
      vj              = UNDEFINED ;
      vk              = UNDEFINED ;
      qj              = UNDEFINED ;
      qk              = UNDEFINED ; 
      tag_res_stat    = UNDEFINED ; 
      addr            = UNDEFINED ;

      pushed_2_exec   = false     ;
   }

};

struct execWrLaneT{
   reservation_station_t*   payloadP;
   int            ttl;
   bool           wr;
   uint32_t       output;
   bool           outputReady;

   execWrLaneT(){
      ttl            = 0;
      wr             = false;
   }

};

struct execWrUnitT{
   execWrLaneT    *lanes;
   int            numLanes;
   int            latency;

   execWrUnitT(){
      lanes          = NULL;
      numLanes       = 0;
      latency        = 0;
   }

   void init(int numLanes, int latency){
      ASSERT( latency > 0, "Impractical latency found (=%d)", latency );
      ASSERT( numLanes > 0, "Unsupported number of lanes (=%d)", numLanes );
      this->numLanes += numLanes;
      this->latency   = latency;
      lanes           = (execWrLaneT*)realloc(lanes, this->numLanes * sizeof(execWrLaneT));
   }
};

struct reorder_buf_t{
   dynamic_instruct_pointer   dyn_instruction_p       ;
   bool                       ready                   ;
   bool                       miss_prediction         ;
   unsigned                   dest                    ;
   unsigned                   value                   ;
   unsigned                   mem_latency             ;


   reorder_buf_t(){
      dyn_instruction_p     = NULL      ;
      ready                 = false     ;
      dest                  = UNDEFINED ;
      value                 = UNDEFINED ;
      mem_latency           = 0         ;
   }

   ~reorder_buf_t(){
   }
};

template <typename T> 
class Fifo {
   private:
      T*        array;
      int       head;
      int       tail;
      int       count;
      int       size;

   public:
      Fifo( int size );
      Fifo();
      ~Fifo();

      uint32_t push( T entry );
      T pop( bool &underflow );
      T* peekHead();
      // n is assumed to be indexed from 0
      // 0 means head, count - 1 means tail
      T* peekNth( int n );
      // Peek a custom physical index
      // NOTE: should be between head and tail
      T* peekIndex( int index );
      int getHeadIndex();
      int getTailIndex();
      int      getCount();
      void popAll();
      // Phase: true if head == tail is to be considered full (head moving against tail)
      //        false if head == tail is empty (head moving towards tail)
      int getCountHeadTail( int head, int tail, bool phase=false );
      void moveHead( int head, bool phase=false, int count=-1 );
      void moveTail( int tail, bool phase=false, int count=-1 );
      bool isFull();
      bool isEmpty();
      uint32_t getSize();
      uint32_t genIndex( uint32_t ith );
      bool isBusy( int index );
};


class sim_ooo{

   int            cycleCount;
   unsigned       PC;

   instructPT     *instMemory;
   int            instMemSize;
   int            instCount;

   int_fileT      int_file[NUM_GP_REGISTERS];
   fp_fileT       fp_file[NUM_FP_REGISTERS];
   execWrUnitT    execFp[EX_TOTAL];

   vector<execWrLaneT> bypassLane;

   unsigned       data_memory_size;
   unsigned char  *data_memory;

   unsigned       mem_latency;
   unsigned       memFlag;
   unsigned       baseAddress;

   vector <reservation_station_t*>  resStation[RS_TOTAL];
   unsigned       *resStSize;

   unsigned       robSize;
   int            issueWidth;
   bool           gSquash;
   bool           memBlock;
   vector <instStatT> log;

   //----------------------------------------------------------------------------//

   Fifo<reorder_buf_t> rob;
   public:

   /* Instantiates the simulator
      Note: registers must be initialized to UNDEFINED value, and data memory to all 0xFF values
   */
   sim_ooo(unsigned mem_size, 		// size of data memory (in byte)
         unsigned rob_size, 		// number of ROB entries
         unsigned num_int_res_stations,	// number of integer reservation stations 
         unsigned num_add_res_stations,	// number of ADD reservation stations
         unsigned num_mul_res_stations, 	// number of MULT/DIV reservation stations
         unsigned num_load_buffers,	// number of LOAD buffers
         unsigned issue_width=1		// issue width
         );	

   //de-allocates the simulator
   ~sim_ooo();

   // adds one or more execution units of a given type to the processor
   // - exec_unit: type of execution unit to be added
   // - latency: latency of the execution unit (in clock cycles)
   // - instances: number of execution units of this type to be added
   void init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances=1);

   //loads the assembly program in file "filename" in instruction memory at the specified address
   void load_program(const char *filename, unsigned base_address=0x0);

   //runs the simulator for "cycles" clock cycles (run the program to completion if cycles=0) 
   void run(unsigned cycles=0);

   //resets the state of the simulator
   /* Note: 
      - registers should be reset to UNDEFINED value 
      - data memory should be reset to all 0xFF values
      - instruction window, reservation stations and rob should be cleaned
      */
   void reset();

   //returns value of the specified integer general purpose register
   int get_int_register(unsigned reg);

   //set the value of the given integer general purpose register to "value"
   void set_int_register(unsigned reg, int value);

   //returns value of the specified floating point general purpose register
   float get_fp_register(unsigned reg);

   //set the value of the given floating point general purpose register to "value"
   void set_fp_register(unsigned reg, float value);

   // returns the index of the ROB entry that will write this integer register (UNDEFINED if the value of the register is not pending
   unsigned get_pending_int_register(unsigned reg);

   // returns the index of the ROB entry that will write this floating point register (UNDEFINED if the value of the register is not pending
   unsigned get_pending_fp_register(unsigned reg);

   //returns the IPC
   float get_IPC();

   //returns the number of instructions fully executed
   unsigned get_instructions_executed();

   //returns the number of clock cycles 
   unsigned get_clock_cycles();

   //prints the content of the data memory within the specified address range
   void print_memory(unsigned start_address, unsigned end_address);

   // writes an integer value to data memory at the specified address (use little-endian format: https://en.wikipedia.org/wiki/Endianness)
   void write_memory(unsigned address, unsigned value);

   //prints the values of the registers 
   void print_registers();

   //prints the status of processor excluding memory
   void print_status();

   // prints the content of the ROB
   void print_rob();

   //prints the content of the reservation stations
   void print_reservation_stations();

   //print the content of the instruction window
   void print_pending_instructions();

   //print the whole execution history 
   void print_log();
   instructT fetchInstruction ( unsigned pc ) ;
   bool fetch();
   bool dispatch();
   void predispatch();
   bool store_conflict(int loadTag, unsigned memAddress, bool& bypassReady, uint32_t& bypassValue );
   bool issue() ;
   bool execute();
   uint32_t aluGetOutput(dynInstructT* dyn_instruction_p, unsigned src1V, unsigned src2V, uint32_t addr, bool& miss_prediction);
   bool writeResult(vector<res_station_t>& resGCUnit, vector<int>& resGCIndex);
   void wakeupAndRob(reservation_station_t* resP, uint32_t output, vector<res_station_t>& resGCUnit, vector<int>& resGCIndex);
   void doExec(execWrLaneT* laneP, bool doWr);
   bool commit(int& popCount);
   void squash();
   bool register_busy_check(uint32_t regNo, bool isF) ;
   exe_unit_t opcodeToExUnit(opcode_t opcode);
   int exLatency(opcode_t opcode) ;
   uint32_t agen (reservation_station_t* resP) ;
   unsigned aluF (unsigned _value1, unsigned _value2, bool value1F, bool value2F, opcode_t opcode);
   unsigned alu (unsigned _value1, unsigned _value2, bool value1F, bool value2F, opcode_t opcode);
   unsigned read_register(unsigned reg, bool isF);
   unsigned register_rename(unsigned reg, bool is_floating, unsigned& tag, bool& ready);
   unsigned get_reg_tag(unsigned reg, bool isF);
   unsigned read_memory(unsigned address);
   void set_fp_reg_tag(unsigned reg, int tag, bool busy);
   void set_int_reg_tag(unsigned reg, int tag, bool busy);
   int indexToOffset( uint32_t line_index, uint32_t pc_index );
   int labelResolve(string label, 
         map <string, int>& label_to_linenum,
         map <string, vector <int> >& unresolved_label_index,
         int line_num );
   void getReg( istringstream& buff_iss, uint32_t& reg, bool& regF, bool with_brackets=false );
   int parse( const string filename, unsigned base_address );
   bool static resStSort( reservation_station_t* a, reservation_station_t* b ) { return a->id < b->id; };
};

#endif /*SIM_OOO_H_*/
