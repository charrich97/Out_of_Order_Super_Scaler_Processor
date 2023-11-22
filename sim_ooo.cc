#include "sim_ooo.h"

using namespace std;

//used for debugging purposes
static const char *stage_names[NUM_STAGES] = {"ISSUE", "EXE", "WR", "COMMIT"};
static const char *instr_names[NUM_OPCODES] = {"LW", "SW", "ADD", "ADDI", "SUB", "SUBI", "XOR", "XORI", "OR", "ORI", "AND", "ANDI", "MULT", "DIV", "BEQZ", "BNEZ", "BLTZ", "BGTZ", "BLEZ", "BGEZ", "JUMP", "EOP", "LWS", "SWS", "ADDS", "SUBS", "MULTS", "DIVS"};
static const char *res_station_names[5]={"Int", "Load", "Add", "Mult"};

map <string, opcode_t> opcode_2str = { {"LW", LW}, {"SW", SW}, {"ADD", ADD}, {"ADDI", ADDI}, {"SUB", SUB}, {"SUBI", SUBI}, {"XOR", XOR}, {"XORI", XORI}, {"OR", OR}, {"ORI", ORI}, {"AND", AND}, {"ANDI", ANDI}, {"MULT", MULT}, {"DIV", DIV}, {"BEQZ", BEQZ}, {"BNEZ", BNEZ}, {"BLTZ", BLTZ}, {"BGTZ", BGTZ}, {"BLEZ", BLEZ}, {"BGEZ", BGEZ}, {"JUMP", JUMP}, {"EOP", EOP}, {"LWS", LWS}, {"SWS", SWS}, {"ADDS", ADDS}, {"SUBS", SUBS}, {"MULTS", MULTS}, {"DIVS", DIVS}};

map <exe_unit_t, res_station_t> ex_2Rs = { {INTEGER, INTEGER_RS}, {ADDER, ADD_RS}, {MULTIPLIER, MULT_RS}, {DIVIDER, MULT_RS}, {MEMORY, LOAD_B} };
//------------------------------------convert functions begin--------------------------------------------------------------//
/* convert a float into an unsigned */
inline unsigned float2unsigned(float value){
        unsigned result;
        memcpy(&result, &value, sizeof value);
        return result;
}

/* convert an unsigned into a float */
inline float unsigned2float(unsigned value){
        float result;
        memcpy(&result, &value, sizeof value);
        return result;
}

/* convert integer into array of unsigned char - little indian */
inline void unsigned2char(unsigned value, unsigned char *buffer){
        buffer[0] = value & 0xFF;
        buffer[1] = (value >> 8) & 0xFF;
        buffer[2] = (value >> 16) & 0xFF;
        buffer[3] = (value >> 24) & 0xFF;
}

/* convert array of char into integer - little indian */
inline unsigned char2unsigned(unsigned char *buffer){
       return buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24);
}
//-------------------------------------convert functions end-------------------------------------------------------------//

sim_ooo::sim_ooo(unsigned mem_size,
                unsigned rob_size,
                unsigned num_int_res_stations,
                unsigned num_add_res_stations,
                unsigned num_mul_res_stations,
                unsigned num_load_res_stations,
                unsigned max_issue){

	data_memory_size       = mem_size;
   robSize                = rob_size;
   issueWidth             = max_issue;

   resStSize              = new unsigned[RS_TOTAL];
   resStSize[INTEGER_RS]  = num_int_res_stations;
   resStSize[ADD_RS]      = num_add_res_stations;
   resStSize[MULT_RS]     = num_mul_res_stations;
   resStSize[LOAD_B]      = num_load_res_stations;

   //Allocating issue queue, ROB, reservation stations
	data_memory            = new unsigned char[data_memory_size];
   rob                    = Fifo<reorder_buf_t>( rob_size );
   gSquash                = false;
   memBlock               = false;

   reset();
}
	
sim_ooo::~sim_ooo(){
}

void sim_ooo::init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances){
   execFp[exec_unit].init(instances, latency);
}

void sim_ooo::load_program(const char *filename, unsigned base_address){
   instMemSize               = parse(string(filename), base_address);
   PC                        = base_address;
}

instructT sim_ooo::fetchInstruction ( unsigned pc ) {
   int      index     = (pc - this->baseAddress)/4;
   ASSERT((index >= 0) && (index < instMemSize), "out of bound access of instruction memory %d", index);
   instructT instruct = *(instMemory[index]);
   return instruct;
}

unsigned sim_ooo::register_rename(unsigned reg, bool is_floating, unsigned& tag, bool& ready){
   unsigned value         = UNDEFINED;
   ready                  = true;
   tag                    = UNDEFINED;
   if(register_busy_check(reg, is_floating)) {
      tag                                 = get_reg_tag(reg, is_floating);
      reorder_buf_t* rob_pointer          = rob.peekIndex(tag);
      if(rob_pointer->ready){
         value            = rob_pointer->value;
         tag              = UNDEFINED;
      }
      else{
         ready            = false;
      }
   }
   else{
      value               = read_register(reg, is_floating);
   }
   return value;
}

bool sim_ooo::fetch(){
   for (int j = 0; j < issueWidth && !rob.isFull(); j++){
      instructT instruct     = fetchInstruction ( PC );

      if( instruct.opcode == EOP ){
         return false;
      }

      exe_unit_t unit        = opcodeToExUnit(instruct.opcode);
      res_station_t rUnit    = ex_2Rs[unit];

      ASSERT(resStation[rUnit].size() <= resStSize[rUnit] , 
            "Illegal resStation size found for %s (%lu > %u)", res_station_names[rUnit], resStation[rUnit].size(), resStSize[rUnit]);

      if (resStation[rUnit].size() < resStSize[rUnit]) {
         reorder_buf_t robEntry;

         dynamic_instruct_pointer dyn_instruction_p   = new dynInstructT(instruct);
         dyn_instruction_p->stat.state     = ISSUE;
         dyn_instruction_p->stat.t_issue   = cycleCount;

         robEntry.dyn_instruction_p        = dyn_instruction_p;

         if(instruct.is_store)
            robEntry.mem_latency = execFp[MEMORY].latency;

         uint32_t robIndex      = rob.push(robEntry);

         reservation_station_t* resP      = new reservation_station_t();

         resP->dyn_instruction_p           = dyn_instruction_p;

         if( instruct.src1Valid ){
            uint32_t qj;
            resP->vj            = register_rename(instruct.src1, instruct.src1F, qj, resP->vj_ready);
            resP->qj            = qj;
         }
         if( instruct.src2Valid ){
            uint32_t qk;
            resP->vk            = register_rename(instruct.src2, instruct.src2F, qk, resP->vk_ready);
            resP->qk            = qk;
         }
         resP->tag_res_stat             = robIndex;

         if( unit == MEMORY )
            resP->addr          = instruct.imm;

         vector<reservation_station_t*> resSt = resStation[rUnit];
         sort(resSt.begin(), resSt.end(), resStSort);
         int id                 = resSt.size();
         for( int index = 0; index < (int)resSt.size(); index++ ){
            if( resSt[index]->id != index ){
               id               = index;
               break;
            }
         }
         resP->id               = id;

         resStation[rUnit].push_back( resP );

         PC                     = PC + 4;

         if(instruct.dstValid){
            if(instruct.dstF)
               set_fp_reg_tag(instruct.dst, robIndex, true); 
            else
               set_int_reg_tag(instruct.dst, robIndex, true); 
         }
      }
      else{
         break;
      }
   }
   return true;
}

bool sim_ooo::dispatch(){
   bool status = false;
   for(int unit = 0; unit < RS_TOTAL; unit++) {
      for(unsigned payIndex = 0; payIndex < resStation[unit].size(); payIndex++) {
         status                = true;
         reservation_station_t* resP     = resStation[unit][payIndex];

         bool instReady        = true;
         bool bypassReady      = false;
         uint32_t bypassValue  = UNDEFINED;
         bool is_store         = resP->dyn_instruction_p->is_store;
         bool is_load          = resP->dyn_instruction_p->is_load;
         uint32_t addr         = agen(resP);
         
         if( is_load ){
            instReady      = !store_conflict(resP->tag_res_stat, addr, bypassReady, bypassValue);
         } 

         if( is_store && resP->vk_ready ){
            rob.peekIndex( resP->tag_res_stat )->dest       = addr;
         }

         if ( !resP->pushed_2_exec && resP->vj_ready && resP->vk_ready && instReady ){
            int execUnit   = opcodeToExUnit(resP->dyn_instruction_p->opcode);
            int numLanes   = execFp[execUnit].numLanes;
            bool isMem     = execUnit == MEMORY;
                  
            if( is_store || bypassReady ){
               resP->pushed_2_exec                                     = true;
               execWrLaneT lane;
               lane.payloadP                                    = resP;
               lane.outputReady                                 = is_load && bypassReady;
               lane.output                                      = (is_load && bypassReady) ? bypassValue : UNDEFINED;
               bypassLane.push_back( lane );
            }
            else{
               for(int laneId = 0; laneId < numLanes; laneId++){
                  if(execFp[execUnit].lanes[laneId].ttl == 0 && ((isMem && !memBlock) || !isMem)){
                     resP->pushed_2_exec                               = true;
                     execFp[execUnit].lanes[laneId].payloadP    = resP;
                     uint32_t ttl                               = execFp[execUnit].latency;

                     execFp[execUnit].lanes[laneId].ttl         = ttl + 1;

                     execFp[execUnit].lanes[laneId].outputReady = false;
                     execFp[execUnit].lanes[laneId].output      = UNDEFINED;
                     break;
                  }
               }
            }
         }
      }
   }
   return status;
}

bool sim_ooo::store_conflict(int loadTag, unsigned memAddress, bool& bypassReady, uint32_t& bypassValue){
   bool conflict               = false;
   bypassReady                 = false;
   for(int i = 0; i < rob.getCount(); i++){
      int tag                           = rob.genIndex(i);
      reorder_buf_t* robEntryP          = rob.peekNth(i);

      if( robEntryP->dyn_instruction_p->is_store ){
         if( robEntryP->dest == UNDEFINED ){
            conflict           = true;
            bypassReady        = false;
         }
         else if(robEntryP->dest == memAddress){
            conflict           = !robEntryP->ready; 
            bypassReady        = robEntryP->ready;  
            bypassValue        = robEntryP->value;
         }
      }
      if(tag == loadTag)
         return conflict;
   }
   return conflict;
}

bool sim_ooo::issue() {
   bool status = fetch();
   status     |= dispatch();
   return status;
}

void sim_ooo::doExec(execWrLaneT* laneP, bool doWr){
   reservation_station_t* resP             = laneP->payloadP;
   if( resP->dyn_instruction_p->stat.state != EXECUTE ){
      resP->dyn_instruction_p->stat.state     = EXECUTE;
      resP->dyn_instruction_p->stat.t_execute = cycleCount;
   }
   bool is_store                 = resP->dyn_instruction_p->is_store;
   bool is_load                  = resP->dyn_instruction_p->is_load;

   if( is_store || is_load )
      resP->addr                              = agen(resP);

   if( doWr ) {
      if( !laneP->outputReady ){
         laneP->output           = aluGetOutput(resP->dyn_instruction_p, resP->vj, resP->vk, resP->addr, rob.peekIndex( resP->tag_res_stat )->miss_prediction);
      }
      else if( is_load )
         resP->vk                = laneP->output;

      laneP->outputReady         = true;
   }
}

bool sim_ooo::execute(){
   bool status = false;
   //TODO: check it
   // Check bypass lanes
   for( int i = 0; i < bypassLane.size(); i++ ){
      status  = true;
      doExec( &(bypassLane[i]), true );
   }

   //iterating through execution units
   for(int i = 0; i < EX_TOTAL; i++){
      //iterating through number of instances of an EXEC UNIT
      for(int j = 0; j < execFp[i].numLanes; j++){
         execWrLaneT* laneP = &(execFp[i].lanes[j]);
         //Checking if TTL of the lane is not zero
         if( laneP->ttl  > 0 ){
            status = true;
            doExec( laneP, laneP->ttl == 1 );
         }
      }
   }
   return status;
}

uint32_t sim_ooo::aluGetOutput(dynInstructT* dyn_instruction_p, unsigned src1V, unsigned src2V, uint32_t addr, bool& miss_prediction){
   bool src1F      = dyn_instruction_p->src1F;
   bool src2F      = dyn_instruction_p->src2F;
   opcode_t opcode = dyn_instruction_p->opcode;
   uint32_t imm    = dyn_instruction_p->imm;
   uint32_t npc    = dyn_instruction_p->pc + 4;
   unsigned aluOut = UNDEFINED;
   miss_prediction         = false;

   switch(opcode) {
      case LW:
      case LWS:
         aluOut  = read_memory (addr);
         break;

      case ADD ... DIV:
      case ADDS ... DIVS:
         aluOut  = alu(src1V, src2V, src1F, src2F, opcode);
         break;

      case ADDI ... ANDI:
         aluOut  = alu(src1V, imm, src1F, false, opcode);
         break;

      case BLTZ:
         miss_prediction = src1V < 0;
         aluOut  = miss_prediction ? alu(npc, imm, false, false, opcode) : npc;
         break;

      case BNEZ:
         miss_prediction = src1V != 0;
         aluOut  = miss_prediction ? alu(npc, imm, false, false, opcode) : npc;
         break;

      case BEQZ:
         miss_prediction = src1V == 0;
         aluOut  = miss_prediction ? alu(npc, imm, false, false, opcode) : npc;
         break;

      case BGTZ:
         miss_prediction = src1V > 0;
         aluOut  = miss_prediction ? alu(npc, imm, false, false, opcode) : npc;
         break;

      case BGEZ:
         miss_prediction = src1V >= 0;
         aluOut  = miss_prediction ? alu(npc, imm, false, false, opcode) : npc;
         break;

      case BLEZ:
         miss_prediction = src1V <= 0;
         aluOut  = miss_prediction ? alu(npc, imm, false, false, opcode) : npc;
         break;

      case JUMP:
         miss_prediction = true;
         aluOut  = alu(npc, imm, false, false, opcode);
         break;

      case SW:
      case SWS:
         aluOut = src1V;
         break;
      case EOP:
         break;

      default:
         ASSERT(false, "Unknown operation encountered");
         break;
   }
   return aluOut;
}
void sim_ooo::wakeupAndRob(reservation_station_t* resP, uint32_t output, vector<res_station_t>& resGCUnit, vector<int>& resGCIndex){
   int resDelIndex       = -1;
   res_station_t resDelUnit;
   for( int unit = 0; unit < RS_TOTAL; unit++ ){
      for(uint32_t k = 0; k < resStation[unit].size(); k++) {
         reservation_station_t* resWakeP   = resStation[unit][k];

         if( resP->tag_res_stat == resWakeP->tag_res_stat ){
            resDelIndex          = k;
            resDelUnit           = (res_station_t)unit;
         }

         if(resWakeP->qj == resP->tag_res_stat) {
            resWakeP->vj  = output;
            resWakeP->vj_ready = true;
            resWakeP->qj  = UNDEFINED;
         }
         if(resWakeP->qk == resP->tag_res_stat) {
            resWakeP->vk  = output;
            resWakeP->vk_ready = true;
            resWakeP->qk  = UNDEFINED;
         }
      }
   }
   rob.peekIndex( resP->tag_res_stat )->value = output;
   rob.peekIndex( resP->tag_res_stat )->ready = true;

   ASSERT( resDelIndex != -1, "resDelIndex == -1" );
   resGCUnit.push_back( resDelUnit );
   resGCIndex.push_back( resDelIndex );
}

bool sim_ooo::writeResult(vector<res_station_t>& resGCUnit, vector<int>& resGCIndex){
   bool status = false;
   vector<int> bypassDelIndex;

   for( int i = 0; i < bypassLane.size(); i++ ){
      reservation_station_t* resP             = bypassLane[i].payloadP;
      if( resP->dyn_instruction_p->stat.state == EXECUTE ){
         status                       = true;
         resP->dyn_instruction_p->stat.state     = WRITE_RESULT;
         resP->dyn_instruction_p->stat.t_wr      = cycleCount;
         wakeupAndRob( resP, bypassLane[i].output, resGCUnit, resGCIndex );
         bypassDelIndex.push_back(i);
      }
   }

   int erase = 0;
   for( int i = 0; i < bypassDelIndex.size(); i++ ){
      bypassLane.erase( bypassLane.begin() + i - erase  );
      erase++;
   }

   for(int i = 0; i < EX_TOTAL; i++){
      for(int j = 0; j < execFp[i].numLanes; j++){
         if(execFp[i].lanes[j].ttl > 0){
            execFp[i].lanes[j].ttl--;
            if(execFp[i].lanes[j].ttl == 0){
               status                    = true;
               reservation_station_t* resP         = execFp[i].lanes[j].payloadP;
               execWrLaneT* laneP        = &(execFp[i].lanes[j]);

               resP->dyn_instruction_p->stat.state  = WRITE_RESULT;
               resP->dyn_instruction_p->stat.t_wr   = cycleCount;

               ASSERT( laneP->outputReady, "At WriteResult, output not ready!" );

               wakeupAndRob( resP, laneP->output, resGCUnit, resGCIndex );
            }
         }
      }
   }
   return status;
}

bool sim_ooo::commit(int& popCount){
   bool status     = false;
   popCount        = 0;
   int commitWidth = 1; 
   for(int i = 0; (i < commitWidth) && (i < rob.getCount()); i++){
      reorder_buf_t* head       = rob.peekNth(i);
      int headTag               = rob.genIndex(i);
      status                    = true;
      if(head->ready){
         if(head->dyn_instruction_p->is_store) {
            if( execFp[MEMORY].lanes[0].ttl != 0 ){
               break;
            }
            memBlock                 = true;
         }

         if( head->dyn_instruction_p->stat.state != COMMIT ){
            head->dyn_instruction_p->stat.state    = COMMIT;
            head->dyn_instruction_p->stat.t_commit = cycleCount;
         }

         if(head->dyn_instruction_p->is_store) {
            head->mem_latency--;
            if(head->mem_latency != 0){
               break;
            }
            else{
               memBlock = false;
               write_memory(head->dest, head->value);
            }
         }


         instCount++;
         gSquash      = head->dyn_instruction_p->is_branch && head->miss_prediction;

         if(head->dyn_instruction_p->dstValid){
            if(head->dyn_instruction_p->dstF){
               fp_file[head->dyn_instruction_p->dst].value    = unsigned2float(head->value);
               if( headTag == fp_file[head->dyn_instruction_p->dst].tag )
                  fp_file[head->dyn_instruction_p->dst].busy  = false; 
            } 
            else {
               int_file[head->dyn_instruction_p->dst].value   = head->value;
               if( headTag == int_file[head->dyn_instruction_p->dst].tag )
                  int_file[head->dyn_instruction_p->dst].busy = false; 
            }
         }


         if(gSquash){
            break;
         }

         popCount++;
      }
      else{
         break;
      }
   }
   return status;
}

//---------------------------------------------------------------------------------------------------------//

void sim_ooo::run(unsigned cycles){
   bool rtc    = (cycles == 0);
   bool status = true;
   while((rtc && status) || cycles) {
      // For feedback FF
      int popCount;
      vector<res_station_t> resGCUnit;
      vector<int>           resGCIndex;

      status    = commit(popCount);
      status   |= writeResult(resGCUnit, resGCIndex);
      status   |= execute();
      status   |= issue();

      if( !gSquash ){
         int *delElems = (int*) calloc(RS_TOTAL, sizeof(int));
         for( unsigned i = 0; i < resGCUnit.size(); i++ ){
            res_station_t unit = resGCUnit[i];
            resStation[unit].erase( resStation[unit].begin() + resGCIndex[i] - delElems[unit] );
            delElems[unit]++;
         }

         for( int i = 0; i < popCount; i++ ){
            bool underflow;
            reorder_buf_t robEntry  = rob.pop(underflow);
            instStatT stat;
            stat.pc        = robEntry.dyn_instruction_p->pc;
            stat.t_issue   = robEntry.dyn_instruction_p->stat.t_issue;
            stat.t_execute = robEntry.dyn_instruction_p->stat.t_execute;
            stat.t_wr      = robEntry.dyn_instruction_p->stat.t_wr;
            stat.t_commit  = robEntry.dyn_instruction_p->stat.t_commit;
            log.push_back(stat);
            ASSERT(!underflow, "ROB underflown");
         }

      }
      else{
         squash(); 
         status   = true;
      }

      cycles = cycles > 0 ? cycles - 1 : 0;
      cycleCount++;
      gSquash       = false;
   }
}

void sim_ooo::reset(){
   for(unsigned i = 0; i < data_memory_size; i++) {
      data_memory[i]   = (unsigned char)UNDEFINED; 
   }

   for(int i = 0; i < NUM_GP_REGISTERS; i++) {
      int_file[i].value = UNDEFINED;
   }

   for(int i = 0; i < NUM_FP_REGISTERS; i++) {
      fp_file[i].value  = UNDEFINED;
   }

   squash();
}

void sim_ooo::squash(){
   for(int i = 0; i < EX_TOTAL; i++){
      for(int j = 0; j < execFp[i].numLanes; j++){
         execFp[i].lanes[j].ttl = 0;
      }
   }
   for(int i = 0; i < RS_TOTAL; i++){
      resStation[i].clear();
   }

   PC                = rob.peekHead()->value;
   int popCount      = rob.getCount();
   for( int i = 0; i < popCount; i++ ){
      bool underflow;
      reorder_buf_t robEntry  = rob.pop(underflow);
      instStatT stat;
      stat.pc        = robEntry.dyn_instruction_p->pc;
      stat.t_issue   = robEntry.dyn_instruction_p->stat.t_issue;
      stat.t_execute = robEntry.dyn_instruction_p->stat.t_execute;
      stat.t_wr      = robEntry.dyn_instruction_p->stat.t_wr;
      stat.t_commit  = robEntry.dyn_instruction_p->stat.t_commit;
      log.push_back(stat);
      ASSERT(!underflow, "ROB underflown");
   }
   rob.popAll();

   for(int i = 0; i < NUM_FP_REGISTERS; i++) {
      fp_file[i].busy  = false;
   }

   for(int i = 0; i < NUM_GP_REGISTERS; i++) {
      int_file[i].busy = false;
   }
}


bool sim_ooo::register_busy_check(uint32_t regNo, bool isF) {
   return isF ? fp_file[regNo].busy : int_file[regNo].busy;
}

exe_unit_t sim_ooo::opcodeToExUnit(opcode_t opcode){
   exe_unit_t unit;
   switch( opcode ){
      case ADD ... AND:
      case ADDI ... ANDI:
      case BEQZ ... EOP:
         unit = INTEGER;
         break;

      case ADDS:
      case SUBS:
         unit = ADDER;
         break;

      case MULTS:
      case MULT:
         unit = MULTIPLIER;
         break;

      case DIVS:
      case DIV:
         unit = DIVIDER;
         break;

      case LW:
      case LWS:
      case SW:
      case SWS:
         unit = MEMORY;
         break;

      default: 
         ASSERT (false, "Opcode not supported");
         unit = INTEGER;
         break;
   }
   ASSERT( execFp[unit].numLanes > 0, "No lanes found for opcode: %s", opcode_str[opcode].c_str());
   return unit;
}

int sim_ooo::exLatency(opcode_t opcode) {
   return execFp[opcodeToExUnit(opcode)].latency;
}

uint32_t sim_ooo::agen ( reservation_station_t* resP ) {
   uint32_t stAddr  = resP->dyn_instruction_p->imm + (int) resP->vk;
   uint32_t ldAddr  = resP->dyn_instruction_p->imm + (int) resP->vj;
   return resP->dyn_instruction_p->is_store ? stAddr : (resP->dyn_instruction_p->is_load ? ldAddr : UNDEFINED);
}

//ALU function for floating point operations
unsigned sim_ooo::aluF (unsigned _value1, unsigned _value2, bool value1F, bool value2F, opcode_t opcode){
   float output;
   float value1 = value1F ? unsigned2float(_value1) : _value1;
   float value2 = value2F ? unsigned2float(_value2) : _value2; 

   switch( opcode ){
      case ADD:
      case ADDI:
      case BEQZ ... ADDS:
         output      = value1 + value2;
         break;

      case SUB:
      case SUBI:
      case SUBS:
         output      = value1 - value2;
         break;

      case XOR:
      case XORI:
         output      = (unsigned)value1 ^ (unsigned)value2;
         break;

      case AND:
      case ANDI:
         output      = (unsigned)value1 & (unsigned)value2;
         break;

      case OR:
      case ORI:
         output      = (unsigned)value1 | (unsigned)value2;
         break;

      case MULT:
      case MULTS:   
         output      = value1 * value2;
         break;

      case DIV:
      case DIVS:
         output      = value1 / value2;
         break;

      default: 
         output      = UNDEFINED;
         break;
   }
   return float2unsigned(output);
}

//function to perform ALU operations
unsigned sim_ooo::alu (unsigned _value1, unsigned _value2, bool value1F, bool value2F, opcode_t opcode){

   if( value1F || value2F ) return aluF(_value1, _value2, value1F, value2F, opcode);

   unsigned output;
   unsigned value1   = value1F ? float2unsigned(_value1) : _value1;
   unsigned value2   = value2F ? float2unsigned(_value2) : _value2; 

   switch( opcode ){
      case ADD:
      case ADDI:
      case BEQZ ... ADDS:
         output      = value1 + value2;
         break;

      case SUB:
      case SUBI:
      case SUBS:
         output      = value1 - value2;
         break;

      case XOR:
      case XORI:
         output      = (unsigned)value1 ^ (unsigned)value2;
         break;

      case AND:
      case ANDI:
         output      = (unsigned)value1 & (unsigned)value2;
         break;

      case OR:
      case ORI:
         output      = (unsigned)value1 | (unsigned)value2;
         break;

      case MULT:
      case MULTS:   
         output      = value1 * value2;
         break;

      case DIV:
      case DIVS:
         output      = value1 / value2;
         break;

      default: 
         output      = UNDEFINED;
         break;
   }
   return output;
}



unsigned sim_ooo::read_register(unsigned reg, bool isF){
   if(register_busy_check(reg, isF)) return UNDEFINED;
   return isF ? float2unsigned(fp_file[reg].value) : int_file[reg].value;
}
unsigned sim_ooo::get_reg_tag(unsigned reg, bool isF){
   if(register_busy_check(reg, isF)) return isF ? fp_file[reg].tag : int_file[reg].tag;
   return UNDEFINED;
}

int sim_ooo::get_int_register(unsigned reg){
	return int_file[reg].value; 
}

void sim_ooo::set_int_register(unsigned reg, int value){
   int_file[reg].value = value;
}

float sim_ooo::get_fp_register(unsigned reg){
	return fp_file[reg].value;
}

void sim_ooo::set_fp_register(unsigned reg, float value){
   fp_file[reg].value = value;
}

void sim_ooo::set_fp_reg_tag(unsigned reg, int tag, bool busy){
   fp_file[reg].tag   = tag;
   fp_file[reg].busy  = busy;
}

void sim_ooo::set_int_reg_tag(unsigned reg, int tag, bool busy){
   int_file[reg].tag   = tag;
   int_file[reg].busy  = busy;
}

unsigned sim_ooo::get_pending_int_register(unsigned reg){
   return get_reg_tag(reg, false);
}

unsigned sim_ooo::get_pending_fp_register(unsigned reg){
   return get_reg_tag(reg, true);
}
//-------------------------------------------------------------------------------------------------------//

void sim_ooo::print_status(){
	print_pending_instructions();
	print_rob();
	print_reservation_stations();
	print_registers();
}

void sim_ooo::print_memory(unsigned start_address, unsigned end_address){
	cout << "DATA MEMORY[0x" << hex << setw(8) << setfill('0') << start_address << ":0x" << hex << setw(8) << setfill('0') <<  end_address << "]" << endl;
	for (unsigned i=start_address; i<end_address; i++){
		if (i%4 == 0) cout << "0x" << hex << setw(8) << setfill('0') << i << ": "; 
		cout << hex << setw(2) << setfill('0') << int(data_memory[i]) << " ";
		if (i%4 == 3){
			cout << endl;
		}
	} 
}

//---------------------------READ AND WRITE MEMORY FUNCTIONS BEGIN--------------------------------------//

void sim_ooo::write_memory(unsigned address, unsigned value){
   ASSERT( address % 4 == 0, "Unaligned memory access found at address %x", address ); 
   ASSERT ( (address >= 0) && (address < data_memory_size), "Out of bounds memory accessed: Seg Fault!!!!" );
	unsigned2char(value,data_memory+address);
}

unsigned sim_ooo::read_memory(unsigned address){
   ASSERT( address % 4 == 0, "Unaligned memory access found at address %x", address ); 
   ASSERT ( (address >= 0) && (address < data_memory_size), "Out of bounds memory accessed: Seg Fault!!!!" );
   return char2unsigned(data_memory+address);
}
//---------------------------READ AND WRITE MEMORY FUNCTIONS END----------------------------------------//

//------------------------------------------------------------------------------------------------------//
void sim_ooo::print_registers(){
        unsigned i;
	cout << "GENERAL PURPOSE REGISTERS" << endl;
	cout << setfill(' ') << setw(8) << "Register" << setw(22) << "Value" << setw(5) << "ROB" << endl;
        for (i=0; i< NUM_GP_REGISTERS; i++){
                if (get_pending_int_register(i)!=UNDEFINED) 
			cout << setfill(' ') << setw(7) << "R" << dec << i << setw(22) << "-" << setw(5) << get_pending_int_register(i) << endl;
                else if (get_int_register(i)!=(int)UNDEFINED) 
			cout << setfill(' ') << setw(7) << "R" << dec << i << setw(11) << get_int_register(i) << hex << "/0x" << setw(8) << setfill('0') << get_int_register(i) << setfill(' ') << setw(5) << "-" << endl;
        }
	for (i=0; i< NUM_GP_REGISTERS; i++){
                if (get_pending_fp_register(i)!=UNDEFINED) 
			cout << setfill(' ') << setw(7) << "F" << dec << i << setw(22) << "-" << setw(5) << get_pending_fp_register(i) << endl;
                else if (get_fp_register(i)!=UNDEFINED) 
			cout << setfill(' ') << setw(7) << "F" << dec << i << setw(11) << get_fp_register(i) << hex << "/0x" << setw(8) << setfill('0') << float2unsigned(get_fp_register(i)) << setfill(' ') << setw(5) << "-" << endl;
	}
	cout << endl;
}

void sim_ooo::print_rob(){
   unsigned i;
	cout << "REORDER BUFFER" << endl; 
	cout << setfill(' ') << setw(5) << "Entry" << setw(6) << "Busy" << setw(7) << "Ready" << setw(12) << "PC" << setw(10) << "State" << setw(6) << "Dest" << setw(12) << "Value" << endl;
   for(i = 0; i < robSize; i++){
      bool busy            = rob.isBusy(i);

      reorder_buf_t* rob_pointer           = busy ? rob.peekIndex(i) : NULL;
      dynamic_instruct_pointer dyn_instruction_p = busy ? rob_pointer->dyn_instruction_p : NULL;
      bool ready           = busy ? busy && (rob_pointer->ready) : false;

      if( !busy )
         cout << setfill(' ') << setw(5) << i << setw(6) << "no" << setw(7) << "no" << setw(12) << "-" << setw(10) << "-" << setw(6) << "-" << setw(12) << "-" << endl;
      else{
         cout << setfill(' ') << setw(5) << i << setw(6) << (busy ? "yes" : "no") << setw(7) << (ready ? "yes" : "no") << setw(4) << "0x" << setw(8) << setfill('0') << hex << dyn_instruction_p->pc << setw(10) << setfill(' ') << stage_names[dyn_instruction_p->stat.state] << setw(5);

         if(dyn_instruction_p->dstValid){
            cout << (dyn_instruction_p->dstF ? "F" : "R");
            cout << dyn_instruction_p->dst;
         }
         else if (dyn_instruction_p->is_store){
            // Bad hotfix
            if(rob_pointer->dest == UNDEFINED || dyn_instruction_p->stat.state < EXECUTE )
               cout << "-";
            else
               cout << setw(8) << dec << rob_pointer->dest << setfill(' ');
         }
         else
            cout << "-";

         cout << setw(12);

         if(rob_pointer->value == UNDEFINED)
            cout << "-";
         else
            cout << setw(5) << setfill(' ') << "0x" << setw(8) << setfill('0') << hex << rob_pointer->value;
         cout << endl;
      }

   }
	
	cout << endl;
}

void sim_ooo::print_reservation_stations(){
	cout << "RESERVATION STATIONS" << endl;
	cout  << setfill(' ');
	cout << setw(7) << "Name" << setw(6) << "Busy" << setw(12) << "PC" << setw(12) << "Vj" << setw(12) << "Vk" << setw(6) << "Qj" << setw(6) << "Qk" << setw(6) << "Dest" << setw(12) << "Address" << endl; 
	
   for( int unit = 0; unit < RS_TOTAL; unit++ ){
      uint32_t id = 0;
      vector<reservation_station_t*> resSt = resStation[unit];
      sort(resSt.begin(), resSt.end(), resStSort);

      for( int i = 0; i < (int)resSt.size(); i++ ){
         reservation_station_t* resP  = resSt[i];
         for( int j = id; j < resSt[i]->id; j++ ){
            cout << setw(7) << res_station_names[unit] << j+1 << setw(6) << "no" << setw(12) << "-" << setw(12) << "-" << setw(12) << "-" << setw(6) << "-" << setw(6) << "-" << setw(6) << "-" << setw(12) << "-" << endl; 
         }
         id = resSt[i]->id + 1;
         cout << setw(7) << res_station_names[unit] << id << setw(6) << "yes";
         cout << setw(4) << "0x" << setw(8) << setfill('0') << hex << resP->dyn_instruction_p->pc << setfill(' ');

         if( resP->vj == UNDEFINED )
            cout << setw(12) << "-";
         else
            cout << setw(4) << setfill(' ') << "0x" << setw(8) << setfill('0') << hex << resP->vj << setfill(' ');

         if( resP->vk == UNDEFINED )
            cout << setw(12) << "-";
         else
            cout << setw(4) << setfill(' ') << "0x" << setw(8) << setfill('0') << hex << resP->vk << setfill(' ');


         if( resP->qj == UNDEFINED )
            cout << setw(6) << "-";
         else
            cout << setw(6)<< hex << resP->qj << setfill(' ');

         if( resP->qk == UNDEFINED )
            cout << setw(6) << "-";
         else
            cout << setw(6) << hex << resP->qk << setfill(' ');

         cout << setw(6) << resP->tag_res_stat;

         if( resP->addr == UNDEFINED )
            cout << setw(12) << "-";
         else
            cout << setw(4) << setfill(' ') << "0x" << setw(8) << setfill('0') << hex << resP->addr << setfill(' ');

         cout << endl;
      }
      for( unsigned i = id; i < resStSize[unit]; i++ ){
         cout << setw(7) << res_station_names[unit] << i+1 << setw(6) << "no" << setw(12) << "-" << setw(12) << "-" << setw(12) << "-" << setw(6) << "-" << setw(6) << "-" << setw(6) << "-" << setw(12) << "-" << endl; 
      }
   }

	cout << endl;
}

void sim_ooo::print_pending_instructions(){
	cout << "PENDING INSTRUCTIONS STATUS" << endl;
	cout << setfill(' ');
	cout << setw(10) << "PC" << setw(7) << "Issue" << setw(7) << "Exe" << setw(7) << "WR" << setw(7) << "Commit" << endl;
   for(unsigned i = 0; i < robSize; i++){
      bool busy            = rob.isBusy(i);
      reorder_buf_t* rob_pointer           = busy ? rob.peekIndex(i) : NULL;
      dynamic_instruct_pointer dP     = busy ? rob_pointer->dyn_instruction_p : NULL;

      if( !busy )
         cout << setw(10) << "-" << setw(7) << "-" << setw(7) << "-" << setw(7) << "-" << setw(7) << "-" << endl;
      else{
         cout << "0x" << setw(8) << setfill('0') << hex << rob_pointer->dyn_instruction_p->pc <<setfill(' ');
         if( dP->stat.t_issue == UNDEFINED ) cout << setw(7) << "-";
         else                           cout << setw(7) << dec << dP->stat.t_issue;

         if( dP->stat.t_execute == UNDEFINED ) cout << setw(7) << "-";
         else                             cout << setw(7) << dec << dP->stat.t_execute;

         if( dP->stat.t_wr == UNDEFINED ) cout << setw(7) << "-";
         else                        cout << setw(7) << dec << dP->stat.t_wr;

         if( dP->stat.t_commit == UNDEFINED ) cout << setw(7) << "-";
         else                            cout << setw(7) << dec << dP->stat.t_commit;

         cout << endl;
      }

   }
	cout << endl;
}

//-------------------------------------------------------------------------------------------------------------------------------------------//
void sim_ooo::print_log(){
   cout << "EXECUTION LOG" << endl;
   cout << setw(12) << setfill(' ') << "PC" << setw(7) << "Issue" << setw(7) << "Exe" << setw(7) << "WR" << setw(7) << "Commit" << endl;
   for( unsigned i = 0; i < log.size(); i++ ){
      cout << "0x" << setw(8) << hex << setfill('0') << log[i].pc << setw(7) << setfill(' ');
      if( log[i].t_issue == UNDEFINED )
         cout << "-";
      else
         cout << dec << log[i].t_issue;
      
      cout << setw(7);
      
      if( log[i].t_execute == UNDEFINED )
         cout << "-";
      else
         cout << log[i].t_execute;
      
      cout << setw(7);
      if( log[i].t_wr == UNDEFINED )
         cout << "-";
      else
         cout << log[i].t_wr;
      
      cout << setw(7);
      if( log[i].t_commit == UNDEFINED )
         cout << "-";
      else
         cout << log[i].t_commit;
      
      cout << endl;
   }
}

float sim_ooo::get_IPC(){
   return (double) get_instructions_executed() / (double) get_clock_cycles();
}
	
unsigned sim_ooo::get_instructions_executed(){
	return instCount; 
}

unsigned sim_ooo::get_clock_cycles(){
   return cycleCount - 1; 
}

//-------------------------------- Fifo FUNCS BEGIN -------------------------------
template <class T> Fifo<T>::Fifo( int size ){
   head              = 0;
   tail              = 0;
   count             = 0;
   this->size        = size;
   array             = NULL;
   if( size > 0 )
      array          = new T[ size ];
}

template <class T> Fifo<T>::Fifo(){
   Fifo(0);
}

template <class T> Fifo<T>::~Fifo(){
}

template <class T> uint32_t Fifo<T>::push( T entry ){
   // Push only if space is available
   // Overflow check condition
   int push_index = -1;
   if( count < size ){
      // Place data on tail and increment tail and count
      array[ tail ]        = entry;
      push_index           = tail;
      tail                 = ( tail + 1 ) % size;
      count++;
      // Un-necessary assert to check fifo sanity
      assert( count == getCountHeadTail( head, tail, true ) );
   }
   assert( push_index != -1 );
   return push_index;
}

template <class T> T Fifo<T>::pop( bool &underflow ){
   // return data from head, increment head and decrement count
   T entry      = array[ head ];
   // Underflow condition check
   if( count > 0 ){
      head      = ( head + 1 ) % size;
      count--;
      underflow = false;
   } else{
      underflow = true;
   }
   // Un-necessary assert to check fifo sanity
   assert( count == getCountHeadTail( head, tail ) );

   // FIXME: replace
   assert( !underflow );
   return entry;
}

template <class T> T* Fifo<T>::peekHead(){
   return &( array[ head ] );
}

// n is assumed to be indexed from 0
// 0 means head, count - 1 means tail
template <class T> T* Fifo<T>::peekNth( int n ){
   assert( n < count );
   uint32_t index = ( head + n ) % size;
   return &( array[ index ] );
}

template <class T> bool Fifo<T>::isBusy( int index ){
   // ----- head ---- index ----- tail -----
   // -- index --- tail --------- head -----
   // ----- tail -------- head ---- index --
   // ----- tail/head ------------- index --
   // ----- index -------------tail/head ---
   bool cond = ((index >= head && index < tail && tail > head)  || 
         (tail > index && head > tail && head > index)  ||
         (head > tail && index >= head && index > tail)  || 
         (tail == head && index >= head && count > 0)   || 
         (tail == head && head >= index && count > 0) );
   return cond;
}

// Peek a custom physical index
// NOTE: should be between head and tail
// TODO: verify
template <class T> T* Fifo<T>::peekIndex( int index ){
   assert( index < size );
   //assert( isBusy(index) );
   return &( array[ index ] );
}

template <class T> int Fifo<T>::getHeadIndex(){
   return head;
}

template <class T> int Fifo<T>::getTailIndex(){
   return tail;
}

template <class T> int Fifo<T>::getCount(){
   return count;
}

template <class T> void Fifo<T>::popAll(){
   tail           = 0;
   head           = 0;
   count          = 0;
}

// Phase: true if head == tail is to be considered full
//        false if head == tail is empty
template <class T> int Fifo<T>::getCountHeadTail( int head, int tail, bool phase){
   uint32_t count = ( tail > head ) ? ( tail - head ) : ( size - head + tail );
   return ( head == tail ) ? ( phase ? size : 0 ) : count;
}

// Phase: true if head == tail is to be considered full
//        false if head == tail is empty
template <class T> void Fifo<T>::moveHead( int head, bool phase, int count ){
   head           = head % size;
   int temp_count = getCountHeadTail( head, tail, phase );

   // Verify head pointer movement if requested (count != -1)
   assert( ( tail == head && ( count == 0 || count == size ) ) || count == -1 || temp_count == count );

   this->head     = head;
   this->count    = temp_count;
}

// Phase: true if head == tail is to be considered full
//        false if head == tail is empty
template <class T> void Fifo<T>::moveTail( int tail, bool phase, int count ){
   tail           = tail % size;
   int temp_count = getCountHeadTail( head, tail, phase );

   // Verify tail pointer movement if requested (count != -1)
   assert( ( tail == head && ( count == 0 || count == size ) ) || count == -1 || temp_count == count );

   this->tail     = tail;
   this->count    = temp_count;
}

template <class T> bool Fifo<T>::isFull(){
   return ( count == size );
}

template <class T> uint32_t Fifo<T>::genIndex( uint32_t ith ){
   return (head + ith) % size;
}

template <class T> bool Fifo<T>::isEmpty(){
   return ( count == 0 );
}

template <class T> uint32_t Fifo<T>::getSize(){
   return size;
}
//--------------------------------- Fifo FUNCS END---------------------------------

//----------------------------------------------PARSING OPERATION BEGINS---------------------------------//
int sim_ooo::indexToOffset( uint32_t line_index, uint32_t pc_index ){
   return ((line_index - pc_index - 1) * 4);
}

int sim_ooo::labelResolve(string label, 
                            map <string, int>& label_to_linenum,
                            map <string, vector <int>>& unresolved_label_index,
                            int line_num ){
   // (2) Check label in storage when branch is encoutered
   if( label_to_linenum.find(label) != label_to_linenum.end() ){
      // Label is found! Immediately compute offset (3)
      return indexToOffset( label_to_linenum[label], line_num );
   } else{
      // Label is not found! Store in vector (4) 
      unresolved_label_index[label].push_back( line_num );
   }
   return UNDEFINED;
}

void sim_ooo::getReg( istringstream& buff_iss, uint32_t& reg, bool& regF, bool with_brackets ){
   string reg_i_or_f, open_bracket, closed_bracket;

   if( with_brackets )
      buff_iss >> setw(1) >> /*open_bracket >> setw(1) >> */ reg_i_or_f >> reg >> closed_bracket;
   else
      buff_iss >> setw(1) >> reg_i_or_f >> reg;

   regF  = ( reg_i_or_f == "F" ) || ( reg_i_or_f == "f" );
}

/*
 * Details     : 1. Parses a given file using c++ ifstream, 
 *                  delimits string based on opcode and its
 *                  corresponding arguments
 *               2. Implements runtime label disambiguation
 *                  process for fast label to PC/offset
 *                  lookup
 *                  Disambiguation details:
 *                  1. Each label encountered in runtime,
 *                     is saved onto a map along with its
 *                     line number
 *                  2. When a branch is encountered, its
 *                     label is checked in the storage
 *                     created in (1)
 *                  3. If the label is found, offset is
 *                     updated immediately
 *                  4. If not found, its index is held in 
 *                     a vector
 *                  5. When a new label is processed, we
 *                     check for all previous encounters 
 *                     using vector in (4) and then
 *                     disambiguate
 * Returns     : Number of instructions successfully parsed
 * Side Effects: Populates class variable instMemory
 *
 * NOTES       : -NA-
 */
int sim_ooo::parse( const string filename, unsigned base_address ){
   int line_num = 0;
   string buff;

   // ASM handle to file
   ifstream asm_h;
   asm_h.open( filename, ifstream::in );
   ASSERT( asm_h.is_open(), "Unable to open file: %s", filename.c_str() ); 

   // Map for label to line number lookup used in offset
   // calculation (1)
   map <string, int> label_to_linenum;

   // Store all index to instMemory that have unresolved labels (4)
   map <string, vector <int>> unresolved_label_index;

   // Loop to iterate through each line in asm and extract
   // instructions
   // getline fetches string till a new line character is reached
   while( getline( asm_h, buff ) ) {
      instMemory               = (instructPT*) realloc(instMemory, (line_num + 1)*sizeof(instructPT));
      instructPT instructP     = new instructT;
      instructP->pc            = (line_num * 4) + base_address;

      instMemory[line_num]     = instructP;

      // At this point in code, buff has the entire line including
      // opcode and its arguments (eg. ADDI	R2 R0 0xA000)
      
      // Making istringstream out of buff gives us the capability
      // to extract strings delimited by whitespace easily
      // for ADDI	R2 R0 0xA000, istringstream would yeild 4 strings
      // namely, ADDI, R2, R0, 0xA000 which can be quite handy
      // NOTE: We cannot use extraction operator (described below)
      // on strings and hence, we use istringstream
      istringstream buff_iss(buff);

      // String to hold opcode
      string opcode;
      // String to hold info on integer/floating identifier
      // in instruction
      string label;
      string imm;

      // Loading up 1st string a.k.a opcode from buff_iss
      // >> is also called 'extraction' operator which extracts
      // 1st string out of buff_iss and assigns it to opcode
      // NOTE: opcode will be moved (i.e., will be removed from buff_is)
      buff_iss >> opcode;

      if( opcode_2str.count( opcode ) <= 0 ){
         // string.back() gives the last character in a string
         ASSERT( opcode.back() == ':', "Unkown 1st token(%s) encountered", opcode.c_str() );

         // At this point in code, opcode actually stores label except for last
         // character which is a ':'
         string label              = opcode.substr(0, opcode.length() - 1);

         // label must be saved along with line number for quick 
         // lookup and disambiguation process (1)
         label_to_linenum[ label ] = line_num;

         // Since we have got a new label, we should check for any ambiguities
         // and resolve them at this step (5)
         // NOTE: opcode here is the label
         if( unresolved_label_index.find( label ) != unresolved_label_index.end() ){
            // We do have some unresolved instructions..
            vector <int> unresolved_index = unresolved_label_index[label];
            for( unsigned index = 0; index < unresolved_index.size(); index++ ){
               // Disambiguate all previous encounters
               int inst_index              = unresolved_index[index];
               instMemory[inst_index]->imm = indexToOffset( line_num, inst_index );
            }
            // Delete the entry from unresolved list
            unresolved_label_index.erase( label );
         }

         // Since the assumed opcode was a label, let's extract next token
         // and assign it to opcode
         buff_iss >> opcode;
      }

      // Translate string to enum for handy usage
      instructP->opcode        = opcode_2str[ opcode ];

      //FIX_ME #1: use reg_i_or_f to flood struct field
      // setw(n) sets the number of characters to extract
      // eg., let's assume buff_iss has "R2 R0 0xA000"
      // buff_iss >> setw(1) >> reg_i_or_f
      // will extract R (1 character) from "R2" and stash
      // it in reg_i_or_f
      switch( instructP->opcode ){
         case ADD ... DIV:
         case ADDS ... DIVS:
            // The following 3 lines store respective I/F in
            // intermediate values of reg_i_or_f and converts
            // string to int implicitly as dst, src1, src2 
            // are of type int
            getReg( buff_iss, instructP->dst , instructP->dstF );
            getReg( buff_iss, instructP->src1, instructP->src1F );
            getReg( buff_iss, instructP->src2, instructP->src2F );
            instructP->dstValid   = true;
            instructP->src1Valid  = true;
            instructP->src2Valid  = true;
            break;

         case BEQZ ... BGEZ:
            getReg( buff_iss, instructP->src1, instructP->src1F );
            buff_iss >> label;
            instructP->imm        = labelResolve( label, label_to_linenum, unresolved_label_index, line_num );
            instructP->src1Valid  = true;
            instructP->is_branch  = true;
            break;

         case ADDI ... ANDI:
            getReg( buff_iss, instructP->dst , instructP->dstF );
            getReg( buff_iss, instructP->src1, instructP->src1F );
            buff_iss >> imm;
            // stod takes care of decimal, hex (0x) string internally
            instructP->imm        = stod(imm);
            instructP->dstValid   = true;
            instructP->src1Valid  = true;
            break;

         case JUMP:
            buff_iss >> label;
            instructP->imm        = labelResolve( label, label_to_linenum, unresolved_label_index, line_num );
            instructP->is_branch  = true;
            break;

         case LW:
         case LWS:
            getReg( buff_iss, instructP->dst , instructP->dstF );
            // Format to parse at this point of code: %d(R%d)
            // Interestingly, ->imm field is of type int
            // extracting from buff_iss onto ->imm will stop right
            // before "(" which is exactly what we want
            getline(buff_iss, imm, '(');
            instructP->imm        = stod(imm);
            // Following line is self explanatory on what is happening
            // at this point in code
            // Format to parse at this point of code: (R%d)
            getReg( buff_iss, instructP->src1, instructP->src1F, true );
            instructP->dstValid   = true;
            instructP->src1Valid  = true;
            instructP->is_load    = true;
            break;

         case SW:
         case SWS:
            getReg( buff_iss, instructP->src1, instructP->src1F );
            // Format to parse at this point of code: %d(R%d)
            // Interestingly, ->imm field is of type int
            // extracting from buff_iss onto ->imm will stop right
            // before "(" which is exactly what we want
            getline(buff_iss, imm, '(');
            instructP->imm        = stod(imm);
            // Following line is self explanatory on what is happening
            // at this point in code
            // Format to parse at this point of code: (R%d)
            getReg( buff_iss, instructP->src2, instructP->src2F, true );
            instructP->src2Valid  = true;
            instructP->src1Valid  = true;
            instructP->is_store   = true;
            break;

         case EOP:
            break;

         default:
            ASSERT(false, "Unknown operation encountered");
            break;
      }
      line_num++;
   }

   // At this point in code, there should be no ambiguity in labels
   uint32_t disambiguities = unresolved_label_index.size();
   ASSERT( disambiguities == 0, "Unable to disambiguate successfully (=%u)", disambiguities);
   return line_num;
}

//----------------------------------------------PARSING OPERATION ENDS-----------------------------------//



