/* ======================================================================
   CORTEX-M3 TRAFFIC CONTROL SIMULATOR
   ----------------------------------------------------------------------
   This file contains three independent modules that only ever talk to
   each other through the CPU's memory-mapped address space:

     1. CortexM3   - a small ARM-Cortex-M3-flavoured emulator: registers,
                     memory, ALU, a fetch/decode/execute loop, a timer
                     peripheral and an NVIC-style interrupt controller.
     2. Firmware   - real "instruction stream" programs (main loop, timer
                     ISR, emergency ISR) that run ON the CPU above. The
                     traffic-light decisions live here, not in JS logic.
     3. TrafficSim - the physical-world simulation (vehicles, queues,
                     spawning). It WRITES sensor data into memory and
                     only ever READS light state back out of GPIO.

   The renderer at the bottom just visualises whatever state (1) and (3)
   already contain. It never decides anything.
   ====================================================================== */

/* ---------------------- 1. MEMORY MAP -------------------------------- */
const ADDR = {
  // GPIO output region (traffic lights) - 0x4000_0000
  NS_RED:        0x40000000,
  NS_YELLOW:     0x40000004,
  NS_GREEN:      0x40000008,
  EW_RED:        0x4000000C,
  EW_YELLOW:     0x40000010,
  EW_GREEN:      0x40000014,

  // Sensor / input region - 0x5000_0000
  NS_COUNT:          0x50000000,
  EW_COUNT:          0x50000004,
  MODE:              0x50000008,   // 0=normal 1=adaptive 2=manual
  MANUAL_CMD:        0x5000000C,   // 0 = request NS green, 1 = request EW green
  MANUAL_TRIGGER:    0x50000010,   // set to 1 by UI, cleared by firmware once applied
  EMERGENCY_DIR:     0x50000014,   // 0 = NS, 1 = EW
  EMERGENCY_CLEAR:   0x50000018,   // set to 1 by traffic sim once emergency vehicle has passed

  // Controller working memory - 0x6000_0000
  PHASE:             0x60000000,   // 0..5 state-machine phase (normal cycle)
  GREEN_NS_DUR:      0x60000004,
  GREEN_EW_DUR:      0x60000008,
  EMERGENCY_ACTIVE:  0x6000000C,
  EMG_STAGE:         0x60000010,

  // Timer peripheral - 0x7000_0000 (writes here are intercepted by the engine)
  TIMER_LOAD:        0x70000000,
};

const GPIO_ADDRS = [ADDR.NS_RED, ADDR.NS_YELLOW, ADDR.NS_GREEN, ADDR.EW_RED, ADDR.EW_YELLOW, ADDR.EW_GREEN];
const GPIO_NAMES = { [ADDR.NS_RED]:'NS_RED', [ADDR.NS_YELLOW]:'NS_YELLOW', [ADDR.NS_GREEN]:'NS_GREEN',
                      [ADDR.EW_RED]:'EW_RED', [ADDR.EW_YELLOW]:'EW_YELLOW', [ADDR.EW_GREEN]:'EW_GREEN' };

/* ---------------------- 2. TINY ASSEMBLER ----------------------------
   Programs are written as arrays of instruction objects plus label
   markers ({lbl:'name'}). assemble() strips the markers out and resolves
   every branch target to a concrete index, so the runtime array the CPU
   actually executes is a flat, label-free instruction stream.
------------------------------------------------------------------------ */
function L(name){ return { lbl: name }; }

function assemble(rawProgram){
  const labelMap = {};
  const flat = [];
  rawProgram.forEach(entry => {
    if (entry.lbl !== undefined){ labelMap[entry.lbl] = flat.length; }
    else flat.push(entry);
  });
  return { instructions: flat, labelMap };
}

/* Small helpers so the firmware below reads like real (if simplified)
   embedded C-adjacent assembly rather than a wall of object literals. */
function SET(addr, val){
  return [ { op:'MOV', rd:'R6', imm: val }, { op:'STR', rs:'R6', addr } ];
}
function SETLIGHTS(nsR,nsY,nsG,ewR,ewY,ewG){
  return [
    ...SET(ADDR.NS_RED, nsR), ...SET(ADDR.NS_YELLOW, nsY), ...SET(ADDR.NS_GREEN, nsG),
    ...SET(ADDR.EW_RED, ewR), ...SET(ADDR.EW_YELLOW, ewY), ...SET(ADDR.EW_GREEN, ewG),
  ];
}

/* ---------------------- 3. FIRMWARE -----------------------------------
   Three programs run on the virtual CPU:
     mainProgram   - background loop: reads sensor memory, computes
                     adaptive green durations, applies manual commands.
     timerISR      - fires when the timer peripheral expires. Advances
                     the normal phase state machine, or steps through
                     the emergency sequence when one is active.
     emergencyISR  - fires once, immediately, when the NVIC receives an
                     emergency-vehicle interrupt. Forces the currently
                     green direction to yellow and arms a short timer.
------------------------------------------------------------------------ */

const mainProgramRaw = [
  L('loop'),
  { op:'LDR', rd:'R0', addr: ADDR.MODE },
  { op:'CMP', rs1:'R0', imm: 1 }, { op:'BEQ', label:'adaptive' },
  { op:'CMP', rs1:'R0', imm: 2 }, { op:'BEQ', label:'manual' },

  // --- normal mode: fixed durations ---
  ...SET(ADDR.GREEN_NS_DUR, 10),
  ...SET(ADDR.GREEN_EW_DUR, 10),
  { op:'B', label:'loop' },

  L('adaptive'),
  { op:'LDR', rd:'R0', addr: ADDR.NS_COUNT },
  { op:'LDR', rd:'R1', addr: ADDR.EW_COUNT },
  { op:'ADD', rd:'R2', rs1:'R0', rs2:'R1' },
  { op:'ADD', rd:'R2', rs1:'R2', imm: 1 },
  { op:'MOV', rd:'R3', imm: 20 },
  { op:'MUL', rd:'R4', rs1:'R0', rs2:'R3' },
  { op:'DIV', rd:'R4', rs1:'R4', rs2:'R2' },
  { op:'ADD', rd:'R4', rs1:'R4', imm: 8 },
  { op:'STR', rs:'R4', addr: ADDR.GREEN_NS_DUR },
  { op:'MUL', rd:'R5', rs1:'R1', rs2:'R3' },
  { op:'DIV', rd:'R5', rs1:'R5', rs2:'R2' },
  { op:'ADD', rd:'R5', rs1:'R5', imm: 8 },
  { op:'STR', rs:'R5', addr: ADDR.GREEN_EW_DUR },
  { op:'B', label:'loop' },

  L('manual'),
  { op:'LDR', rd:'R6', addr: ADDR.MANUAL_TRIGGER },
  { op:'CMP', rs1:'R6', imm: 1 }, { op:'BEQ', label:'apply_manual' },
  { op:'B', label:'loop' },

  L('apply_manual'),
  { op:'LDR', rd:'R7', addr: ADDR.MANUAL_CMD },
  { op:'CMP', rs1:'R7', imm: 0 }, { op:'BEQ', label:'manual_ns' },
  ...SETLIGHTS(1,0,0, 0,0,1),
  ...SET(ADDR.PHASE, 3),
  { op:'B', label:'manual_done' },
  L('manual_ns'),
  ...SETLIGHTS(0,0,1, 1,0,0),
  ...SET(ADDR.PHASE, 0),
  L('manual_done'),
  ...SET(ADDR.MANUAL_TRIGGER, 0),
  { op:'B', label:'loop' },
];

const timerISRRaw = [
  L('start'),
  { op:'LDR', rd:'R0', addr: ADDR.EMERGENCY_ACTIVE },
  { op:'CMP', rs1:'R0', imm: 1 }, { op:'BEQ', label:'emg_dispatch' },

  // --- normal phase advance ---
  { op:'LDR', rd:'R1', addr: ADDR.PHASE },
  { op:'ADD', rd:'R1', rs1:'R1', imm: 1 },
  { op:'CMP', rs1:'R1', imm: 6 }, { op:'BLT', label:'phase_store' },
  { op:'MOV', rd:'R1', imm: 0 },
  L('phase_store'),
  { op:'STR', rs:'R1', addr: ADDR.PHASE },
  { op:'CMP', rs1:'R1', imm: 0 }, { op:'BEQ', label:'p0' },
  { op:'CMP', rs1:'R1', imm: 1 }, { op:'BEQ', label:'p1' },
  { op:'CMP', rs1:'R1', imm: 2 }, { op:'BEQ', label:'p2' },
  { op:'CMP', rs1:'R1', imm: 3 }, { op:'BEQ', label:'p3' },
  { op:'CMP', rs1:'R1', imm: 4 }, { op:'BEQ', label:'p4' },
  { op:'B', label:'p5' },

  L('p0'), ...SETLIGHTS(0,0,1, 1,0,0),
    { op:'LDR', rd:'R4', addr: ADDR.GREEN_NS_DUR }, { op:'STR', rs:'R4', addr: ADDR.TIMER_LOAD },
    { op:'B', label:'isr_end' },
  L('p1'), ...SETLIGHTS(0,1,0, 1,0,0), ...SET(ADDR.TIMER_LOAD, 3), { op:'B', label:'isr_end' },
  L('p2'), ...SETLIGHTS(1,0,0, 1,0,0), ...SET(ADDR.TIMER_LOAD, 2), { op:'B', label:'isr_end' },
  L('p3'), ...SETLIGHTS(1,0,0, 0,0,1),
    { op:'LDR', rd:'R4', addr: ADDR.GREEN_EW_DUR }, { op:'STR', rs:'R4', addr: ADDR.TIMER_LOAD },
    { op:'B', label:'isr_end' },
  L('p4'), ...SETLIGHTS(1,0,0, 0,1,0), ...SET(ADDR.TIMER_LOAD, 3), { op:'B', label:'isr_end' },
  L('p5'), ...SETLIGHTS(1,0,0, 1,0,0), ...SET(ADDR.TIMER_LOAD, 2), { op:'B', label:'isr_end' },

  // --- emergency sequencing ---
  L('emg_dispatch'),
  { op:'LDR', rd:'R0', addr: ADDR.EMG_STAGE },
  { op:'CMP', rs1:'R0', imm: 0 }, { op:'BEQ', label:'emg_allred' },
  { op:'CMP', rs1:'R0', imm: 1 }, { op:'BEQ', label:'emg_green' },
  { op:'B', label:'emg_checkclear' },

  L('emg_allred'), ...SETLIGHTS(1,0,0, 1,0,0),
    ...SET(ADDR.EMG_STAGE, 1), ...SET(ADDR.TIMER_LOAD, 2), { op:'B', label:'isr_end' },

  L('emg_green'),
  { op:'LDR', rd:'R2', addr: ADDR.EMERGENCY_DIR },
  { op:'CMP', rs1:'R2', imm: 0 }, { op:'BEQ', label:'emg_green_ns' },
  ...SETLIGHTS(1,0,0, 0,0,1), ...SET(ADDR.EMG_STAGE, 2), ...SET(ADDR.TIMER_LOAD, 6),
  { op:'B', label:'isr_end' },
  L('emg_green_ns'),
  ...SETLIGHTS(0,0,1, 1,0,0), ...SET(ADDR.EMG_STAGE, 2), ...SET(ADDR.TIMER_LOAD, 6),
  { op:'B', label:'isr_end' },

  L('emg_checkclear'),
  { op:'LDR', rd:'R3', addr: ADDR.EMERGENCY_CLEAR },
  { op:'CMP', rs1:'R3', imm: 1 }, { op:'BEQ', label:'emg_finish' },
  ...SET(ADDR.TIMER_LOAD, 2), { op:'B', label:'isr_end' },

  L('emg_finish'),
  ...SET(ADDR.EMERGENCY_ACTIVE, 0), ...SET(ADDR.EMERGENCY_CLEAR, 0),
  { op:'LDR', rd:'R2', addr: ADDR.EMERGENCY_DIR },
  { op:'CMP', rs1:'R2', imm: 0 }, { op:'BEQ', label:'resume_after_ns' },
  ...SET(ADDR.PHASE, 0), { op:'B', label:'emg_allred2' },
  L('resume_after_ns'),
  ...SET(ADDR.PHASE, 3),
  L('emg_allred2'),
  ...SETLIGHTS(1,0,0, 1,0,0), ...SET(ADDR.TIMER_LOAD, 2), { op:'B', label:'isr_end' },

  L('isr_end'),
  { op:'RETI' },
];

const emergencyISRRaw = [
  L('start'),
  { op:'LDR', rd:'R0', addr: ADDR.EMERGENCY_ACTIVE },
  { op:'CMP', rs1:'R0', imm: 1 }, { op:'BEQ', label:'already' },

  ...SET(ADDR.EMERGENCY_ACTIVE, 1),
  ...SET(ADDR.EMG_STAGE, 0),
  { op:'LDR', rd:'R1', addr: ADDR.PHASE },
  { op:'CMP', rs1:'R1', imm: 0 }, { op:'BEQ', label:'from_ns' },
  { op:'CMP', rs1:'R1', imm: 3 }, { op:'BEQ', label:'from_ew' },
  ...SETLIGHTS(1,0,0, 1,0,0), { op:'B', label:'set_timer' },
  L('from_ns'), ...SETLIGHTS(0,1,0, 1,0,0), { op:'B', label:'set_timer' },
  L('from_ew'), ...SETLIGHTS(1,0,0, 0,1,0), { op:'B', label:'set_timer' },
  L('set_timer'), ...SET(ADDR.TIMER_LOAD, 2),
  L('already'),
  { op:'RETI' },
];

const mainAsm = assemble(mainProgramRaw);
const timerAsm = assemble(timerISRRaw);
const emgAsm = assemble(emergencyISRRaw);

/* ---------------------- 4. CORTEX-M3 ENGINE --------------------------- */
class CortexM3 {
  constructor(){
    this.regs = { R0:0,R1:0,R2:0,R3:0,R4:0,R5:0,R6:0,R7:0, R13:0x20001000, R14:0, R15:0 };
    this.flags = { Z:false, N:false };
    this.memory = {};              // sparse address -> value
    Object.values(ADDR).forEach(a => this.memory[a] = 0);
    this.memory[ADDR.EMG_STAGE] = 0;

    this.mainPc = 0;               // background program counter
    this.inISR = false;
    this.isrKind = null;           // 'TIMER' | 'EMERGENCY'
    this.isrPc = 0;

    this.timer = { remaining: 10, period: 10, frozen:false };
    this.nvic = { pendingEmergency:false, pendingTimer:false, firing:null };

    this.cycles = 0;
    this.instructionsExecuted = 0;
    this.lastInstr = null;
    this.lastWrite = null;         // {addr, val} for renderer flash cues
    this.lastRegWrite = null;      // register name just written, for flash cue

    this.listeners = {};
  }

  on(evt, fn){ (this.listeners[evt] ||= []).push(fn); }
  emit(evt, payload){ (this.listeners[evt]||[]).forEach(fn => fn(payload)); }

  readMem(addr){ return this.memory[addr] ?? 0; }

  writeMem(addr, val){
    this.memory[addr] = val;
    this.lastWrite = { addr, val };
    if (GPIO_ADDRS.includes(addr)){
      this.emit('gpio-write', { addr, val, name: GPIO_NAMES[addr] });
    }
    if (addr === ADDR.TIMER_LOAD){
      this.timer.period = val;
      this.timer.remaining = val;
    }
    this.emit('mem-write', { addr, val });
  }

  requestInterrupt(kind){
    if (kind === 'EMERGENCY') this.nvic.pendingEmergency = true;
    if (kind === 'TIMER') this.nvic.pendingTimer = true;
  }

  /* Execute exactly one instruction from the given assembled program,
     operating on either the main PC or the current ISR's PC. Returns
     the instruction object that was executed (or null if RETI just
     unwound the ISR). */
  stepProgram(asm, isISR){
    const pcKey = isISR ? 'isrPc' : 'mainPc';
    let pc = this[pcKey];
    if (pc >= asm.instructions.length) pc = 0;
    const instr = asm.instructions[pc];
    this.emit('fetch', { pc, instr, isISR });

    const rv = (o) => o.imm !== undefined ? o.imm : this.regs[o.rs !== undefined ? o.rs : o.rs2];
    let nextPc = pc + 1;

    switch(instr.op){
      case 'MOV': this.regs[instr.rd] = instr.imm !== undefined ? instr.imm : this.regs[instr.rs]; break;
      case 'ADD': this.regs[instr.rd] = this.regs[instr.rs1] + (instr.imm !== undefined ? instr.imm : this.regs[instr.rs2]); break;
      case 'SUB': this.regs[instr.rd] = this.regs[instr.rs1] - (instr.imm !== undefined ? instr.imm : this.regs[instr.rs2]); break;
      case 'MUL': this.regs[instr.rd] = this.regs[instr.rs1] * (instr.imm !== undefined ? instr.imm : this.regs[instr.rs2]); break;
      case 'DIV': { const d = instr.imm !== undefined ? instr.imm : this.regs[instr.rs2]; this.regs[instr.rd] = d ? Math.floor(this.regs[instr.rs1] / d) : 0; break; }
      case 'AND': this.regs[instr.rd] = this.regs[instr.rs1] & (instr.imm !== undefined ? instr.imm : this.regs[instr.rs2]); break;
      case 'ORR': this.regs[instr.rd] = this.regs[instr.rs1] | (instr.imm !== undefined ? instr.imm : this.regs[instr.rs2]); break;
      case 'EOR': this.regs[instr.rd] = this.regs[instr.rs1] ^ (instr.imm !== undefined ? instr.imm : this.regs[instr.rs2]); break;
      case 'CMP': { const b = instr.imm !== undefined ? instr.imm : this.regs[instr.rs2]; const d = this.regs[instr.rs1] - b; this.flags.Z = d===0; this.flags.N = d<0; break; }
      case 'LDR': this.regs[instr.rd] = this.readMem(instr.addr); break;
      case 'STR': this.writeMem(instr.addr, this.regs[instr.rs]); break;
      case 'B': nextPc = asm.labelMap[instr.label]; break;
      case 'BEQ': if (this.flags.Z) nextPc = asm.labelMap[instr.label]; break;
      case 'BNE': if (!this.flags.Z) nextPc = asm.labelMap[instr.label]; break;
      case 'BGT': if (!this.flags.Z && !this.flags.N) nextPc = asm.labelMap[instr.label]; break;
      case 'BLT': if (this.flags.N) nextPc = asm.labelMap[instr.label]; break;
      case 'RETI': break;
      case 'NOP': break;
    }

    this[pcKey] = nextPc;
    this.regs.R15 = isISR ? this.mainPc : nextPc;
    this.cycles++;
    this.instructionsExecuted++;
    this.lastInstr = instr;
    this.emit('execute', { instr, isISR, regs: {...this.regs} });
    return instr;
  }

  enterISR(kind){
    const asm = kind === 'EMERGENCY' ? emgAsm : timerAsm;
    this.regs.R14 = this.mainPc;       // LR <- return address (cosmetic but real)
    this.regs.R13 -= 4;                // SP moves as if a stack frame was pushed
    this.inISR = true;
    this.isrKind = kind;
    this.isrPc = 0;
    this.emit('interrupt-enter', { kind });
    if (kind === 'EMERGENCY') this.nvic.pendingEmergency = false; else this.nvic.pendingTimer = false;
  }

  exitISR(){
    this.regs.R13 += 4;
    this.inISR = false;
    this.emit('interrupt-exit', { kind: this.isrKind });
    this.isrKind = null;
  }

  /* The timer peripheral is real hardware, independent of how fast the
     CPU happens to be stepping - it counts down in wall-clock time every
     frame regardless of the instruction throttle below. */
  tickTimer(dtSeconds){
    if (this.timer.frozen) return;
    this.timer.remaining -= dtSeconds;
    if (this.timer.remaining <= 0){
      this.requestInterrupt('TIMER');
      if (this.timer.remaining < -1) this.timer.remaining = 0;
    }
  }

  /* Executes exactly one pipeline step: either the next ISR instruction,
     an ISR entry, or the next main-loop instruction. Called at a
     throttled rate from the render loop so a human can follow along in
     the console and the architecture view. */
  stepOnce(){
    if (this.inISR){
      const asm = this.isrKind === 'EMERGENCY' ? emgAsm : timerAsm;
      const instr = this.stepProgram(asm, true);
      if (instr.op === 'RETI') this.exitISR();
    } else if (this.nvic.pendingEmergency) {
      this.enterISR('EMERGENCY');
    } else if (this.nvic.pendingTimer) {
      this.enterISR('TIMER');
    } else {
      this.stepProgram(mainAsm, false);
    }
  }
}

/* ---------------------- 5. TRAFFIC SIMULATION -------------------------
   Pure physical-world simulation. Reads light state ONLY from GPIO
   memory (via cortex.readMem). Writes vehicle counts / emergency flags
   into the CPU's sensor memory region - it never touches GPIO itself.
------------------------------------------------------------------------ */
const LANES = {
  N: { axis:'y', sign:+1, spawn:-40, stopLine:236, exit:378, limit:650 },
  E: { axis:'x', sign:-1, spawn:640, stopLine:364, exit:222, limit:-40 },
};
const VEHICLE_TYPES = {
  car:      { length:26, speed:78, w:16, h:26, color:'#5b8dd9' },
  bike:     { length:14, speed:70, w:9,  h:14, color:'#e0a83c' },
  bus:      { length:42, speed:58, w:18, h:42, color:'#5bd98f' },
  truck:    { length:40, speed:55, w:19, h:40, color:'#c96ae0' },
  ambulance:{ length:30, speed:95, w:17, h:30, color:'#f2f2f2' },
  firetruck:{ length:38, speed:90, w:19, h:38, color:'#ff5a5a' },
};
const NORMAL_TYPES = ['car','car','car','bike','bus','truck'];

let vehicleIdCounter = 1;

class TrafficSim {
  constructor(cortex){
    this.cortex = cortex;
    this.vehicles = [];
    this.spawnAccumulatorN = 0;
    this.spawnAccumulatorE = 0;
    this.spawnRate = 0.45;          // 0..1, UI controlled
    this.pendingEmergencyClearCheck = { N:false, E:false };
  }

  spawn(dir, typeOverride){
    const type = typeOverride || NORMAL_TYPES[Math.floor(Math.random()*NORMAL_TYPES.length)];
    const cfg = LANES[dir];
    const spec = VEHICLE_TYPES[type];
    this.vehicles.push({
      id: vehicleIdCounter++,
      dir, type,
      pos: cfg.spawn,
      speed: spec.speed,
      length: spec.length,
      emergency: type === 'ambulance' || type === 'firetruck',
    });
    if (type === 'ambulance' || type === 'firetruck'){
      this.cortex.writeMem(ADDR.EMERGENCY_DIR, dir === 'N' ? 0 : 1);
      this.cortex.requestInterrupt('EMERGENCY');
    }
  }

  update(dt){
    const cfg_N = LANES.N, cfg_E = LANES.E;
    const nsGreen = this.cortex.readMem(ADDR.NS_GREEN) === 1;
    const ewGreen = this.cortex.readMem(ADDR.EW_GREEN) === 1;

    // background traffic spawning
    this.spawnAccumulatorN += dt * this.spawnRate * 0.8;
    this.spawnAccumulatorE += dt * this.spawnRate * 0.8;
    if (this.spawnAccumulatorN > 1){ this.spawnAccumulatorN = 0; if (Math.random() < 0.7) this.spawn('N'); }
    if (this.spawnAccumulatorE > 1){ this.spawnAccumulatorE = 0; if (Math.random() < 0.7) this.spawn('E'); }

    ['N','E'].forEach(dir => {
      const cfg = LANES[dir];
      const lightGreen = dir === 'N' ? nsGreen : ewGreen;
      // sort front-most first (largest progress along travel direction)
      const list = this.vehicles.filter(v => v.dir === dir)
        .sort((a,b) => cfg.sign>0 ? b.pos - a.pos : a.pos - b.pos);

      list.forEach((v, idx) => {
        const pastStopLine = cfg.sign>0 ? v.pos >= cfg.stopLine : v.pos <= cfg.stopLine;
        let maxPos = cfg.sign>0 ? Infinity : -Infinity;
        if (!pastStopLine && !lightGreen && !v.emergency){
          maxPos = cfg.stopLine;
        }
        if (idx > 0){
          const ahead = list[idx-1];
          const gap = ahead.length + 10;
          const aheadLimit = cfg.sign>0 ? ahead.pos - gap : ahead.pos + gap;
          maxPos = cfg.sign>0 ? Math.min(maxPos, aheadLimit) : Math.max(maxPos, aheadLimit);
        }
        let next = v.pos + cfg.sign * v.speed * dt;
        next = cfg.sign>0 ? Math.min(next, maxPos) : Math.max(next, maxPos);
        v.pos = next;

        // emergency vehicle clearing the intersection -> tell the ISR
        if (v.emergency){
          const cleared = cfg.sign>0 ? v.pos > cfg.exit : v.pos < cfg.exit;
          if (cleared) this.cortex.writeMem(ADDR.EMERGENCY_CLEAR, 1);
        }
      });
    });

    this.vehicles = this.vehicles.filter(v => {
      const cfg = LANES[v.dir];
      return cfg.sign>0 ? v.pos < cfg.limit : v.pos > cfg.limit;
    });

    // publish sensor data for the firmware to read (direct memory write -
    // this models a peripheral/sensor updating a memory-mapped input
    // register; it deliberately bypasses writeMem() since it is not a
    // GPIO output and should not trigger the GPIO-write visual cue)
    const nsQueue = this.vehicles.filter(v => v.dir==='N' && v.pos < LANES.N.stopLine).length;
    const ewQueue = this.vehicles.filter(v => v.dir==='E' && v.pos > LANES.E.stopLine).length;
    this.cortex.memory[ADDR.NS_COUNT] = nsQueue;
    this.cortex.memory[ADDR.EW_COUNT] = ewQueue;
    return { nsQueue, ewQueue };
  }
}

/* ---------------------- 6. RENDER + UI WIRING -------------------------- */
const ADDR_NAME = {};
Object.entries(ADDR).forEach(([name, val]) => { ADDR_NAME[val] = name; });

function fmtInstr(instr){
  if (!instr) return '\u2014';
  const a = (o) => o.imm !== undefined ? ('#' + o.imm) : o.rs2 || o.rs;
  switch(instr.op){
    case 'MOV': return `MOV ${instr.rd}, ${a(instr)}`;
    case 'ADD': return `ADD ${instr.rd}, ${instr.rs1}, ${a(instr)}`;
    case 'SUB': return `SUB ${instr.rd}, ${instr.rs1}, ${a(instr)}`;
    case 'MUL': return `MUL ${instr.rd}, ${instr.rs1}, ${a(instr)}`;
    case 'DIV': return `DIV ${instr.rd}, ${instr.rs1}, ${a(instr)}`;
    case 'AND': return `AND ${instr.rd}, ${instr.rs1}, ${a(instr)}`;
    case 'ORR': return `ORR ${instr.rd}, ${instr.rs1}, ${a(instr)}`;
    case 'EOR': return `EOR ${instr.rd}, ${instr.rs1}, ${a(instr)}`;
    case 'CMP': return `CMP ${instr.rs1}, ${a(instr)}`;
    case 'LDR': return `LDR ${instr.rd}, [${ADDR_NAME[instr.addr] || instr.addr}]`;
    case 'STR': return `STR ${instr.rs}, [${ADDR_NAME[instr.addr] || instr.addr}]`;
    case 'B': return `B ${instr.label}`;
    case 'BEQ': return `BEQ ${instr.label}`;
    case 'BNE': return `BNE ${instr.label}`;
    case 'BGT': return `BGT ${instr.label}`;
    case 'BLT': return `BLT ${instr.label}`;
    case 'RETI': return `RETI`;
    default: return instr.op;
  }
}

const cortex = new CortexM3();
const traffic = new TrafficSim(cortex);

// bootstrap: firmware hasn't run yet, so seed a legal initial GPIO state
cortex.memory[ADDR.NS_GREEN] = 1;
cortex.memory[ADDR.EW_RED] = 1;
cortex.memory[ADDR.GREEN_NS_DUR] = 10;
cortex.memory[ADDR.GREEN_EW_DUR] = 10;
cortex.memory[ADDR.MODE] = 0;

const REG_ORDER = ['R0','R1','R2','R3','R4','R5','R6','R7'];
const regGridEl = document.getElementById('regGrid');
const regCells = {};
REG_ORDER.forEach(name => {
  const cell = document.createElement('div');
  cell.className = 'reg-cell';
  cell.innerHTML = `<span class="reg-name">${name}</span><span class="reg-val mono">0</span>`;
  regGridEl.appendChild(cell);
  regCells[name] = cell;
});

let prevRegs = {...cortex.regs};

function renderConsole(){
  REG_ORDER.forEach(name => {
    const cell = regCells[name];
    const val = cortex.regs[name];
    cell.querySelector('.reg-val').textContent = val;
    if (val !== prevRegs[name]) { cell.classList.add('flash'); setTimeout(()=>cell.classList.remove('flash'), 260); }
  });
  prevRegs = {...cortex.regs};

  document.getElementById('valPC').textContent = '0x' + cortex.regs.R15.toString(16).padStart(4,'0');
  document.getElementById('valSP').textContent = '0x' + cortex.regs.R13.toString(16);
  document.getElementById('valLR').textContent = '0x' + cortex.regs.R14.toString(16).padStart(4,'0');
  document.getElementById('valFlags').textContent = `Z:${cortex.flags.Z?1:0} N:${cortex.flags.N?1:0}`;
  document.getElementById('valInstrCount').textContent = cortex.instructionsExecuted;
  document.getElementById('valThread').textContent = cortex.inISR ? (cortex.isrKind + '_ISR') : 'main';

  document.getElementById('currentInstr').innerHTML = `<span class="instr-op">${fmtInstr(cortex.lastInstr)}</span>`;

  GPIO_ADDRS.forEach(addr => {
    const name = GPIO_NAMES[addr];
    const led = document.getElementById('gpio-' + name);
    const on = cortex.memory[addr] === 1;
    led.className = 'led' + (on ? (' lit-' + (name.includes('RED')?'red':name.includes('YELLOW')?'yellow':'green')) : '');
  });

  const rem = Math.max(0, cortex.timer.remaining);
  document.getElementById('valTimerRem').textContent = rem.toFixed(1) + 's';
  document.getElementById('valTimerLoad').textContent = cortex.timer.period + 's';
  document.getElementById('timerBarFill').style.width = Math.min(100, (rem / Math.max(1,cortex.timer.period)) * 100) + '%';

  document.getElementById('intTimerBadge').classList.toggle('firing', cortex.inISR && cortex.isrKind==='TIMER');
  document.getElementById('intEmgBadge').classList.toggle('firing', (cortex.inISR && cortex.isrKind==='EMERGENCY') || cortex.memory[ADDR.EMERGENCY_ACTIVE]===1);
  document.getElementById('valNvicState').textContent = cortex.inISR ? `servicing ${cortex.isrKind}` : 'idle';

  document.getElementById('cycleCounter').textContent = cortex.cycles;
}

/* ---- traffic scene rendering ---- */
const vehicleLayer = document.getElementById('vehicleLayer');
const vehicleEls = {};

function bulbClass(base){ return `bulb ${base}`; }

function renderScene(queues){
  document.getElementById('nsQueueCount').textContent = queues.nsQueue;
  document.getElementById('ewQueueCount').textContent = queues.ewQueue;

  const setBulb = (id, on, colorClass) => {
    const el = document.getElementById(id);
    el.setAttribute('class', 'bulb' + (on ? ' ' + colorClass : ''));
  };
  setBulb('ns-red', cortex.memory[ADDR.NS_RED]===1, 'on-red');
  setBulb('ns-yellow', cortex.memory[ADDR.NS_YELLOW]===1, 'on-yellow');
  setBulb('ns-green', cortex.memory[ADDR.NS_GREEN]===1, 'on-green');
  setBulb('ew-red', cortex.memory[ADDR.EW_RED]===1, 'on-red');
  setBulb('ew-yellow', cortex.memory[ADDR.EW_YELLOW]===1, 'on-yellow');
  setBulb('ew-green', cortex.memory[ADDR.EW_GREEN]===1, 'on-green');

  const liveIds = new Set();
  traffic.vehicles.forEach(v => {
    liveIds.add(v.id);
    const spec = VEHICLE_TYPES[v.type];
    let el = vehicleEls[v.id];
    if (!el){
      el = document.createElementNS('http://www.w3.org/2000/svg', 'g');
      const body = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
      body.setAttribute('rx', 3);
      body.setAttribute('class', 'vehicle-body');
      body.setAttribute('fill', spec.color);
      el.appendChild(body);
      if (v.emergency){
        const bar = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
        bar.setAttribute('class', 'vehicle-emg');
        bar.setAttribute('fill', '#49c8ff');
        el.appendChild(bar);
      }
      vehicleLayer.appendChild(el);
      vehicleEls[v.id] = el;
    }
    const body = el.querySelector('.vehicle-body');
    const laneCenterN = 275, laneCenterE = 275;
    if (v.dir === 'N'){
      body.setAttribute('x', laneCenterN - spec.w/2);
      body.setAttribute('y', v.pos - spec.length);
      body.setAttribute('width', spec.w);
      body.setAttribute('height', spec.length);
      const bar = el.querySelector('.vehicle-emg');
      if (bar){ bar.setAttribute('x', laneCenterN - spec.w/2 + 2); bar.setAttribute('y', v.pos - spec.length + 3); bar.setAttribute('width', spec.w-4); bar.setAttribute('height', 4); }
    } else {
      body.setAttribute('x', v.pos);
      body.setAttribute('y', laneCenterE - spec.w/2);
      body.setAttribute('width', spec.length);
      body.setAttribute('height', spec.w);
      const bar = el.querySelector('.vehicle-emg');
      if (bar){ bar.setAttribute('x', v.pos + 2); bar.setAttribute('y', laneCenterE - spec.w/2 + 2); bar.setAttribute('width', 4); bar.setAttribute('height', spec.w-4); }
    }
  });
  Object.keys(vehicleEls).forEach(id => {
    if (!liveIds.has(Number(id))){ vehicleEls[id].remove(); delete vehicleEls[id]; }
  });
}

/* ---------------------- 7. INTERNAL ARCHITECTURE VIEW ------------------ */
const SVGNS = 'http://www.w3.org/2000/svg';
function svgEl(tag, attrs){
  const el = document.createElementNS(SVGNS, tag);
  Object.entries(attrs).forEach(([k,v]) => el.setAttribute(k, v));
  return el;
}

const archSvg = document.getElementById('archSvg');
const archBlocks = {};
const archRegCells = {};
const archGpioLeds = {};
let archDot;

function buildArch(){
  const defs = svgEl('defs', {});
  archSvg.appendChild(defs);

  // bus lines
  const busPaths = [
    [95,45, 150,45],       // PC -> memory
    [220,80, 220,120],     // memory -> decoder
    [220,170, 220,215],    // decoder -> ALU
    [290,235, 340,120],    // ALU -> register file (writeback)
    [290,255, 340,265],    // ALU -> GPIO (STR path)
    [340,150, 290,225],    // register file -> ALU (operand read)
  ];
  busPaths.forEach(([x1,y1,x2,y2]) => {
    archSvg.appendChild(svgEl('path', { d:`M${x1},${y1} L${x2},${y2}`, class:'arch-bus' }));
  });

  function block(id, x,y,w,h,label,sub){
    const g = svgEl('g', {});
    const rect = svgEl('rect', { x,y,width:w,height:h, rx:8, class:'arch-block' });
    g.appendChild(rect);
    g.appendChild(svgEl('text', { x:x+10, y:y+18, class:'arch-block-label' })).textContent = label;
    if (sub){ g.appendChild(svgEl('text', { x:x+10, y:y+h-8, class:'arch-block-sub' })).textContent = sub; }
    archSvg.appendChild(g);
    archBlocks[id] = rect;
    return g;
  }

  block('pc', 15,25,80,40,'PC','program ctr');
  block('mem', 150,20,140,60,'INSTR MEMORY','fetch stage');
  block('decoder', 150,120,140,50,'DECODER','opcode/operands');
  block('alu', 150,210,140,60,'ALU','execute stage');
  const regFileG = block('regfile', 340,20,180,160,'REGISTER FILE','');
  const gpioG = block('gpio', 340,210,180,100,'GPIO / MEM MAP','writeback (STR)');

  // mini register grid inside register-file block
  const rNames = ['R0','R1','R2','R3','R4','R5','R6','R7','SP','LR','PC','xPSR'];
  rNames.forEach((name, i) => {
    const col = i % 3, row = Math.floor(i/3);
    const cx = 352 + col*57, cy = 55 + row*30;
    const cell = svgEl('rect', { x:cx, y:cy, width:52, height:24, rx:4, class:'arch-reg-cell' });
    regFileG.appendChild(cell);
    const txt = svgEl('text', { x:cx+6, y:cy+16, class:'arch-reg-text' });
    regFileG.appendChild(txt);
    archRegCells[name] = { cell, txt };
  });

  // gpio leds inside gpio block
  GPIO_ADDRS.forEach((addr, i) => {
    const name = GPIO_NAMES[addr];
    const col = i % 2, row = Math.floor(i/2);
    const cx = 352 + col*88, cy = 250 + row*20;
    const led = svgEl('circle', { cx:cx, cy:cy, r:5, fill:'#2a3442' });
    gpioG.appendChild(led);
    gpioG.appendChild(svgEl('text', { x:cx+10, y:cy+3, class:'arch-block-sub' })).textContent = name;
    archGpioLeds[name] = led;
  });

  archDot = svgEl('circle', { r:5, class:'arch-flow-dot' });
  archSvg.appendChild(archDot);

  archSvg.appendChild(svgEl('text', { x:20, y:100, class:'arch-stage-label' })).textContent = 'fetch';
  archSvg.appendChild(svgEl('text', { x:20, y:250, class:'arch-stage-label' })).textContent = 'execute';
}
buildArch();

function clearArchActive(){
  Object.values(archBlocks).forEach(b => b.classList.remove('arch-active'));
}
function moveDot(x1,y1,x2,y2, delay){
  setTimeout(() => {
    archDot.style.transition = 'none';
    archDot.setAttribute('cx', x1); archDot.setAttribute('cy', y1);
    archDot.style.opacity = 1;
    // force reflow so the transition below actually animates
    archDot.getBoundingClientRect();
    archDot.style.transition = 'cx .32s ease, cy .32s ease';
    archDot.setAttribute('cx', x2); archDot.setAttribute('cy', y2);
  }, delay);
}

function animateArchStages(instr, isISR){
  if (currentTab !== 'arch') return;
  const stepMs = 380;
  clearArchActive();
  archBlocks.pc.classList.add('arch-active');
  moveDot(55,45, 150,45, 0);

  setTimeout(() => { clearArchActive(); archBlocks.mem.classList.add('arch-active'); moveDot(220,50, 220,140, stepMs*0.3); }, stepMs*0.28);
  setTimeout(() => { clearArchActive(); archBlocks.decoder.classList.add('arch-active'); moveDot(220,150, 220,215, stepMs*0.3); }, stepMs*0.56);
  setTimeout(() => {
    clearArchActive();
    archBlocks.alu.classList.add('arch-active');
    const isStore = instr && instr.op === 'STR';
    if (isStore){ archBlocks.gpio.classList.add('arch-active'); moveDot(290,250, 345,265, stepMs*0.3); }
    else { archBlocks.regfile.classList.add('arch-active'); moveDot(290,225, 345,90, stepMs*0.3); }
  }, stepMs*0.82);
  setTimeout(() => { archDot.style.opacity = 0; }, stepMs*1.2);
}

function renderArchPanels(){
  const rNames = ['R0','R1','R2','R3','R4','R5','R6','R7'];
  rNames.forEach(name => { archRegCells[name].txt.textContent = `${name}:${cortex.regs[name]}`; });
  archRegCells['SP'].txt.textContent = 'SP:' + cortex.regs.R13.toString(16);
  archRegCells['LR'].txt.textContent = 'LR:' + cortex.regs.R14.toString(16);
  archRegCells['PC'].txt.textContent = 'PC:' + cortex.regs.R15.toString(16);
  archRegCells['xPSR'].txt.textContent = `Z${cortex.flags.Z?1:0} N${cortex.flags.N?1:0}`;

  GPIO_ADDRS.forEach(addr => {
    const name = GPIO_NAMES[addr];
    const on = cortex.memory[addr] === 1;
    const color = !on ? '#2a3442' : (name.includes('RED') ? '#ff4d5e' : name.includes('YELLOW') ? '#ffb020' : '#00e08f');
    archGpioLeds[name].setAttribute('fill', color);
  });
}

cortex.on('execute', ({instr, isISR}) => animateArchStages(instr, isISR));

/* ---------------------- 8. UI WIRING ------------------------------------ */
let currentMode = 'normal';
let currentTab = 'console';
let running = true;
let archLockstep = true;
let speedMultiplier = 1;

document.querySelectorAll('.mode-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    currentMode = btn.dataset.mode;
    cortex.writeMem(ADDR.MODE, currentMode==='normal'?0 : currentMode==='adaptive'?1 : 2);
    document.getElementById('manualControls').hidden = currentMode !== 'manual';
  });
});

document.getElementById('manualNS').addEventListener('click', () => {
  cortex.writeMem(ADDR.MANUAL_CMD, 0);
  cortex.writeMem(ADDR.MANUAL_TRIGGER, 1);
});
document.getElementById('manualEW').addEventListener('click', () => {
  cortex.writeMem(ADDR.MANUAL_CMD, 1);
  cortex.writeMem(ADDR.MANUAL_TRIGGER, 1);
});

document.getElementById('spawnAmbNS').addEventListener('click', () => traffic.spawn('N', 'ambulance'));
document.getElementById('spawnAmbEW').addEventListener('click', () => traffic.spawn('E', 'firetruck'));

document.getElementById('spawnRateSlider').addEventListener('input', (e) => {
  traffic.spawnRate = Number(e.target.value) / 100;
});

document.querySelectorAll('.tab-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    currentTab = btn.dataset.tab;
    document.getElementById('tabConsole').hidden = currentTab !== 'console';
    document.getElementById('tabArch').hidden = currentTab !== 'arch';
  });
});

const archPlayPauseBtn = document.getElementById('archPlayPause');
const archStepBtn = document.getElementById('archStep');
const archHint = document.getElementById('archHint');
archPlayPauseBtn.addEventListener('click', () => {
  archLockstep = !archLockstep;
  archPlayPauseBtn.innerHTML = archLockstep ? '&#10074;&#10074; Pause lockstep' : '&#9654; Resume lockstep';
  archStepBtn.disabled = archLockstep;
  archHint.textContent = archLockstep
    ? 'Watching the live controller loop in real time.'
    : 'Paused — click "Step once" to execute a single instruction.';
});
archStepBtn.addEventListener('click', () => {
  cortex.stepOnce();
  renderConsole();
  renderArchPanels();
});

const globalPlayPauseBtn = document.getElementById('globalPlayPause');
globalPlayPauseBtn.addEventListener('click', () => {
  running = !running;
  globalPlayPauseBtn.innerHTML = running ? '&#10074;&#10074; Pause' : '&#9654; Resume';
  document.getElementById('cpuStatusText').textContent = running ? 'CPU RUNNING' : 'CPU HALTED';
  document.getElementById('cpuStatusDot').style.background = running ? 'var(--green)' : 'var(--text-faint)';
});

const speedSlider = document.getElementById('speedSlider');
const speedLabel = document.getElementById('speedLabel');
speedSlider.addEventListener('input', (e) => {
  speedMultiplier = Number(e.target.value);
  speedLabel.textContent = speedMultiplier.toFixed(2).replace(/0$/,'').replace(/\.$/,'.0') + '\u00d7';
});

/* ---------------------- 9. MAIN LOOP ------------------------------------ */
const CPU_STEP_INTERVAL = 0.07;
let lastTime = performance.now();
let cpuAccumulator = 0;

function loop(now){
  let dt = (now - lastTime) / 1000;
  lastTime = now;
  dt = Math.min(dt, 0.1) * speedMultiplier;

  if (running){
    const queues = traffic.update(dt);
    cortex.tickTimer(dt);

    const shouldStepCpu = !(currentTab === 'arch' && !archLockstep);
    if (shouldStepCpu){
      cpuAccumulator += dt;
      while (cpuAccumulator >= CPU_STEP_INTERVAL){
        cpuAccumulator -= CPU_STEP_INTERVAL;
        cortex.stepOnce();
      }
    }

    renderConsole();
    renderScene(queues);
    if (currentTab === 'arch') renderArchPanels();
  }

  requestAnimationFrame(loop);
}
requestAnimationFrame(loop);
