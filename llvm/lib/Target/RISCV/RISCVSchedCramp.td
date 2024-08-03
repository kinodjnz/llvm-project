//==- RISCVSchedCramp.td - Cramp Scheduling Definitions ----*- tablegen -*-=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

// ===---------------------------------------------------------------------===//
// The following definitions describe the simpler per-operand machine model.
// This works with MachineScheduler. See MCSchedule.h for details.

// Cramp machine model for scheduling and other instruction cost heuristics.
def CrampModel : SchedMachineModel {
  let MicroOpBufferSize = 0; // Cramp is in-order.
  let IssueWidth = 1;        // 1 micro-op is dispatched per cycle.
  let LoadLatency = 3;       // Cycles for loads to access the cache.
  let MispredictPenalty = 4; // Extra cycles for a mispredicted branch.
  let PostRAScheduler = true;
  let CompleteModel = false;
  let UnsupportedFeatures = [HasStdExtZbkb, HasStdExtZbkc, HasStdExtZbkx,
                             HasStdExtZcmt, HasStdExtZknd, HasStdExtZkne,
                             HasStdExtZknh, HasStdExtZksed, HasStdExtZksh,
                             HasStdExtZkr, HasVInstructions, HasVInstructionsI64,
                             HasStdExtZvbb, HasStdExtZvbc, HasStdExtZvkb, HasStdExtZvkg,
                             HasStdExtZalasr, HasStdExtZicfiss, HasStdExtZcmop,
                             HasStdExtZacas, HasStdExtZabha,
                             HasVendorXTHeadBa, HasVendorXTHeadBb, HasVendorXTHeadCondMov,
                             HasVendorXTHeadMac, HasVendorXTHeadMemPair, HasVendorXTHeadMemIdx,
                             HasVendorXTHeadCmo, HasVendorXTHeadSync, HasVendorXTHeadVdot,
                             HasVendorXSfvcp, HasVendorXSfvqmaccdod, HasVendorXSfvqmaccqoq,
                             HasVendorXSfvfwmaccqqq, HasVendorXSfvfnrclipxfqf,
                             HasVendorXCVsimd, HasVendorXCVbi, HasVendorXCVmem, HasVendorXCVelw,
                             HasVendorXCVbitmanip, HasVendorXCValu, HasVendorXCVmac,
                             IsRV64, HasStdExtF, HasStdExtA];
}

//===----------------------------------------------------------------------===//

let SchedModel = CrampModel in {

def CrampIALU       : ProcResource<1>; // Int ALU
defvar CrampIMulDiv = CrampIALU;       // Int Multiply/Division
defvar CrampMem     = CrampIALU;       // Load/Store
defvar CrampBranch  = CrampIALU;       // Branch

// Branching
let Latency = 2 in {
def : WriteRes<WriteJmp, [CrampBranch]>;
def : WriteRes<WriteJal, [CrampBranch]>;
def : WriteRes<WriteJalr, [CrampBranch]>;
}

// Integer arithmetic and logic
let Latency = 1 in {
def : WriteRes<WriteIALU, [CrampIALU]>;
def : WriteRes<WriteShiftImm, [CrampIALU]>;
def : WriteRes<WriteShiftReg, [CrampIALU]>;
def : WriteRes<WriteIALU32, []>;
def : WriteRes<WriteShiftImm32, []>;
def : WriteRes<WriteShiftReg32, []>;
}

// Integer multiplication
let Latency = 3 in {
def : WriteRes<WriteIMul, [CrampIMulDiv]>;
def : WriteRes<WriteIMul32, []>;
}


// Integer division
// Worst case latency is used.
def : WriteRes<WriteIDiv, [CrampIMulDiv]> {
  let Latency = 24;
  let ReleaseAtCycles = [20];
}
def : WriteRes<WriteIDiv32, []>;

// Integer remainder
def : WriteRes<WriteIRem, [CrampIMulDiv]> {
  let Latency = 24;
  let ReleaseAtCycles = [20];
}
def : WriteRes<WriteIRem32, []>;

// Memory
let Latency = 1 in {
def : WriteRes<WriteSTB, [CrampMem]>;
def : WriteRes<WriteSTH, [CrampMem]>;
def : WriteRes<WriteSTW, [CrampMem]>;
def : WriteRes<WriteSTD, []>;
def : WriteRes<WriteFST16, []>;
def : WriteRes<WriteFST32, []>;
def : WriteRes<WriteFST64, []>;
}

let Latency = 4 in {
def : WriteRes<WriteLDB, [CrampMem]>;
def : WriteRes<WriteLDH, [CrampMem]>;
}

let Latency = 3 in {
def : WriteRes<WriteLDW, [CrampMem]>;
def : WriteRes<WriteLDD, []>;
def : WriteRes<WriteFLD16, []>;
def : WriteRes<WriteFLD32, []>;
def : WriteRes<WriteFLD64, []>;

// Atomic memory
def : WriteRes<WriteAtomicW, []>;
def : WriteRes<WriteAtomicD, []>;

def : WriteRes<WriteAtomicLDW, []>;
def : WriteRes<WriteAtomicLDD, []>;
}

let Latency = 1 in {
def : WriteRes<WriteAtomicSTW, [CrampMem]>;
def : WriteRes<WriteAtomicSTD, []>;
}

// Single precision.
def : WriteRes<WriteFAdd32, []>;
def : WriteRes<WriteFSGNJ32, []>;
def : WriteRes<WriteFMinMax32, []>;

// Double precision
def : WriteRes<WriteFAdd64, []>;
def : WriteRes<WriteFSGNJ64, []>;
def : WriteRes<WriteFMinMax64, []>;

// Conversions
def : WriteRes<WriteFCvtI32ToF32, []>;
def : WriteRes<WriteFCvtI32ToF64, []>;
def : WriteRes<WriteFCvtI64ToF32, []>;
def : WriteRes<WriteFCvtI64ToF64, []>;
def : WriteRes<WriteFCvtF32ToI32, []>;
def : WriteRes<WriteFCvtF32ToI64, []>;
def : WriteRes<WriteFCvtF64ToI32, []>;
def : WriteRes<WriteFCvtF64ToI64, []>;
def : WriteRes<WriteFCvtF32ToF64, []>;
def : WriteRes<WriteFCvtF32ToF16, []>;
def : WriteRes<WriteFCvtF16ToF64, []>;
def : WriteRes<WriteFCvtF64ToF16, []>;
def : WriteRes<WriteFCvtF16ToF32, []>;
def : WriteRes<WriteFCvtF64ToF32, []>;

def : WriteRes<WriteFClass32, []>;
def : WriteRes<WriteFClass64, []>;
def : WriteRes<WriteFCmp32, []>;
def : WriteRes<WriteFCmp64, []>;
def : WriteRes<WriteFMovF32ToI32, []>;
def : WriteRes<WriteFMovI32ToF32, []>;
def : WriteRes<WriteFMovF64ToI64, []>;
def : WriteRes<WriteFMovI64ToF64, []>;
def : WriteRes<WriteFMovI16ToF16, []>;
def : WriteRes<WriteFMovF16ToI16, []>;

// FP multiplication
def : WriteRes<WriteFMul32, []>;
def : WriteRes<WriteFMA32, []>;

def : WriteRes<WriteFMul64, []>;
def : WriteRes<WriteFMA64, []>;

// FP division
def : WriteRes<WriteFDiv32, []>;
def : WriteRes<WriteFDiv64, []>;

// FP square root
def : WriteRes<WriteFSqrt32, []>;
def : WriteRes<WriteFSqrt64, []>;

let Latency = 1 in {
// bitmanip
def : WriteRes<WriteRotateImm, [CrampIALU]>;
def : WriteRes<WriteRotateImm32, []>;
def : WriteRes<WriteRotateReg, [CrampIALU]>;
def : WriteRes<WriteRotateReg32, []>;

def : WriteRes<WriteIMinMax, [CrampIALU]>;

def : WriteRes<WriteSHXADD, [CrampIALU]>;
def : WriteRes<WriteSHXADD32, []>;

def : WriteRes<WriteSingleBit, [CrampIALU]>;
def : WriteRes<WriteSingleBitImm, [CrampIALU]>;
def : WriteRes<WriteBEXT, [CrampIALU]>;
def : WriteRes<WriteBEXTI, [CrampIALU]>;
}

let Latency = 2 in {
// bitmanip
def : WriteRes<WriteCLZ, [CrampIALU]>;
def : WriteRes<WriteCLZ32, []>;
def : WriteRes<WriteCTZ, [CrampIALU]>;
def : WriteRes<WriteCTZ32, []>;

def : WriteRes<WriteORCB, [CrampIALU]>;

def : WriteRes<WriteREV8, [CrampIALU]>;

def : WriteRes<WriteCPOP, [CrampIALU]>;
def : WriteRes<WriteCPOP32, []>;
}

let Latency = 1 in {
// XCramp
def : WriteRes<WriteCMov, [CrampIALU]>;
def : WriteRes<WriteFSReg, [CrampIALU]>;
def : WriteRes<WriteFSRImm, [CrampIALU]>;
}

let Latency = 2 in {
// XCramp
def : WriteRes<WriteBscthReg, [CrampIALU]>;
def : WriteRes<WriteBFReg, [CrampIALU]>;
def : WriteRes<WriteBFImm, [CrampIALU]>;
def : WriteRes<WriteBFRLen, [CrampIALU]>;
}

// Others
def : WriteRes<WriteCSR, []>;
def : WriteRes<WriteNop, []>;

def : InstRW<[WriteIALU], (instrs COPY)>;

//===----------------------------------------------------------------------===//
// Bypass and advance
def : ReadAdvance<ReadJmp, 0>;
def : ReadAdvance<ReadJalr, 0>;
def : ReadAdvance<ReadCSR, 0>;
def : ReadAdvance<ReadStoreData, 0>;
def : ReadAdvance<ReadMemBase, 0>;
def : ReadAdvance<ReadIALU, 0>;
def : ReadAdvance<ReadIALU32, 0>;
def : ReadAdvance<ReadShiftImm, 0>;
def : ReadAdvance<ReadShiftImm32, 0>;
def : ReadAdvance<ReadShiftReg, 0>;
def : ReadAdvance<ReadShiftReg32, 0>;
def : ReadAdvance<ReadIDiv, 0>;
def : ReadAdvance<ReadIDiv32, 0>;
def : ReadAdvance<ReadIRem, 0>;
def : ReadAdvance<ReadIRem32, 0>;
def : ReadAdvance<ReadIMul, 0>;
def : ReadAdvance<ReadIMul32, 0>;
def : ReadAdvance<ReadAtomicWA, 0>;
def : ReadAdvance<ReadAtomicWD, 0>;
def : ReadAdvance<ReadAtomicDA, 0>;
def : ReadAdvance<ReadAtomicDD, 0>;
def : ReadAdvance<ReadAtomicLDW, 0>;
def : ReadAdvance<ReadAtomicLDD, 0>;
def : ReadAdvance<ReadAtomicSTW, 0>;
def : ReadAdvance<ReadAtomicSTD, 0>;
def : ReadAdvance<ReadFStoreData, 0>;
def : ReadAdvance<ReadFMemBase, 0>;
def : ReadAdvance<ReadFAdd32, 0>;
def : ReadAdvance<ReadFAdd64, 0>;
def : ReadAdvance<ReadFMul32, 0>;
def : ReadAdvance<ReadFMul64, 0>;
def : ReadAdvance<ReadFMA32, 0>;
def : ReadAdvance<ReadFMA32Addend, 0>;
def : ReadAdvance<ReadFMA64, 0>;
def : ReadAdvance<ReadFMA64Addend, 0>;
def : ReadAdvance<ReadFDiv32, 0>;
def : ReadAdvance<ReadFDiv64, 0>;
def : ReadAdvance<ReadFSqrt32, 0>;
def : ReadAdvance<ReadFSqrt64, 0>;
def : ReadAdvance<ReadFCmp32, 0>;
def : ReadAdvance<ReadFCmp64, 0>;
def : ReadAdvance<ReadFSGNJ32, 0>;
def : ReadAdvance<ReadFSGNJ64, 0>;
def : ReadAdvance<ReadFMinMax32, 0>;
def : ReadAdvance<ReadFMinMax64, 0>;
def : ReadAdvance<ReadFCvtF32ToI32, 0>;
def : ReadAdvance<ReadFCvtF32ToI64, 0>;
def : ReadAdvance<ReadFCvtF64ToI32, 0>;
def : ReadAdvance<ReadFCvtF64ToI64, 0>;
def : ReadAdvance<ReadFCvtI32ToF32, 0>;
def : ReadAdvance<ReadFCvtI32ToF64, 0>;
def : ReadAdvance<ReadFCvtI64ToF32, 0>;
def : ReadAdvance<ReadFCvtI64ToF64, 0>;
def : ReadAdvance<ReadFCvtF32ToF64, 0>;
def : ReadAdvance<ReadFCvtF32ToF16, 0>;
def : ReadAdvance<ReadFCvtF16ToF64, 0>;
def : ReadAdvance<ReadFCvtF64ToF16, 0>;
def : ReadAdvance<ReadFCvtF16ToF32, 0>;
def : ReadAdvance<ReadFCvtF64ToF32, 0>;
def : ReadAdvance<ReadFMovF32ToI32, 0>;
def : ReadAdvance<ReadFMovI32ToF32, 0>;
def : ReadAdvance<ReadFMovF64ToI64, 0>;
def : ReadAdvance<ReadFMovI64ToF64, 0>;
def : ReadAdvance<ReadFMovI16ToF16, 0>;
def : ReadAdvance<ReadFMovF16ToI16, 0>;
def : ReadAdvance<ReadFClass32, 0>;
def : ReadAdvance<ReadFClass64, 0>;
def : ReadAdvance<ReadRotateImm, 0>;
def : ReadAdvance<ReadRotateImm32, 0>;
def : ReadAdvance<ReadRotateReg, 0>;
def : ReadAdvance<ReadRotateReg32, 0>;
def : ReadAdvance<ReadCLZ, 0>;
def : ReadAdvance<ReadCLZ32, 0>;
def : ReadAdvance<ReadCTZ, 0>;
def : ReadAdvance<ReadCTZ32, 0>;
def : ReadAdvance<ReadCPOP, 0>;
def : ReadAdvance<ReadCPOP32, 0>;
def : ReadAdvance<ReadORCB, 0>;
def : ReadAdvance<ReadIMinMax, 0>;
def : ReadAdvance<ReadREV8, 0>;
def : ReadAdvance<ReadSHXADD, 0>;
def : ReadAdvance<ReadSHXADD32, 0>;
def : ReadAdvance<ReadSingleBit, 0>;
def : ReadAdvance<ReadSingleBitImm, 0>;
def : ReadAdvance<ReadCMov, 0>;
def : ReadAdvance<ReadFSReg, 0>;
def : ReadAdvance<ReadFSRImm, 0>;
def : ReadAdvance<ReadBscthReg, 0>;
def : ReadAdvance<ReadBFReg, 0>;
def : ReadAdvance<ReadBFImm, 0>;
def : ReadAdvance<ReadBFRLen, 0>;

//===----------------------------------------------------------------------===//
// Unsupported extensions
defm : UnsupportedSchedV;
defm : UnsupportedSchedZabha;
defm : UnsupportedSchedZbc;
defm : UnsupportedSchedZbkb;
defm : UnsupportedSchedZbkx;
defm : UnsupportedSchedZfa;
defm : UnsupportedSchedZfh;
defm : UnsupportedSchedSFB;
defm : UnsupportedSchedXsfvcp;
defm : UnsupportedSchedZvk;
}
