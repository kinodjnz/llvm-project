//===-- Cramp.h - Top-level interface for Cramp -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// RISC-V back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Cramp_Cramp_H
#define LLVM_LIB_TARGET_Cramp_Cramp_H

#include "MCTargetDesc/CrampBaseInfo.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
class CrampRegisterBankInfo;
class CrampSubtarget;
class CrampTargetMachine;
class AsmPrinter;
class FunctionPass;
class InstructionSelector;
class MCInst;
class MCOperand;
class MachineInstr;
class MachineOperand;
class PassRegistry;

FunctionPass *createCrampCodeGenPreparePass();
void initializeCrampCodeGenPreparePass(PassRegistry &);

bool lowerCrampMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                    AsmPrinter &AP);
bool lowerCrampMachineOperandToMCOperand(const MachineOperand &MO,
                                         MCOperand &MCOp, const AsmPrinter &AP);

FunctionPass *createCrampISelDag(CrampTargetMachine &TM,
                                 CodeGenOpt::Level OptLevel);

FunctionPass *createCrampMakeCompressibleOptPass();
void initializeCrampMakeCompressibleOptPass(PassRegistry &);

FunctionPass *createCrampGatherScatterLoweringPass();
void initializeCrampGatherScatterLoweringPass(PassRegistry &);

FunctionPass *createCrampSExtWRemovalPass();
void initializeCrampSExtWRemovalPass(PassRegistry &);

FunctionPass *createCrampMergeBaseOffsetOptPass();
void initializeCrampMergeBaseOffsetOptPass(PassRegistry &);

FunctionPass *createCrampExpandPseudoPass();
void initializeCrampExpandPseudoPass(PassRegistry &);

FunctionPass *createCrampExpandAtomicPseudoPass();
void initializeCrampExpandAtomicPseudoPass(PassRegistry &);

FunctionPass *createCrampInsertVSETVLIPass();
void initializeCrampInsertVSETVLIPass(PassRegistry &);

FunctionPass *createCrampRedundantCopyEliminationPass();
void initializeCrampRedundantCopyEliminationPass(PassRegistry &);

InstructionSelector *createCrampInstructionSelector(const CrampTargetMachine &,
                                                    CrampSubtarget &,
                                                    CrampRegisterBankInfo &);
}

#endif
