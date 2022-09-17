//===- CrampMacroFusion.cpp - Cramp Macro Fusion --------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file This file contains the Cramp implementation of the DAG scheduling
/// mutation to pair instructions back to back.
//
//===----------------------------------------------------------------------===//
//
#include "CrampMacroFusion.h"
#include "CrampSubtarget.h"
#include "llvm/CodeGen/MacroFusion.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

using namespace llvm;

// Fuse LUI followed by ADDI or ADDIW.
// rd = imm[31:0] which decomposes to
// lui rd, imm[31:12]
// addi(w) rd, rd, imm[11:0]
static bool isLUIADDI(const MachineInstr *FirstMI,
                      const MachineInstr &SecondMI) {
  if (SecondMI.getOpcode() != Cramp::ADDI &&
      SecondMI.getOpcode() != Cramp::ADDIW)
    return false;

  // Assume the 1st instr to be a wildcard if it is unspecified.
  if (!FirstMI)
    return true;

  if (FirstMI->getOpcode() != Cramp::LUI)
    return false;

  // The first operand of ADDI might be a frame index.
  if (!SecondMI.getOperand(1).isReg())
    return false;

  Register FirstDest = FirstMI->getOperand(0).getReg();

  // Destination of LUI should be the ADDI(W) source register.
  if (SecondMI.getOperand(1).getReg() != FirstDest)
    return false;

  // If the FirstMI destination is non-virtual, it should match the SecondMI
  // destination.
  return FirstDest.isVirtual() || SecondMI.getOperand(0).getReg() == FirstDest;
}

static bool shouldScheduleAdjacent(const TargetInstrInfo &TII,
                                   const TargetSubtargetInfo &TSI,
                                   const MachineInstr *FirstMI,
                                   const MachineInstr &SecondMI) {
  const CrampSubtarget &ST = static_cast<const CrampSubtarget &>(TSI);

  if (ST.hasLUIADDIFusion() && isLUIADDI(FirstMI, SecondMI))
    return true;

  return false;
}

std::unique_ptr<ScheduleDAGMutation> llvm::createCrampMacroFusionDAGMutation() {
  return createMacroFusionDAGMutation(shouldScheduleAdjacent);
}
