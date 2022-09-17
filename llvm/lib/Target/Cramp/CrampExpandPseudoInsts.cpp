//===-- CrampExpandPseudoInsts.cpp - Expand pseudo instructions -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that expands pseudo instructions into target
// instructions. This pass should be run after register allocation but before
// the post-regalloc scheduling pass.
//
//===----------------------------------------------------------------------===//

#include "Cramp.h"
#include "CrampInstrInfo.h"
#include "CrampTargetMachine.h"

#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"

using namespace llvm;

#define Cramp_EXPAND_PSEUDO_NAME "Cramp pseudo instruction expansion pass"

namespace {

class CrampExpandPseudo : public MachineFunctionPass {
public:
  const CrampInstrInfo *TII;
  static char ID;

  CrampExpandPseudo() : MachineFunctionPass(ID) {
    initializeCrampExpandPseudoPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return Cramp_EXPAND_PSEUDO_NAME; }

private:
  bool expandMBB(MachineBasicBlock &MBB);
  bool expandMI(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                MachineBasicBlock::iterator &NextMBBI);
  bool expandAuipcInstPair(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MBBI,
                           MachineBasicBlock::iterator &NextMBBI,
                           unsigned FlagsHi, unsigned SecondOpcode);
  bool expandLoadLocalAddress(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MBBI,
                              MachineBasicBlock::iterator &NextMBBI);
  bool expandLoadAddress(MachineBasicBlock &MBB,
                         MachineBasicBlock::iterator MBBI,
                         MachineBasicBlock::iterator &NextMBBI);
  bool expandLoadTLSIEAddress(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MBBI,
                              MachineBasicBlock::iterator &NextMBBI);
  bool expandLoadTLSGDAddress(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MBBI,
                              MachineBasicBlock::iterator &NextMBBI);
  bool expandVSetVL(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI);
  bool expandVMSET_VMCLR(MachineBasicBlock &MBB,
                         MachineBasicBlock::iterator MBBI, unsigned Opcode);
  bool expandVSPILL(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI);
  bool expandVRELOAD(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI);
};

char CrampExpandPseudo::ID = 0;

bool CrampExpandPseudo::runOnMachineFunction(MachineFunction &MF) {
  TII = static_cast<const CrampInstrInfo *>(MF.getSubtarget().getInstrInfo());
  bool Modified = false;
  for (auto &MBB : MF)
    Modified |= expandMBB(MBB);
  return Modified;
}

bool CrampExpandPseudo::expandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  MachineBasicBlock::iterator MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    MachineBasicBlock::iterator NMBBI = std::next(MBBI);
    Modified |= expandMI(MBB, MBBI, NMBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool CrampExpandPseudo::expandMI(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MBBI,
                                 MachineBasicBlock::iterator &NextMBBI) {
  // CrampInstrInfo::getInstSizeInBytes expects that the total size of the
  // expanded instructions for each pseudo is correct in the Size field of the
  // tablegen definition for the pseudo.
  switch (MBBI->getOpcode()) {
  case Cramp::PseudoLLA:
    return expandLoadLocalAddress(MBB, MBBI, NextMBBI);
  case Cramp::PseudoLA:
    return expandLoadAddress(MBB, MBBI, NextMBBI);
  case Cramp::PseudoLA_TLS_IE:
    return expandLoadTLSIEAddress(MBB, MBBI, NextMBBI);
  case Cramp::PseudoLA_TLS_GD:
    return expandLoadTLSGDAddress(MBB, MBBI, NextMBBI);
  case Cramp::PseudoVSETVLI:
  case Cramp::PseudoVSETVLIX0:
  case Cramp::PseudoVSETIVLI:
    return expandVSetVL(MBB, MBBI);
  case Cramp::PseudoVMCLR_M_B1:
  case Cramp::PseudoVMCLR_M_B2:
  case Cramp::PseudoVMCLR_M_B4:
  case Cramp::PseudoVMCLR_M_B8:
  case Cramp::PseudoVMCLR_M_B16:
  case Cramp::PseudoVMCLR_M_B32:
  case Cramp::PseudoVMCLR_M_B64:
    // vmclr.m vd => vmxor.mm vd, vd, vd
    return expandVMSET_VMCLR(MBB, MBBI, Cramp::VMXOR_MM);
  case Cramp::PseudoVMSET_M_B1:
  case Cramp::PseudoVMSET_M_B2:
  case Cramp::PseudoVMSET_M_B4:
  case Cramp::PseudoVMSET_M_B8:
  case Cramp::PseudoVMSET_M_B16:
  case Cramp::PseudoVMSET_M_B32:
  case Cramp::PseudoVMSET_M_B64:
    // vmset.m vd => vmxnor.mm vd, vd, vd
    return expandVMSET_VMCLR(MBB, MBBI, Cramp::VMXNOR_MM);
  case Cramp::PseudoVSPILL2_M1:
  case Cramp::PseudoVSPILL2_M2:
  case Cramp::PseudoVSPILL2_M4:
  case Cramp::PseudoVSPILL3_M1:
  case Cramp::PseudoVSPILL3_M2:
  case Cramp::PseudoVSPILL4_M1:
  case Cramp::PseudoVSPILL4_M2:
  case Cramp::PseudoVSPILL5_M1:
  case Cramp::PseudoVSPILL6_M1:
  case Cramp::PseudoVSPILL7_M1:
  case Cramp::PseudoVSPILL8_M1:
    return expandVSPILL(MBB, MBBI);
  case Cramp::PseudoVRELOAD2_M1:
  case Cramp::PseudoVRELOAD2_M2:
  case Cramp::PseudoVRELOAD2_M4:
  case Cramp::PseudoVRELOAD3_M1:
  case Cramp::PseudoVRELOAD3_M2:
  case Cramp::PseudoVRELOAD4_M1:
  case Cramp::PseudoVRELOAD4_M2:
  case Cramp::PseudoVRELOAD5_M1:
  case Cramp::PseudoVRELOAD6_M1:
  case Cramp::PseudoVRELOAD7_M1:
  case Cramp::PseudoVRELOAD8_M1:
    return expandVRELOAD(MBB, MBBI);
  }

  return false;
}

bool CrampExpandPseudo::expandAuipcInstPair(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
    MachineBasicBlock::iterator &NextMBBI, unsigned FlagsHi,
    unsigned SecondOpcode) {
  MachineFunction *MF = MBB.getParent();
  MachineInstr &MI = *MBBI;
  DebugLoc DL = MI.getDebugLoc();

  Register DestReg = MI.getOperand(0).getReg();
  const MachineOperand &Symbol = MI.getOperand(1);

  MachineBasicBlock *NewMBB = MF->CreateMachineBasicBlock(MBB.getBasicBlock());

  // Tell AsmPrinter that we unconditionally want the symbol of this label to be
  // emitted.
  NewMBB->setLabelMustBeEmitted();

  MF->insert(++MBB.getIterator(), NewMBB);

  BuildMI(NewMBB, DL, TII->get(Cramp::AUIPC), DestReg)
      .addDisp(Symbol, 0, FlagsHi);
  BuildMI(NewMBB, DL, TII->get(SecondOpcode), DestReg)
      .addReg(DestReg)
      .addMBB(NewMBB, CrampII::MO_PCREL_LO);

  // Move all the rest of the instructions to NewMBB.
  NewMBB->splice(NewMBB->end(), &MBB, std::next(MBBI), MBB.end());
  // Update machine-CFG edges.
  NewMBB->transferSuccessorsAndUpdatePHIs(&MBB);
  // Make the original basic block fall-through to the new.
  MBB.addSuccessor(NewMBB);

  // Make sure live-ins are correctly attached to this new basic block.
  LivePhysRegs LiveRegs;
  computeAndAddLiveIns(LiveRegs, *NewMBB);

  NextMBBI = MBB.end();
  MI.eraseFromParent();
  return true;
}

bool CrampExpandPseudo::expandLoadLocalAddress(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
    MachineBasicBlock::iterator &NextMBBI) {
  return expandAuipcInstPair(MBB, MBBI, NextMBBI, CrampII::MO_PCREL_HI,
                             Cramp::ADDI);
}

bool CrampExpandPseudo::expandLoadAddress(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
    MachineBasicBlock::iterator &NextMBBI) {
  MachineFunction *MF = MBB.getParent();

  unsigned SecondOpcode;
  unsigned FlagsHi;
  if (MF->getTarget().isPositionIndependent()) {
    const auto &STI = MF->getSubtarget<CrampSubtarget>();
    SecondOpcode = STI.is64Bit() ? Cramp::LD : Cramp::LW;
    FlagsHi = CrampII::MO_GOT_HI;
  } else {
    SecondOpcode = Cramp::ADDI;
    FlagsHi = CrampII::MO_PCREL_HI;
  }
  return expandAuipcInstPair(MBB, MBBI, NextMBBI, FlagsHi, SecondOpcode);
}

bool CrampExpandPseudo::expandLoadTLSIEAddress(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
    MachineBasicBlock::iterator &NextMBBI) {
  MachineFunction *MF = MBB.getParent();

  const auto &STI = MF->getSubtarget<CrampSubtarget>();
  unsigned SecondOpcode = STI.is64Bit() ? Cramp::LD : Cramp::LW;
  return expandAuipcInstPair(MBB, MBBI, NextMBBI, CrampII::MO_TLS_GOT_HI,
                             SecondOpcode);
}

bool CrampExpandPseudo::expandLoadTLSGDAddress(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
    MachineBasicBlock::iterator &NextMBBI) {
  return expandAuipcInstPair(MBB, MBBI, NextMBBI, CrampII::MO_TLS_GD_HI,
                             Cramp::ADDI);
}

bool CrampExpandPseudo::expandVSetVL(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator MBBI) {
  assert(MBBI->getNumExplicitOperands() == 3 && MBBI->getNumOperands() >= 5 &&
         "Unexpected instruction format");

  DebugLoc DL = MBBI->getDebugLoc();

  assert((MBBI->getOpcode() == Cramp::PseudoVSETVLI ||
          MBBI->getOpcode() == Cramp::PseudoVSETVLIX0 ||
          MBBI->getOpcode() == Cramp::PseudoVSETIVLI) &&
         "Unexpected pseudo instruction");
  unsigned Opcode;
  if (MBBI->getOpcode() == Cramp::PseudoVSETIVLI)
    Opcode = Cramp::VSETIVLI;
  else
    Opcode = Cramp::VSETVLI;
  const MCInstrDesc &Desc = TII->get(Opcode);
  assert(Desc.getNumOperands() == 3 && "Unexpected instruction format");

  Register DstReg = MBBI->getOperand(0).getReg();
  bool DstIsDead = MBBI->getOperand(0).isDead();
  BuildMI(MBB, MBBI, DL, Desc)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .add(MBBI->getOperand(1))  // VL
      .add(MBBI->getOperand(2)); // VType

  MBBI->eraseFromParent(); // The pseudo instruction is gone now.
  return true;
}

bool CrampExpandPseudo::expandVMSET_VMCLR(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator MBBI,
                                          unsigned Opcode) {
  DebugLoc DL = MBBI->getDebugLoc();
  Register DstReg = MBBI->getOperand(0).getReg();
  const MCInstrDesc &Desc = TII->get(Opcode);
  BuildMI(MBB, MBBI, DL, Desc, DstReg)
      .addReg(DstReg, RegState::Undef)
      .addReg(DstReg, RegState::Undef);
  MBBI->eraseFromParent(); // The pseudo instruction is gone now.
  return true;
}

bool CrampExpandPseudo::expandVSPILL(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator MBBI) {
  const TargetRegisterInfo *TRI =
      MBB.getParent()->getSubtarget().getRegisterInfo();
  DebugLoc DL = MBBI->getDebugLoc();
  Register SrcReg = MBBI->getOperand(0).getReg();
  Register Base = MBBI->getOperand(1).getReg();
  Register VL = MBBI->getOperand(2).getReg();
  auto ZvlssegInfo = Cramp::isRVVSpillForZvlsseg(MBBI->getOpcode());
  if (!ZvlssegInfo)
    return false;
  unsigned NF = ZvlssegInfo->first;
  unsigned LMUL = ZvlssegInfo->second;
  assert(NF * LMUL <= 8 && "Invalid NF/LMUL combinations.");
  unsigned Opcode = Cramp::VS1R_V;
  unsigned SubRegIdx = Cramp::sub_vrm1_0;
  static_assert(Cramp::sub_vrm1_7 == Cramp::sub_vrm1_0 + 7,
                "Unexpected subreg numbering");
  if (LMUL == 2) {
    Opcode = Cramp::VS2R_V;
    SubRegIdx = Cramp::sub_vrm2_0;
    static_assert(Cramp::sub_vrm2_3 == Cramp::sub_vrm2_0 + 3,
                  "Unexpected subreg numbering");
  } else if (LMUL == 4) {
    Opcode = Cramp::VS4R_V;
    SubRegIdx = Cramp::sub_vrm4_0;
    static_assert(Cramp::sub_vrm4_1 == Cramp::sub_vrm4_0 + 1,
                  "Unexpected subreg numbering");
  } else
    assert(LMUL == 1 && "LMUL must be 1, 2, or 4.");

  for (unsigned I = 0; I < NF; ++I) {
    // Adding implicit-use of super register to describe we are using part of
    // super register, that prevents machine verifier complaining when part of
    // subreg is undef, see comment in MachineVerifier::checkLiveness for more
    // detail.
    BuildMI(MBB, MBBI, DL, TII->get(Opcode))
        .addReg(TRI->getSubReg(SrcReg, SubRegIdx + I))
        .addReg(Base)
        .addMemOperand(*(MBBI->memoperands_begin()))
        .addReg(SrcReg, RegState::Implicit);
    if (I != NF - 1)
      BuildMI(MBB, MBBI, DL, TII->get(Cramp::ADD), Base)
          .addReg(Base)
          .addReg(VL);
  }
  MBBI->eraseFromParent();
  return true;
}

bool CrampExpandPseudo::expandVRELOAD(MachineBasicBlock &MBB,
                                      MachineBasicBlock::iterator MBBI) {
  const TargetRegisterInfo *TRI =
      MBB.getParent()->getSubtarget().getRegisterInfo();
  DebugLoc DL = MBBI->getDebugLoc();
  Register DestReg = MBBI->getOperand(0).getReg();
  Register Base = MBBI->getOperand(1).getReg();
  Register VL = MBBI->getOperand(2).getReg();
  auto ZvlssegInfo = Cramp::isRVVSpillForZvlsseg(MBBI->getOpcode());
  if (!ZvlssegInfo)
    return false;
  unsigned NF = ZvlssegInfo->first;
  unsigned LMUL = ZvlssegInfo->second;
  assert(NF * LMUL <= 8 && "Invalid NF/LMUL combinations.");
  unsigned Opcode = Cramp::VL1RE8_V;
  unsigned SubRegIdx = Cramp::sub_vrm1_0;
  static_assert(Cramp::sub_vrm1_7 == Cramp::sub_vrm1_0 + 7,
                "Unexpected subreg numbering");
  if (LMUL == 2) {
    Opcode = Cramp::VL2RE8_V;
    SubRegIdx = Cramp::sub_vrm2_0;
    static_assert(Cramp::sub_vrm2_3 == Cramp::sub_vrm2_0 + 3,
                  "Unexpected subreg numbering");
  } else if (LMUL == 4) {
    Opcode = Cramp::VL4RE8_V;
    SubRegIdx = Cramp::sub_vrm4_0;
    static_assert(Cramp::sub_vrm4_1 == Cramp::sub_vrm4_0 + 1,
                  "Unexpected subreg numbering");
  } else
    assert(LMUL == 1 && "LMUL must be 1, 2, or 4.");

  for (unsigned I = 0; I < NF; ++I) {
    BuildMI(MBB, MBBI, DL, TII->get(Opcode),
            TRI->getSubReg(DestReg, SubRegIdx + I))
        .addReg(Base)
        .addMemOperand(*(MBBI->memoperands_begin()));
    if (I != NF - 1)
      BuildMI(MBB, MBBI, DL, TII->get(Cramp::ADD), Base)
          .addReg(Base)
          .addReg(VL);
  }
  MBBI->eraseFromParent();
  return true;
}

} // end of anonymous namespace

INITIALIZE_PASS(CrampExpandPseudo, "cramp-expand-pseudo",
                Cramp_EXPAND_PSEUDO_NAME, false, false)
namespace llvm {

FunctionPass *createCrampExpandPseudoPass() { return new CrampExpandPseudo(); }

} // end of namespace llvm
