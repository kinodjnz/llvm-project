//===-------------- CrampSExtWRemoval.cpp - MI sext.w Removal -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
//
// This pass removes unneeded sext.w instructions at the MI level.
//
//===---------------------------------------------------------------------===//

#include "Cramp.h"
#include "CrampSubtarget.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

using namespace llvm;

#define DEBUG_TYPE "cramp-sextw-removal"

STATISTIC(NumRemovedSExtW, "Number of removed sign-extensions");
STATISTIC(NumTransformedToWInstrs,
          "Number of instructions transformed to W-ops");

static cl::opt<bool> DisableSExtWRemoval("cramp-disable-sextw-removal",
                                         cl::desc("Disable removal of sext.w"),
                                         cl::init(false), cl::Hidden);
namespace {

class CrampSExtWRemoval : public MachineFunctionPass {
public:
  static char ID;

  CrampSExtWRemoval() : MachineFunctionPass(ID) {
    initializeCrampSExtWRemovalPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  StringRef getPassName() const override { return "Cramp sext.w Removal"; }
};

} // end anonymous namespace

char CrampSExtWRemoval::ID = 0;
INITIALIZE_PASS(CrampSExtWRemoval, DEBUG_TYPE, "Cramp sext.w Removal", false,
                false)

FunctionPass *llvm::createCrampSExtWRemovalPass() {
  return new CrampSExtWRemoval();
}

// add uses of MI to the Worklist
static void addUses(const MachineInstr &MI,
                    SmallVectorImpl<const MachineInstr *> &Worklist,
                    MachineRegisterInfo &MRI) {
  for (auto &UserOp : MRI.reg_operands(MI.getOperand(0).getReg())) {
    const auto *User = UserOp.getParent();
    if (User == &MI) // ignore the def, current MI
      continue;
    Worklist.push_back(User);
  }
}

// returns true if all uses of OrigMI only depend on the lower word of its
// output, so we can transform OrigMI to the corresponding W-version.
// TODO: handle multiple interdependent transformations
static bool isAllUsesReadW(const MachineInstr &OrigMI,
                           MachineRegisterInfo &MRI) {

  SmallPtrSet<const MachineInstr *, 4> Visited;
  SmallVector<const MachineInstr *, 4> Worklist;

  Visited.insert(&OrigMI);
  addUses(OrigMI, Worklist, MRI);

  while (!Worklist.empty()) {
    const MachineInstr *MI = Worklist.pop_back_val();

    if (!Visited.insert(MI).second) {
      // If we've looped back to OrigMI through a PHI cycle, we can't transform
      // LD or LWU, because these operations use all 64 bits of input.
      if (MI == &OrigMI) {
        unsigned opcode = MI->getOpcode();
        if (opcode == Cramp::LD || opcode == Cramp::LWU)
          return false;
      }
      continue;
    }

    switch (MI->getOpcode()) {
    case Cramp::ADDIW:
    case Cramp::ADDW:
    case Cramp::DIVUW:
    case Cramp::DIVW:
    case Cramp::MULW:
    case Cramp::REMUW:
    case Cramp::REMW:
    case Cramp::SLLIW:
    case Cramp::SLLW:
    case Cramp::SRAIW:
    case Cramp::SRAW:
    case Cramp::SRLIW:
    case Cramp::SRLW:
    case Cramp::SUBW:
    case Cramp::ROLW:
    case Cramp::RORW:
    case Cramp::RORIW:
    case Cramp::CLZW:
    case Cramp::CTZW:
    case Cramp::CPOPW:
    case Cramp::SLLI_UW:
    case Cramp::FCVT_S_W:
    case Cramp::FCVT_S_WU:
    case Cramp::FCVT_D_W:
    case Cramp::FCVT_D_WU:
      continue;

    // these overwrite higher input bits, otherwise the lower word of output
    // depends only on the lower word of input. So check their uses read W.
    case Cramp::SLLI:
      if (MI->getOperand(2).getImm() >= 32)
        continue;
      addUses(*MI, Worklist, MRI);
      continue;
    case Cramp::ANDI:
      if (isUInt<11>(MI->getOperand(2).getImm()))
        continue;
      addUses(*MI, Worklist, MRI);
      continue;
    case Cramp::ORI:
      if (!isUInt<11>(MI->getOperand(2).getImm()))
        continue;
      addUses(*MI, Worklist, MRI);
      continue;

    case Cramp::BEXTI:
      if (MI->getOperand(2).getImm() >= 32)
        return false;
      continue;

    // For these, lower word of output in these operations, depends only on
    // the lower word of input. So, we check all uses only read lower word.
    case Cramp::COPY:
    case Cramp::PHI:

    case Cramp::ADD:
    case Cramp::ADDI:
    case Cramp::AND:
    case Cramp::MUL:
    case Cramp::OR:
    case Cramp::SLL:
    case Cramp::SUB:
    case Cramp::XOR:
    case Cramp::XORI:

    case Cramp::ADD_UW:
    case Cramp::ANDN:
    case Cramp::CLMUL:
    case Cramp::ORC_B:
    case Cramp::ORN:
    case Cramp::SEXT_B:
    case Cramp::SEXT_H:
    case Cramp::SH1ADD:
    case Cramp::SH1ADD_UW:
    case Cramp::SH2ADD:
    case Cramp::SH2ADD_UW:
    case Cramp::SH3ADD:
    case Cramp::SH3ADD_UW:
    case Cramp::XNOR:
    case Cramp::ZEXT_H_RV64:
      addUses(*MI, Worklist, MRI);
      continue;
    default:
      return false;
    }
  }
  return true;
}

// This function returns true if the machine instruction always outputs a value
// where bits 63:32 match bit 31.
// Alternatively, if the instruction can be converted to W variant
// (e.g. ADD->ADDW) and all of its uses only use the lower word of its output,
// then return true and add the instr to FixableDef to be convereted later
// TODO: Allocate a bit in TSFlags for the W instructions?
// TODO: Add other W instructions.
static bool isSignExtendingOpW(MachineInstr &MI, MachineRegisterInfo &MRI,
                               SmallPtrSetImpl<MachineInstr *> &FixableDef) {
  switch (MI.getOpcode()) {
  case Cramp::LUI:
  case Cramp::LW:
  case Cramp::ADDW:
  case Cramp::ADDIW:
  case Cramp::SUBW:
  case Cramp::MULW:
  case Cramp::SLLW:
  case Cramp::SLLIW:
  case Cramp::SRAW:
  case Cramp::SRAIW:
  case Cramp::SRLW:
  case Cramp::SRLIW:
  case Cramp::DIVW:
  case Cramp::DIVUW:
  case Cramp::REMW:
  case Cramp::REMUW:
  case Cramp::ROLW:
  case Cramp::RORW:
  case Cramp::RORIW:
  case Cramp::CLZW:
  case Cramp::CTZW:
  case Cramp::CPOPW:
  case Cramp::FCVT_W_H:
  case Cramp::FCVT_WU_H:
  case Cramp::FCVT_W_S:
  case Cramp::FCVT_WU_S:
  case Cramp::FCVT_W_D:
  case Cramp::FCVT_WU_D:
  case Cramp::FMV_X_W:
  // The following aren't W instructions, but are either sign extended from a
  // smaller size, always outputs a small integer, or put zeros in bits 63:31.
  case Cramp::LBU:
  case Cramp::LHU:
  case Cramp::LB:
  case Cramp::LH:
  case Cramp::SLT:
  case Cramp::SLTI:
  case Cramp::SLTU:
  case Cramp::SLTIU:
  case Cramp::SEXT_B:
  case Cramp::SEXT_H:
  case Cramp::ZEXT_H_RV64:
  case Cramp::FMV_X_H:
  case Cramp::BEXT:
  case Cramp::BEXTI:
  case Cramp::CLZ:
  case Cramp::CPOP:
  case Cramp::CTZ:
    return true;
  // shifting right sufficiently makes the value 32-bit sign-extended
  case Cramp::SRAI:
    return MI.getOperand(2).getImm() >= 32;
  case Cramp::SRLI:
    return MI.getOperand(2).getImm() > 32;
  // The LI pattern ADDI rd, X0, imm is sign extended.
  case Cramp::ADDI:
    if (MI.getOperand(1).isReg() && MI.getOperand(1).getReg() == Cramp::X0)
      return true;
    if (isAllUsesReadW(MI, MRI)) {
      // transform to ADDIW
      FixableDef.insert(&MI);
      return true;
    }
    return false;
  // An ANDI with an 11 bit immediate will zero bits 63:11.
  case Cramp::ANDI:
    return isUInt<11>(MI.getOperand(2).getImm());
  // An ORI with an >11 bit immediate (negative 12-bit) will set bits 63:11.
  case Cramp::ORI:
    return !isUInt<11>(MI.getOperand(2).getImm());
  // Copying from X0 produces zero.
  case Cramp::COPY:
    return MI.getOperand(1).getReg() == Cramp::X0;

  // With these opcode, we can "fix" them with the W-version
  // if we know all users of the result only rely on bits 31:0
  case Cramp::SLLI:
    // SLLIW reads the lowest 5 bits, while SLLI reads lowest 6 bits
    if (MI.getOperand(2).getImm() >= 32)
      return false;
    LLVM_FALLTHROUGH;
  case Cramp::ADD:
  case Cramp::LD:
  case Cramp::LWU:
  case Cramp::MUL:
  case Cramp::SUB:
    if (isAllUsesReadW(MI, MRI)) {
      FixableDef.insert(&MI);
      return true;
    }
  }

  return false;
}

static bool isSignExtendedW(MachineInstr &OrigMI, MachineRegisterInfo &MRI,
                            SmallPtrSetImpl<MachineInstr *> &FixableDef) {

  SmallPtrSet<const MachineInstr *, 4> Visited;
  SmallVector<MachineInstr *, 4> Worklist;

  Worklist.push_back(&OrigMI);

  while (!Worklist.empty()) {
    MachineInstr *MI = Worklist.pop_back_val();

    // If we already visited this instruction, we don't need to check it again.
    if (!Visited.insert(MI).second)
      continue;

    // If this is a sign extending operation we don't need to look any further.
    if (isSignExtendingOpW(*MI, MRI, FixableDef))
      continue;

    // Is this an instruction that propagates sign extend.
    switch (MI->getOpcode()) {
    default:
      // Unknown opcode, give up.
      return false;
    case Cramp::COPY: {
      Register SrcReg = MI->getOperand(1).getReg();

      // TODO: Handle arguments and returns from calls?

      // If this is a copy from another register, check its source instruction.
      if (!SrcReg.isVirtual())
        return false;
      MachineInstr *SrcMI = MRI.getVRegDef(SrcReg);
      if (!SrcMI)
        return false;

      // Add SrcMI to the worklist.
      Worklist.push_back(SrcMI);
      break;
    }

    // For these, we just need to check if the 1st operand is sign extended.
    case Cramp::BCLRI:
    case Cramp::BINVI:
    case Cramp::BSETI:
      if (MI->getOperand(2).getImm() >= 31)
        return false;
      LLVM_FALLTHROUGH;
    case Cramp::REM:
    case Cramp::ANDI:
    case Cramp::ORI:
    case Cramp::XORI: {
      // |Remainder| is always <= |Dividend|. If D is 32-bit, then so is R.
      // DIV doesn't work because of the edge case 0xf..f 8000 0000 / (long)-1
      // Logical operations use a sign extended 12-bit immediate.
      Register SrcReg = MI->getOperand(1).getReg();
      if (!SrcReg.isVirtual())
        return false;
      MachineInstr *SrcMI = MRI.getVRegDef(SrcReg);
      if (!SrcMI)
        return false;

      // Add SrcMI to the worklist.
      Worklist.push_back(SrcMI);
      break;
    }
    case Cramp::REMU:
    case Cramp::AND:
    case Cramp::OR:
    case Cramp::XOR:
    case Cramp::ANDN:
    case Cramp::ORN:
    case Cramp::XNOR:
    case Cramp::MAX:
    case Cramp::MAXU:
    case Cramp::MIN:
    case Cramp::MINU:
    case Cramp::PHI: {
      // If all incoming values are sign-extended, the output of AND, OR, XOR,
      // MIN, MAX, or PHI is also sign-extended.

      // The input registers for PHI are operand 1, 3, ...
      // The input registers for others are operand 1 and 2.
      unsigned E = 3, D = 1;
      if (MI->getOpcode() == Cramp::PHI) {
        E = MI->getNumOperands();
        D = 2;
      }

      for (unsigned I = 1; I != E; I += D) {
        if (!MI->getOperand(I).isReg())
          return false;

        Register SrcReg = MI->getOperand(I).getReg();
        if (!SrcReg.isVirtual())
          return false;
        MachineInstr *SrcMI = MRI.getVRegDef(SrcReg);
        if (!SrcMI)
          return false;

        // Add SrcMI to the worklist.
        Worklist.push_back(SrcMI);
      }

      break;
    }
    }
  }

  // If we get here, then every node we visited produces a sign extended value
  // or propagated sign extended values. So the result must be sign extended.
  return true;
}

static unsigned getWOp(unsigned Opcode) {
  switch (Opcode) {
  case Cramp::ADDI:
    return Cramp::ADDIW;
  case Cramp::ADD:
    return Cramp::ADDW;
  case Cramp::LD:
  case Cramp::LWU:
    return Cramp::LW;
  case Cramp::MUL:
    return Cramp::MULW;
  case Cramp::SLLI:
    return Cramp::SLLIW;
  case Cramp::SUB:
    return Cramp::SUBW;
  default:
    llvm_unreachable("Unexpected opcode for replacement with W variant");
  }
}

bool CrampSExtWRemoval::runOnMachineFunction(MachineFunction &MF) {
  if (skipFunction(MF.getFunction()) || DisableSExtWRemoval)
    return false;

  MachineRegisterInfo &MRI = MF.getRegInfo();
  const CrampSubtarget &ST = MF.getSubtarget<CrampSubtarget>();

  if (!ST.is64Bit())
    return false;

  SmallPtrSet<MachineInstr *, 4> SExtWRemovalCands;

  // Replacing instructions invalidates the MI iterator
  // we collect the candidates, then iterate over them separately.
  for (MachineBasicBlock &MBB : MF) {
    for (auto I = MBB.begin(), IE = MBB.end(); I != IE;) {
      MachineInstr *MI = &*I++;

      // We're looking for the sext.w pattern ADDIW rd, rs1, 0.
      if (!Cramp::isSEXT_W(*MI))
        continue;

      // Input should be a virtual register.
      Register SrcReg = MI->getOperand(1).getReg();
      if (!SrcReg.isVirtual())
        continue;

      SExtWRemovalCands.insert(MI);
    }
  }

  bool MadeChange = false;
  for (auto MI : SExtWRemovalCands) {
    SmallPtrSet<MachineInstr *, 4> FixableDef;
    Register SrcReg = MI->getOperand(1).getReg();
    MachineInstr &SrcMI = *MRI.getVRegDef(SrcReg);

    // If all definitions reaching MI sign-extend their output,
    // then sext.w is redundant
    if (!isSignExtendedW(SrcMI, MRI, FixableDef))
      continue;

    Register DstReg = MI->getOperand(0).getReg();
    if (!MRI.constrainRegClass(SrcReg, MRI.getRegClass(DstReg)))
      continue;
    // Replace Fixable instructions with their W versions.
    for (MachineInstr *Fixable : FixableDef) {
      MachineBasicBlock &MBB = *Fixable->getParent();
      const DebugLoc &DL = Fixable->getDebugLoc();
      unsigned Code = getWOp(Fixable->getOpcode());
      MachineInstrBuilder Replacement =
          BuildMI(MBB, Fixable, DL, ST.getInstrInfo()->get(Code));
      for (auto Op : Fixable->operands())
        Replacement.add(Op);
      for (auto Op : Fixable->memoperands())
        Replacement.addMemOperand(Op);

      LLVM_DEBUG(dbgs() << "Replacing " << *Fixable);
      LLVM_DEBUG(dbgs() << "     with " << *Replacement);

      Fixable->eraseFromParent();
      ++NumTransformedToWInstrs;
    }

    LLVM_DEBUG(dbgs() << "Removing redundant sign-extension\n");
    MRI.replaceRegWith(DstReg, SrcReg);
    MRI.clearKillFlags(SrcReg);
    MI->eraseFromParent();
    ++NumRemovedSExtW;
    MadeChange = true;
  }

  return MadeChange;
}
