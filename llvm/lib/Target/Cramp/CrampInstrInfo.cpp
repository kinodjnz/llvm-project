//===-- CrampInstrInfo.cpp - Cramp Instruction Information ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Cramp implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "CrampInstrInfo.h"
#include "MCTargetDesc/CrampMatInt.h"
#include "Cramp.h"
#include "CrampMachineFunctionInfo.h"
#include "CrampSubtarget.h"
#include "CrampTargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Analysis/MemoryLocation.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define GEN_CHECK_COMPRESS_INSTR
#include "CrampGenCompressInstEmitter.inc"

#define GET_INSTRINFO_CTOR_DTOR
#define GET_INSTRINFO_NAMED_OPS
#include "CrampGenInstrInfo.inc"

static cl::opt<bool> PreferWholeRegisterMove(
    "cramp-prefer-whole-register-move", cl::init(false), cl::Hidden,
    cl::desc("Prefer whole register move for vector registers."));

namespace llvm {
namespace CrampVPseudosTable {

using namespace Cramp;

#define GET_CrampVPseudosTable_IMPL
#include "CrampGenSearchableTables.inc"

} // namespace CrampVPseudosTable
} // namespace llvm

CrampInstrInfo::CrampInstrInfo(CrampSubtarget &STI)
    : CrampGenInstrInfo(Cramp::ADJCALLSTACKDOWN, Cramp::ADJCALLSTACKUP),
      STI(STI) {}

MCInst CrampInstrInfo::getNop() const {
  if (STI.getFeatureBits()[Cramp::FeatureStdExtC])
    return MCInstBuilder(Cramp::C_NOP);
  return MCInstBuilder(Cramp::ADDI)
      .addReg(Cramp::X0)
      .addReg(Cramp::X0)
      .addImm(0);
}

unsigned CrampInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case Cramp::LB:
  case Cramp::LBU:
  case Cramp::LH:
  case Cramp::LHU:
  case Cramp::FLH:
  case Cramp::LW:
  case Cramp::FLW:
  case Cramp::LWU:
  case Cramp::LD:
  case Cramp::FLD:
    break;
  }

  if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
      MI.getOperand(2).getImm() == 0) {
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  }

  return 0;
}

unsigned CrampInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                            int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case Cramp::SB:
  case Cramp::SH:
  case Cramp::SW:
  case Cramp::FSH:
  case Cramp::FSW:
  case Cramp::SD:
  case Cramp::FSD:
    break;
  }

  if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
      MI.getOperand(2).getImm() == 0) {
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  }

  return 0;
}

static bool forwardCopyWillClobberTuple(unsigned DstReg, unsigned SrcReg,
                                        unsigned NumRegs) {
  return DstReg > SrcReg && (DstReg - SrcReg) < NumRegs;
}

static bool isConvertibleToVMV_V_V(const CrampSubtarget &STI,
                                   const MachineBasicBlock &MBB,
                                   MachineBasicBlock::const_iterator MBBI,
                                   MachineBasicBlock::const_iterator &DefMBBI,
                                   CrampII::VLMUL LMul) {
  if (PreferWholeRegisterMove)
    return false;

  assert(MBBI->getOpcode() == TargetOpcode::COPY &&
         "Unexpected COPY instruction.");
  Register SrcReg = MBBI->getOperand(1).getReg();
  const TargetRegisterInfo *TRI = STI.getRegisterInfo();

  bool FoundDef = false;
  bool FirstVSetVLI = false;
  unsigned FirstSEW = 0;
  while (MBBI != MBB.begin()) {
    --MBBI;
    if (MBBI->isMetaInstruction())
      continue;

    if (MBBI->getOpcode() == Cramp::PseudoVSETVLI ||
        MBBI->getOpcode() == Cramp::PseudoVSETVLIX0 ||
        MBBI->getOpcode() == Cramp::PseudoVSETIVLI) {
      // There is a vsetvli between COPY and source define instruction.
      // vy = def_vop ...  (producing instruction)
      // ...
      // vsetvli
      // ...
      // vx = COPY vy
      if (!FoundDef) {
        if (!FirstVSetVLI) {
          FirstVSetVLI = true;
          unsigned FirstVType = MBBI->getOperand(2).getImm();
          CrampII::VLMUL FirstLMul = CrampVType::getVLMUL(FirstVType);
          FirstSEW = CrampVType::getSEW(FirstVType);
          // The first encountered vsetvli must have the same lmul as the
          // register class of COPY.
          if (FirstLMul != LMul)
            return false;
        }
        // Only permit `vsetvli x0, x0, vtype` between COPY and the source
        // define instruction.
        if (MBBI->getOperand(0).getReg() != Cramp::X0)
          return false;
        if (MBBI->getOperand(1).isImm())
          return false;
        if (MBBI->getOperand(1).getReg() != Cramp::X0)
          return false;
        continue;
      }

      // MBBI is the first vsetvli before the producing instruction.
      unsigned VType = MBBI->getOperand(2).getImm();
      // If there is a vsetvli between COPY and the producing instruction.
      if (FirstVSetVLI) {
        // If SEW is different, return false.
        if (CrampVType::getSEW(VType) != FirstSEW)
          return false;
      }

      // If the vsetvli is tail undisturbed, keep the whole register move.
      if (!CrampVType::isTailAgnostic(VType))
        return false;

      // The checking is conservative. We only have register classes for
      // LMUL = 1/2/4/8. We should be able to convert vmv1r.v to vmv.v.v
      // for fractional LMUL operations. However, we could not use the vsetvli
      // lmul for widening operations. The result of widening operation is
      // 2 x LMUL.
      return LMul == CrampVType::getVLMUL(VType);
    } else if (MBBI->isInlineAsm() || MBBI->isCall()) {
      return false;
    } else if (MBBI->getNumDefs()) {
      // Check all the instructions which will change VL.
      // For example, vleff has implicit def VL.
      if (MBBI->modifiesRegister(Cramp::VL))
        return false;

      // Only converting whole register copies to vmv.v.v when the defining
      // value appears in the explicit operands.
      for (const MachineOperand &MO : MBBI->explicit_operands()) {
        if (!MO.isReg() || !MO.isDef())
          continue;
        if (!FoundDef && TRI->isSubRegisterEq(MO.getReg(), SrcReg)) {
          // We only permit the source of COPY has the same LMUL as the defined
          // operand.
          // There are cases we need to keep the whole register copy if the LMUL
          // is different.
          // For example,
          // $x0 = PseudoVSETIVLI 4, 73   // vsetivli zero, 4, e16,m2,ta,m
          // $v28m4 = PseudoVWADD_VV_M2 $v26m2, $v8m2
          // # The COPY may be created by vlmul_trunc intrinsic.
          // $v26m2 = COPY renamable $v28m2, implicit killed $v28m4
          //
          // After widening, the valid value will be 4 x e32 elements. If we
          // convert the COPY to vmv.v.v, it will only copy 4 x e16 elements.
          // FIXME: The COPY of subregister of Zvlsseg register will not be able
          // to convert to vmv.v.[v|i] under the constraint.
          if (MO.getReg() != SrcReg)
            return false;

          // In widening reduction instructions with LMUL_1 input vector case,
          // only checking the LMUL is insufficient due to reduction result is
          // always LMUL_1.
          // For example,
          // $x11 = PseudoVSETIVLI 1, 64 // vsetivli a1, 1, e8, m1, ta, mu
          // $v8m1 = PseudoVWREDSUM_VS_M1 $v26, $v27
          // $v26 = COPY killed renamable $v8
          // After widening, The valid value will be 1 x e16 elements. If we
          // convert the COPY to vmv.v.v, it will only copy 1 x e8 elements.
          uint64_t TSFlags = MBBI->getDesc().TSFlags;
          if (CrampII::isRVVWideningReduction(TSFlags))
            return false;

          // Found the definition.
          FoundDef = true;
          DefMBBI = MBBI;
          // If the producing instruction does not depend on vsetvli, do not
          // convert COPY to vmv.v.v. For example, VL1R_V or PseudoVRELOAD.
          if (!CrampII::hasSEWOp(TSFlags))
            return false;
          break;
        }
      }
    }
  }

  return false;
}

void CrampInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MBBI,
                                 const DebugLoc &DL, MCRegister DstReg,
                                 MCRegister SrcReg, bool KillSrc) const {
  if (Cramp::GPRRegClass.contains(DstReg, SrcReg)) {
    BuildMI(MBB, MBBI, DL, get(Cramp::ADDI), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addImm(0);
    return;
  }

  // Handle copy from csr
  if (Cramp::VCSRRegClass.contains(SrcReg) &&
      Cramp::GPRRegClass.contains(DstReg)) {
    const TargetRegisterInfo &TRI = *STI.getRegisterInfo();
    BuildMI(MBB, MBBI, DL, get(Cramp::CSRRS), DstReg)
      .addImm(CrampSysReg::lookupSysRegByName(TRI.getName(SrcReg))->Encoding)
      .addReg(Cramp::X0);
    return;
  }

  // FPR->FPR copies and VR->VR copies.
  unsigned Opc;
  bool IsScalableVector = true;
  unsigned NF = 1;
  CrampII::VLMUL LMul = CrampII::LMUL_1;
  unsigned SubRegIdx = Cramp::sub_vrm1_0;
  if (Cramp::FPR16RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::FSGNJ_H;
    IsScalableVector = false;
  } else if (Cramp::FPR32RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::FSGNJ_S;
    IsScalableVector = false;
  } else if (Cramp::FPR64RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::FSGNJ_D;
    IsScalableVector = false;
  } else if (Cramp::VRRegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV1R_V;
    LMul = CrampII::LMUL_1;
  } else if (Cramp::VRM2RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV2R_V;
    LMul = CrampII::LMUL_2;
  } else if (Cramp::VRM4RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV4R_V;
    LMul = CrampII::LMUL_4;
  } else if (Cramp::VRM8RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV8R_V;
    LMul = CrampII::LMUL_8;
  } else if (Cramp::VRN2M1RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV1R_V;
    SubRegIdx = Cramp::sub_vrm1_0;
    NF = 2;
    LMul = CrampII::LMUL_1;
  } else if (Cramp::VRN2M2RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV2R_V;
    SubRegIdx = Cramp::sub_vrm2_0;
    NF = 2;
    LMul = CrampII::LMUL_2;
  } else if (Cramp::VRN2M4RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV4R_V;
    SubRegIdx = Cramp::sub_vrm4_0;
    NF = 2;
    LMul = CrampII::LMUL_4;
  } else if (Cramp::VRN3M1RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV1R_V;
    SubRegIdx = Cramp::sub_vrm1_0;
    NF = 3;
    LMul = CrampII::LMUL_1;
  } else if (Cramp::VRN3M2RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV2R_V;
    SubRegIdx = Cramp::sub_vrm2_0;
    NF = 3;
    LMul = CrampII::LMUL_2;
  } else if (Cramp::VRN4M1RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV1R_V;
    SubRegIdx = Cramp::sub_vrm1_0;
    NF = 4;
    LMul = CrampII::LMUL_1;
  } else if (Cramp::VRN4M2RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV2R_V;
    SubRegIdx = Cramp::sub_vrm2_0;
    NF = 4;
    LMul = CrampII::LMUL_2;
  } else if (Cramp::VRN5M1RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV1R_V;
    SubRegIdx = Cramp::sub_vrm1_0;
    NF = 5;
    LMul = CrampII::LMUL_1;
  } else if (Cramp::VRN6M1RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV1R_V;
    SubRegIdx = Cramp::sub_vrm1_0;
    NF = 6;
    LMul = CrampII::LMUL_1;
  } else if (Cramp::VRN7M1RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV1R_V;
    SubRegIdx = Cramp::sub_vrm1_0;
    NF = 7;
    LMul = CrampII::LMUL_1;
  } else if (Cramp::VRN8M1RegClass.contains(DstReg, SrcReg)) {
    Opc = Cramp::PseudoVMV1R_V;
    SubRegIdx = Cramp::sub_vrm1_0;
    NF = 8;
    LMul = CrampII::LMUL_1;
  } else {
    llvm_unreachable("Impossible reg-to-reg copy");
  }

  if (IsScalableVector) {
    bool UseVMV_V_V = false;
    MachineBasicBlock::const_iterator DefMBBI;
    unsigned DefExplicitOpNum;
    unsigned VIOpc;
    if (isConvertibleToVMV_V_V(STI, MBB, MBBI, DefMBBI, LMul)) {
      UseVMV_V_V = true;
      DefExplicitOpNum = DefMBBI->getNumExplicitOperands();
      // We only need to handle LMUL = 1/2/4/8 here because we only define
      // vector register classes for LMUL = 1/2/4/8.
      switch (LMul) {
      default:
        llvm_unreachable("Impossible LMUL for vector register copy.");
      case CrampII::LMUL_1:
        Opc = Cramp::PseudoVMV_V_V_M1;
        VIOpc = Cramp::PseudoVMV_V_I_M1;
        break;
      case CrampII::LMUL_2:
        Opc = Cramp::PseudoVMV_V_V_M2;
        VIOpc = Cramp::PseudoVMV_V_I_M2;
        break;
      case CrampII::LMUL_4:
        Opc = Cramp::PseudoVMV_V_V_M4;
        VIOpc = Cramp::PseudoVMV_V_I_M4;
        break;
      case CrampII::LMUL_8:
        Opc = Cramp::PseudoVMV_V_V_M8;
        VIOpc = Cramp::PseudoVMV_V_I_M8;
        break;
      }
    }

    bool UseVMV_V_I = false;
    if (UseVMV_V_V && (DefMBBI->getOpcode() == VIOpc)) {
      UseVMV_V_I = true;
      Opc = VIOpc;
    }

    if (NF == 1) {
      auto MIB = BuildMI(MBB, MBBI, DL, get(Opc), DstReg);
      if (UseVMV_V_I)
        MIB = MIB.add(DefMBBI->getOperand(1));
      else
        MIB = MIB.addReg(SrcReg, getKillRegState(KillSrc));
      if (UseVMV_V_V) {
        // The last two arguments of vector instructions are
        // AVL, SEW. We also need to append the implicit-use vl and vtype.
        MIB.add(DefMBBI->getOperand(DefExplicitOpNum - 2)); // AVL
        MIB.add(DefMBBI->getOperand(DefExplicitOpNum - 1)); // SEW
        MIB.addReg(Cramp::VL, RegState::Implicit);
        MIB.addReg(Cramp::VTYPE, RegState::Implicit);
      }
    } else {
      const TargetRegisterInfo *TRI = STI.getRegisterInfo();

      int I = 0, End = NF, Incr = 1;
      unsigned SrcEncoding = TRI->getEncodingValue(SrcReg);
      unsigned DstEncoding = TRI->getEncodingValue(DstReg);
      unsigned LMulVal;
      bool Fractional;
      std::tie(LMulVal, Fractional) = CrampVType::decodeVLMUL(LMul);
      assert(!Fractional && "It is impossible be fractional lmul here.");
      if (forwardCopyWillClobberTuple(DstEncoding, SrcEncoding, NF * LMulVal)) {
        I = NF - 1;
        End = -1;
        Incr = -1;
      }

      for (; I != End; I += Incr) {
        auto MIB = BuildMI(MBB, MBBI, DL, get(Opc),
                           TRI->getSubReg(DstReg, SubRegIdx + I));
        if (UseVMV_V_I)
          MIB = MIB.add(DefMBBI->getOperand(1));
        else
          MIB = MIB.addReg(TRI->getSubReg(SrcReg, SubRegIdx + I),
                           getKillRegState(KillSrc));
        if (UseVMV_V_V) {
          MIB.add(DefMBBI->getOperand(DefExplicitOpNum - 2)); // AVL
          MIB.add(DefMBBI->getOperand(DefExplicitOpNum - 1)); // SEW
          MIB.addReg(Cramp::VL, RegState::Implicit);
          MIB.addReg(Cramp::VTYPE, RegState::Implicit);
        }
      }
    }
  } else {
    BuildMI(MBB, MBBI, DL, get(Opc), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addReg(SrcReg, getKillRegState(KillSrc));
  }
}

void CrampInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         Register SrcReg, bool IsKill, int FI,
                                         const TargetRegisterClass *RC,
                                         const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  MachineFunction *MF = MBB.getParent();
  MachineFrameInfo &MFI = MF->getFrameInfo();

  unsigned Opcode;
  bool IsScalableVector = true;
  bool IsZvlsseg = true;
  if (Cramp::GPRRegClass.hasSubClassEq(RC)) {
    Opcode = TRI->getRegSizeInBits(Cramp::GPRRegClass) == 32 ?
             Cramp::SW : Cramp::SD;
    IsScalableVector = false;
  } else if (Cramp::FPR16RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::FSH;
    IsScalableVector = false;
  } else if (Cramp::FPR32RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::FSW;
    IsScalableVector = false;
  } else if (Cramp::FPR64RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::FSD;
    IsScalableVector = false;
  } else if (Cramp::VRRegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::PseudoVSPILL_M1;
    IsZvlsseg = false;
  } else if (Cramp::VRM2RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::PseudoVSPILL_M2;
    IsZvlsseg = false;
  } else if (Cramp::VRM4RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::PseudoVSPILL_M4;
    IsZvlsseg = false;
  } else if (Cramp::VRM8RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::PseudoVSPILL_M8;
    IsZvlsseg = false;
  } else if (Cramp::VRN2M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL2_M1;
  else if (Cramp::VRN2M2RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL2_M2;
  else if (Cramp::VRN2M4RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL2_M4;
  else if (Cramp::VRN3M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL3_M1;
  else if (Cramp::VRN3M2RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL3_M2;
  else if (Cramp::VRN4M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL4_M1;
  else if (Cramp::VRN4M2RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL4_M2;
  else if (Cramp::VRN5M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL5_M1;
  else if (Cramp::VRN6M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL6_M1;
  else if (Cramp::VRN7M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL7_M1;
  else if (Cramp::VRN8M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVSPILL8_M1;
  else
    llvm_unreachable("Can't store this register to stack slot");

  if (IsScalableVector) {
    MachineMemOperand *MMO = MF->getMachineMemOperand(
        MachinePointerInfo::getFixedStack(*MF, FI), MachineMemOperand::MOStore,
        MemoryLocation::UnknownSize, MFI.getObjectAlign(FI));

    MFI.setStackID(FI, TargetStackID::ScalableVector);
    auto MIB = BuildMI(MBB, I, DL, get(Opcode))
                   .addReg(SrcReg, getKillRegState(IsKill))
                   .addFrameIndex(FI)
                   .addMemOperand(MMO);
    if (IsZvlsseg) {
      // For spilling/reloading Zvlsseg registers, append the dummy field for
      // the scaled vector length. The argument will be used when expanding
      // these pseudo instructions.
      MIB.addReg(Cramp::X0);
    }
  } else {
    MachineMemOperand *MMO = MF->getMachineMemOperand(
        MachinePointerInfo::getFixedStack(*MF, FI), MachineMemOperand::MOStore,
        MFI.getObjectSize(FI), MFI.getObjectAlign(FI));

    BuildMI(MBB, I, DL, get(Opcode))
        .addReg(SrcReg, getKillRegState(IsKill))
        .addFrameIndex(FI)
        .addImm(0)
        .addMemOperand(MMO);
  }
}

void CrampInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator I,
                                          Register DstReg, int FI,
                                          const TargetRegisterClass *RC,
                                          const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  MachineFunction *MF = MBB.getParent();
  MachineFrameInfo &MFI = MF->getFrameInfo();

  unsigned Opcode;
  bool IsScalableVector = true;
  bool IsZvlsseg = true;
  if (Cramp::GPRRegClass.hasSubClassEq(RC)) {
    Opcode = TRI->getRegSizeInBits(Cramp::GPRRegClass) == 32 ?
             Cramp::LW : Cramp::LD;
    IsScalableVector = false;
  } else if (Cramp::FPR16RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::FLH;
    IsScalableVector = false;
  } else if (Cramp::FPR32RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::FLW;
    IsScalableVector = false;
  } else if (Cramp::FPR64RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::FLD;
    IsScalableVector = false;
  } else if (Cramp::VRRegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::PseudoVRELOAD_M1;
    IsZvlsseg = false;
  } else if (Cramp::VRM2RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::PseudoVRELOAD_M2;
    IsZvlsseg = false;
  } else if (Cramp::VRM4RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::PseudoVRELOAD_M4;
    IsZvlsseg = false;
  } else if (Cramp::VRM8RegClass.hasSubClassEq(RC)) {
    Opcode = Cramp::PseudoVRELOAD_M8;
    IsZvlsseg = false;
  } else if (Cramp::VRN2M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD2_M1;
  else if (Cramp::VRN2M2RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD2_M2;
  else if (Cramp::VRN2M4RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD2_M4;
  else if (Cramp::VRN3M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD3_M1;
  else if (Cramp::VRN3M2RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD3_M2;
  else if (Cramp::VRN4M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD4_M1;
  else if (Cramp::VRN4M2RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD4_M2;
  else if (Cramp::VRN5M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD5_M1;
  else if (Cramp::VRN6M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD6_M1;
  else if (Cramp::VRN7M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD7_M1;
  else if (Cramp::VRN8M1RegClass.hasSubClassEq(RC))
    Opcode = Cramp::PseudoVRELOAD8_M1;
  else
    llvm_unreachable("Can't load this register from stack slot");

  if (IsScalableVector) {
    MachineMemOperand *MMO = MF->getMachineMemOperand(
        MachinePointerInfo::getFixedStack(*MF, FI), MachineMemOperand::MOLoad,
        MemoryLocation::UnknownSize, MFI.getObjectAlign(FI));

    MFI.setStackID(FI, TargetStackID::ScalableVector);
    auto MIB = BuildMI(MBB, I, DL, get(Opcode), DstReg)
                   .addFrameIndex(FI)
                   .addMemOperand(MMO);
    if (IsZvlsseg) {
      // For spilling/reloading Zvlsseg registers, append the dummy field for
      // the scaled vector length. The argument will be used when expanding
      // these pseudo instructions.
      MIB.addReg(Cramp::X0);
    }
  } else {
    MachineMemOperand *MMO = MF->getMachineMemOperand(
        MachinePointerInfo::getFixedStack(*MF, FI), MachineMemOperand::MOLoad,
        MFI.getObjectSize(FI), MFI.getObjectAlign(FI));

    BuildMI(MBB, I, DL, get(Opcode), DstReg)
        .addFrameIndex(FI)
        .addImm(0)
        .addMemOperand(MMO);
  }
}

MachineInstr *CrampInstrInfo::foldMemoryOperandImpl(
    MachineFunction &MF, MachineInstr &MI, ArrayRef<unsigned> Ops,
    MachineBasicBlock::iterator InsertPt, int FrameIndex, LiveIntervals *LIS,
    VirtRegMap *VRM) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  // The below optimizations narrow the load so they are only valid for little
  // endian.
  // TODO: Support big endian by adding an offset into the frame object?
  if (MF.getDataLayout().isBigEndian())
    return nullptr;

  // Fold load from stack followed by sext.w into lw.
  // TODO: Fold with sext.b, sext.h, zext.b, zext.h, zext.w?
  if (Ops.size() != 1 || Ops[0] != 1)
   return nullptr;

  unsigned LoadOpc;
  switch (MI.getOpcode()) {
  default:
    if (Cramp::isSEXT_W(MI)) {
      LoadOpc = Cramp::LW;
      break;
    }
    if (Cramp::isZEXT_W(MI)) {
      LoadOpc = Cramp::LWU;
      break;
    }
    if (Cramp::isZEXT_B(MI)) {
      LoadOpc = Cramp::LBU;
      break;
    }
    return nullptr;
  case Cramp::SEXT_H:
    LoadOpc = Cramp::LH;
    break;
  case Cramp::SEXT_B:
    LoadOpc = Cramp::LB;
    break;
  case Cramp::ZEXT_H_RV32:
  case Cramp::ZEXT_H_RV64:
    LoadOpc = Cramp::LHU;
    break;
  }

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOLoad, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  Register DstReg = MI.getOperand(0).getReg();
  return BuildMI(*MI.getParent(), InsertPt, MI.getDebugLoc(), get(LoadOpc),
                 DstReg)
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addMemOperand(MMO);
}

void CrampInstrInfo::movImm(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MBBI,
                            const DebugLoc &DL, Register DstReg, uint64_t Val,
                            MachineInstr::MIFlag Flag) const {
  Register SrcReg = Cramp::X0;

  if (!STI.is64Bit() && !isInt<32>(Val))
    report_fatal_error("Should only materialize 32-bit constants for RV32");

  CrampMatInt::InstSeq Seq =
      CrampMatInt::generateInstSeq(Val, STI.getFeatureBits());
  assert(!Seq.empty());

  for (CrampMatInt::Inst &Inst : Seq) {
    switch (Inst.getOpndKind()) {
    case CrampMatInt::Imm:
      BuildMI(MBB, MBBI, DL, get(Inst.Opc), DstReg)
          .addImm(Inst.Imm)
          .setMIFlag(Flag);
      break;
    case CrampMatInt::RegX0:
      BuildMI(MBB, MBBI, DL, get(Inst.Opc), DstReg)
          .addReg(SrcReg, RegState::Kill)
          .addReg(Cramp::X0)
          .setMIFlag(Flag);
      break;
    case CrampMatInt::RegReg:
      BuildMI(MBB, MBBI, DL, get(Inst.Opc), DstReg)
          .addReg(SrcReg, RegState::Kill)
          .addReg(SrcReg, RegState::Kill)
          .setMIFlag(Flag);
      break;
    case CrampMatInt::RegImm:
      BuildMI(MBB, MBBI, DL, get(Inst.Opc), DstReg)
          .addReg(SrcReg, RegState::Kill)
          .addImm(Inst.Imm)
          .setMIFlag(Flag);
      break;
    }

    // Only the first instruction has X0 as its source.
    SrcReg = DstReg;
  }
}

static CrampCC::CondCode getCondFromBranchOpc(unsigned Opc) {
  switch (Opc) {
  default:
    return CrampCC::COND_INVALID;
  case Cramp::BEQ:
    return CrampCC::COND_EQ;
  case Cramp::BNE:
    return CrampCC::COND_NE;
  case Cramp::BLT:
    return CrampCC::COND_LT;
  case Cramp::BGE:
    return CrampCC::COND_GE;
  case Cramp::BLTU:
    return CrampCC::COND_LTU;
  case Cramp::BGEU:
    return CrampCC::COND_GEU;
  }
}

// The contents of values added to Cond are not examined outside of
// CrampInstrInfo, giving us flexibility in what to push to it. For Cramp, we
// push BranchOpcode, Reg1, Reg2.
static void parseCondBranch(MachineInstr &LastInst, MachineBasicBlock *&Target,
                            SmallVectorImpl<MachineOperand> &Cond) {
  // Block ends with fall-through condbranch.
  assert(LastInst.getDesc().isConditionalBranch() &&
         "Unknown conditional branch");
  Target = LastInst.getOperand(2).getMBB();
  unsigned CC = getCondFromBranchOpc(LastInst.getOpcode());
  Cond.push_back(MachineOperand::CreateImm(CC));
  Cond.push_back(LastInst.getOperand(0));
  Cond.push_back(LastInst.getOperand(1));
}

const MCInstrDesc &CrampInstrInfo::getBrCond(CrampCC::CondCode CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Unknown condition code!");
  case CrampCC::COND_EQ:
    return get(Cramp::BEQ);
  case CrampCC::COND_NE:
    return get(Cramp::BNE);
  case CrampCC::COND_LT:
    return get(Cramp::BLT);
  case CrampCC::COND_GE:
    return get(Cramp::BGE);
  case CrampCC::COND_LTU:
    return get(Cramp::BLTU);
  case CrampCC::COND_GEU:
    return get(Cramp::BGEU);
  }
}

CrampCC::CondCode CrampCC::getOppositeBranchCondition(CrampCC::CondCode CC) {
  switch (CC) {
  default:
    llvm_unreachable("Unrecognized conditional branch");
  case CrampCC::COND_EQ:
    return CrampCC::COND_NE;
  case CrampCC::COND_NE:
    return CrampCC::COND_EQ;
  case CrampCC::COND_LT:
    return CrampCC::COND_GE;
  case CrampCC::COND_GE:
    return CrampCC::COND_LT;
  case CrampCC::COND_LTU:
    return CrampCC::COND_GEU;
  case CrampCC::COND_GEU:
    return CrampCC::COND_LTU;
  }
}

bool CrampInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                   MachineBasicBlock *&TBB,
                                   MachineBasicBlock *&FBB,
                                   SmallVectorImpl<MachineOperand> &Cond,
                                   bool AllowModify) const {
  TBB = FBB = nullptr;
  Cond.clear();

  // If the block has no terminators, it just falls into the block after it.
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end() || !isUnpredicatedTerminator(*I))
    return false;

  // Count the number of terminators and find the first unconditional or
  // indirect branch.
  MachineBasicBlock::iterator FirstUncondOrIndirectBr = MBB.end();
  int NumTerminators = 0;
  for (auto J = I.getReverse(); J != MBB.rend() && isUnpredicatedTerminator(*J);
       J++) {
    NumTerminators++;
    if (J->getDesc().isUnconditionalBranch() ||
        J->getDesc().isIndirectBranch()) {
      FirstUncondOrIndirectBr = J.getReverse();
    }
  }

  // If AllowModify is true, we can erase any terminators after
  // FirstUncondOrIndirectBR.
  if (AllowModify && FirstUncondOrIndirectBr != MBB.end()) {
    while (std::next(FirstUncondOrIndirectBr) != MBB.end()) {
      std::next(FirstUncondOrIndirectBr)->eraseFromParent();
      NumTerminators--;
    }
    I = FirstUncondOrIndirectBr;
  }

  // We can't handle blocks that end in an indirect branch.
  if (I->getDesc().isIndirectBranch())
    return true;

  // We can't handle blocks with more than 2 terminators.
  if (NumTerminators > 2)
    return true;

  // Handle a single unconditional branch.
  if (NumTerminators == 1 && I->getDesc().isUnconditionalBranch()) {
    TBB = getBranchDestBlock(*I);
    return false;
  }

  // Handle a single conditional branch.
  if (NumTerminators == 1 && I->getDesc().isConditionalBranch()) {
    parseCondBranch(*I, TBB, Cond);
    return false;
  }

  // Handle a conditional branch followed by an unconditional branch.
  if (NumTerminators == 2 && std::prev(I)->getDesc().isConditionalBranch() &&
      I->getDesc().isUnconditionalBranch()) {
    parseCondBranch(*std::prev(I), TBB, Cond);
    FBB = getBranchDestBlock(*I);
    return false;
  }

  // Otherwise, we can't handle this.
  return true;
}

unsigned CrampInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                      int *BytesRemoved) const {
  if (BytesRemoved)
    *BytesRemoved = 0;
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end())
    return 0;

  if (!I->getDesc().isUnconditionalBranch() &&
      !I->getDesc().isConditionalBranch())
    return 0;

  // Remove the branch.
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);
  I->eraseFromParent();

  I = MBB.end();

  if (I == MBB.begin())
    return 1;
  --I;
  if (!I->getDesc().isConditionalBranch())
    return 1;

  // Remove the branch.
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);
  I->eraseFromParent();
  return 2;
}

// Inserts a branch into the end of the specific MachineBasicBlock, returning
// the number of instructions inserted.
unsigned CrampInstrInfo::insertBranch(
    MachineBasicBlock &MBB, MachineBasicBlock *TBB, MachineBasicBlock *FBB,
    ArrayRef<MachineOperand> Cond, const DebugLoc &DL, int *BytesAdded) const {
  if (BytesAdded)
    *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 3 || Cond.size() == 0) &&
         "Cramp branch conditions have two components!");

  // Unconditional branch.
  if (Cond.empty()) {
    MachineInstr &MI = *BuildMI(&MBB, DL, get(Cramp::PseudoBR)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Either a one or two-way conditional branch.
  auto CC = static_cast<CrampCC::CondCode>(Cond[0].getImm());
  MachineInstr &CondMI =
      *BuildMI(&MBB, DL, getBrCond(CC)).add(Cond[1]).add(Cond[2]).addMBB(TBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(CondMI);

  // One-way conditional branch.
  if (!FBB)
    return 1;

  // Two-way conditional branch.
  MachineInstr &MI = *BuildMI(&MBB, DL, get(Cramp::PseudoBR)).addMBB(FBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(MI);
  return 2;
}

void CrampInstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                          MachineBasicBlock &DestBB,
                                          MachineBasicBlock &RestoreBB,
                                          const DebugLoc &DL, int64_t BrOffset,
                                          RegScavenger *RS) const {
  assert(RS && "RegScavenger required for long branching");
  assert(MBB.empty() &&
         "new block should be inserted for expanding unconditional branch");
  assert(MBB.pred_size() == 1);

  MachineFunction *MF = MBB.getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();

  if (!isInt<32>(BrOffset))
    report_fatal_error(
        "Branch offsets outside of the signed 32-bit range not supported");

  // FIXME: A virtual register must be used initially, as the register
  // scavenger won't work with empty blocks (SIInstrInfo::insertIndirectBranch
  // uses the same workaround).
  Register ScratchReg = MRI.createVirtualRegister(&Cramp::GPRRegClass);
  auto II = MBB.end();

  MachineInstr &MI = *BuildMI(MBB, II, DL, get(Cramp::PseudoJump))
                          .addReg(ScratchReg, RegState::Define | RegState::Dead)
                          .addMBB(&DestBB, CrampII::MO_CALL);

  RS->enterBasicBlockEnd(MBB);
  Register Scav = RS->scavengeRegisterBackwards(Cramp::GPRRegClass,
                                                MI.getIterator(), false, 0);
  // TODO: The case when there is no scavenged register needs special handling.
  assert(Scav != Cramp::NoRegister && "No register is scavenged!");
  MRI.replaceRegWith(ScratchReg, Scav);
  MRI.clearVirtRegs();
  RS->setRegUsed(Scav);
}

bool CrampInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert((Cond.size() == 3) && "Invalid branch condition!");
  auto CC = static_cast<CrampCC::CondCode>(Cond[0].getImm());
  Cond[0].setImm(getOppositeBranchCondition(CC));
  return false;
}

MachineBasicBlock *
CrampInstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  assert(MI.getDesc().isBranch() && "Unexpected opcode!");
  // The branch target is always the last operand.
  int NumOp = MI.getNumExplicitOperands();
  return MI.getOperand(NumOp - 1).getMBB();
}

bool CrampInstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                           int64_t BrOffset) const {
  unsigned XLen = STI.getXLen();
  // Ideally we could determine the supported branch offset from the
  // CrampII::FormMask, but this can't be used for Pseudo instructions like
  // PseudoBR.
  switch (BranchOp) {
  default:
    llvm_unreachable("Unexpected opcode!");
  case Cramp::BEQ:
  case Cramp::BNE:
  case Cramp::BLT:
  case Cramp::BGE:
  case Cramp::BLTU:
  case Cramp::BGEU:
    return isIntN(13, BrOffset);
  case Cramp::JAL:
  case Cramp::PseudoBR:
    return isIntN(21, BrOffset);
  case Cramp::PseudoJump:
    return isIntN(32, SignExtend64(BrOffset + 0x800, XLen));
  }
}

unsigned CrampInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  if (MI.isMetaInstruction())
    return 0;

  unsigned Opcode = MI.getOpcode();

  if (Opcode == TargetOpcode::INLINEASM ||
      Opcode == TargetOpcode::INLINEASM_BR) {
    const MachineFunction &MF = *MI.getParent()->getParent();
    const auto &TM = static_cast<const CrampTargetMachine &>(MF.getTarget());
    return getInlineAsmLength(MI.getOperand(0).getSymbolName(),
                              *TM.getMCAsmInfo());
  }

  if (MI.getParent() && MI.getParent()->getParent()) {
    const auto MF = MI.getMF();
    const auto &TM = static_cast<const CrampTargetMachine &>(MF->getTarget());
    const MCRegisterInfo &MRI = *TM.getMCRegisterInfo();
    const MCSubtargetInfo &STI = *TM.getMCSubtargetInfo();
    const CrampSubtarget &ST = MF->getSubtarget<CrampSubtarget>();
    if (isCompressibleInst(MI, &ST, MRI, STI))
      return 2;
  }
  return get(Opcode).getSize();
}

bool CrampInstrInfo::isAsCheapAsAMove(const MachineInstr &MI) const {
  const unsigned Opcode = MI.getOpcode();
  switch (Opcode) {
  default:
    break;
  case Cramp::FSGNJ_D:
  case Cramp::FSGNJ_S:
  case Cramp::FSGNJ_H:
    // The canonical floating-point move is fsgnj rd, rs, rs.
    return MI.getOperand(1).isReg() && MI.getOperand(2).isReg() &&
           MI.getOperand(1).getReg() == MI.getOperand(2).getReg();
  case Cramp::ADDI:
  case Cramp::ORI:
  case Cramp::XORI:
    return (MI.getOperand(1).isReg() &&
            MI.getOperand(1).getReg() == Cramp::X0) ||
           (MI.getOperand(2).isImm() && MI.getOperand(2).getImm() == 0);
  }
  return MI.isAsCheapAsAMove();
}

Optional<DestSourcePair>
CrampInstrInfo::isCopyInstrImpl(const MachineInstr &MI) const {
  if (MI.isMoveReg())
    return DestSourcePair{MI.getOperand(0), MI.getOperand(1)};
  switch (MI.getOpcode()) {
  default:
    break;
  case Cramp::ADDI:
    // Operand 1 can be a frameindex but callers expect registers
    if (MI.getOperand(1).isReg() && MI.getOperand(2).isImm() &&
        MI.getOperand(2).getImm() == 0)
      return DestSourcePair{MI.getOperand(0), MI.getOperand(1)};
    break;
  case Cramp::FSGNJ_D:
  case Cramp::FSGNJ_S:
  case Cramp::FSGNJ_H:
    // The canonical floating-point move is fsgnj rd, rs, rs.
    if (MI.getOperand(1).isReg() && MI.getOperand(2).isReg() &&
        MI.getOperand(1).getReg() == MI.getOperand(2).getReg())
      return DestSourcePair{MI.getOperand(0), MI.getOperand(1)};
    break;
  }
  return None;
}

bool CrampInstrInfo::verifyInstruction(const MachineInstr &MI,
                                       StringRef &ErrInfo) const {
  const MCInstrInfo *MCII = STI.getInstrInfo();
  MCInstrDesc const &Desc = MCII->get(MI.getOpcode());

  for (auto &OI : enumerate(Desc.operands())) {
    unsigned OpType = OI.value().OperandType;
    if (OpType >= CrampOp::OPERAND_FIRST_Cramp_IMM &&
        OpType <= CrampOp::OPERAND_LAST_Cramp_IMM) {
      const MachineOperand &MO = MI.getOperand(OI.index());
      if (MO.isImm()) {
        int64_t Imm = MO.getImm();
        bool Ok;
        switch (OpType) {
        default:
          llvm_unreachable("Unexpected operand type");

          // clang-format off
#define CASE_OPERAND_UIMM(NUM)                                                 \
  case CrampOp::OPERAND_UIMM##NUM:                                             \
    Ok = isUInt<NUM>(Imm);                                                     \
    break;
        CASE_OPERAND_UIMM(2)
        CASE_OPERAND_UIMM(3)
        CASE_OPERAND_UIMM(4)
        CASE_OPERAND_UIMM(5)
        CASE_OPERAND_UIMM(7)
        CASE_OPERAND_UIMM(12)
        CASE_OPERAND_UIMM(20)
          // clang-format on
        case CrampOp::OPERAND_SIMM12:
          Ok = isInt<12>(Imm);
          break;
        case CrampOp::OPERAND_SIMM12_LSB00000:
          Ok = isShiftedInt<7, 5>(Imm);
          break;
        case CrampOp::OPERAND_UIMMLOG2XLEN:
          if (STI.getTargetTriple().isArch64Bit())
            Ok = isUInt<6>(Imm);
          else
            Ok = isUInt<5>(Imm);
          break;
        case CrampOp::OPERAND_RVKRNUM:
          Ok = Imm >= 0 && Imm <= 10;
          break;
        }
        if (!Ok) {
          ErrInfo = "Invalid immediate";
          return false;
        }
      }
    }
  }

  return true;
}

// Return true if get the base operand, byte offset of an instruction and the
// memory width. Width is the size of memory that is being loaded/stored.
bool CrampInstrInfo::getMemOperandWithOffsetWidth(
    const MachineInstr &LdSt, const MachineOperand *&BaseReg, int64_t &Offset,
    unsigned &Width, const TargetRegisterInfo *TRI) const {
  if (!LdSt.mayLoadOrStore())
    return false;

  // Here we assume the standard RISC-V ISA, which uses a base+offset
  // addressing mode. You'll need to relax these conditions to support custom
  // load/stores instructions.
  if (LdSt.getNumExplicitOperands() != 3)
    return false;
  if (!LdSt.getOperand(1).isReg() || !LdSt.getOperand(2).isImm())
    return false;

  if (!LdSt.hasOneMemOperand())
    return false;

  Width = (*LdSt.memoperands_begin())->getSize();
  BaseReg = &LdSt.getOperand(1);
  Offset = LdSt.getOperand(2).getImm();
  return true;
}

bool CrampInstrInfo::areMemAccessesTriviallyDisjoint(
    const MachineInstr &MIa, const MachineInstr &MIb) const {
  assert(MIa.mayLoadOrStore() && "MIa must be a load or store.");
  assert(MIb.mayLoadOrStore() && "MIb must be a load or store.");

  if (MIa.hasUnmodeledSideEffects() || MIb.hasUnmodeledSideEffects() ||
      MIa.hasOrderedMemoryRef() || MIb.hasOrderedMemoryRef())
    return false;

  // Retrieve the base register, offset from the base register and width. Width
  // is the size of memory that is being loaded/stored (e.g. 1, 2, 4).  If
  // base registers are identical, and the offset of a lower memory access +
  // the width doesn't overlap the offset of a higher memory access,
  // then the memory accesses are different.
  const TargetRegisterInfo *TRI = STI.getRegisterInfo();
  const MachineOperand *BaseOpA = nullptr, *BaseOpB = nullptr;
  int64_t OffsetA = 0, OffsetB = 0;
  unsigned int WidthA = 0, WidthB = 0;
  if (getMemOperandWithOffsetWidth(MIa, BaseOpA, OffsetA, WidthA, TRI) &&
      getMemOperandWithOffsetWidth(MIb, BaseOpB, OffsetB, WidthB, TRI)) {
    if (BaseOpA->isIdenticalTo(*BaseOpB)) {
      int LowOffset = std::min(OffsetA, OffsetB);
      int HighOffset = std::max(OffsetA, OffsetB);
      int LowWidth = (LowOffset == OffsetA) ? WidthA : WidthB;
      if (LowOffset + LowWidth <= HighOffset)
        return true;
    }
  }
  return false;
}

std::pair<unsigned, unsigned>
CrampInstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  const unsigned Mask = CrampII::MO_DIRECT_FLAG_MASK;
  return std::make_pair(TF & Mask, TF & ~Mask);
}

ArrayRef<std::pair<unsigned, const char *>>
CrampInstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
  using namespace CrampII;
  static const std::pair<unsigned, const char *> TargetFlags[] = {
      {MO_CALL, "cramp-call"},
      {MO_PLT, "cramp-plt"},
      {MO_LO, "cramp-lo"},
      {MO_HI, "cramp-hi"},
      {MO_PCREL_LO, "cramp-pcrel-lo"},
      {MO_PCREL_HI, "cramp-pcrel-hi"},
      {MO_GOT_HI, "cramp-got-hi"},
      {MO_TPREL_LO, "cramp-tprel-lo"},
      {MO_TPREL_HI, "cramp-tprel-hi"},
      {MO_TPREL_ADD, "cramp-tprel-add"},
      {MO_TLS_GOT_HI, "cramp-tls-got-hi"},
      {MO_TLS_GD_HI, "cramp-tls-gd-hi"}};
  return makeArrayRef(TargetFlags);
}
bool CrampInstrInfo::isFunctionSafeToOutlineFrom(
    MachineFunction &MF, bool OutlineFromLinkOnceODRs) const {
  const Function &F = MF.getFunction();

  // Can F be deduplicated by the linker? If it can, don't outline from it.
  if (!OutlineFromLinkOnceODRs && F.hasLinkOnceODRLinkage())
    return false;

  // Don't outline from functions with section markings; the program could
  // expect that all the code is in the named section.
  if (F.hasSection())
    return false;

  // It's safe to outline from MF.
  return true;
}

bool CrampInstrInfo::isMBBSafeToOutlineFrom(MachineBasicBlock &MBB,
                                            unsigned &Flags) const {
  // More accurate safety checking is done in getOutliningCandidateInfo.
  return TargetInstrInfo::isMBBSafeToOutlineFrom(MBB, Flags);
}

// Enum values indicating how an outlined call should be constructed.
enum MachineOutlinerConstructionID {
  MachineOutlinerDefault
};

bool CrampInstrInfo::shouldOutlineFromFunctionByDefault(
    MachineFunction &MF) const {
  return MF.getFunction().hasMinSize();
}

outliner::OutlinedFunction CrampInstrInfo::getOutliningCandidateInfo(
    std::vector<outliner::Candidate> &RepeatedSequenceLocs) const {

  // First we need to filter out candidates where the X5 register (IE t0) can't
  // be used to setup the function call.
  auto CannotInsertCall = [](outliner::Candidate &C) {
    const TargetRegisterInfo *TRI = C.getMF()->getSubtarget().getRegisterInfo();
    return !C.isAvailableAcrossAndOutOfSeq(Cramp::X5, *TRI);
  };

  llvm::erase_if(RepeatedSequenceLocs, CannotInsertCall);

  // If the sequence doesn't have enough candidates left, then we're done.
  if (RepeatedSequenceLocs.size() < 2)
    return outliner::OutlinedFunction();

  unsigned SequenceSize = 0;

  auto I = RepeatedSequenceLocs[0].front();
  auto E = std::next(RepeatedSequenceLocs[0].back());
  for (; I != E; ++I)
    SequenceSize += getInstSizeInBytes(*I);

  // call t0, function = 8 bytes.
  unsigned CallOverhead = 8;
  for (auto &C : RepeatedSequenceLocs)
    C.setCallInfo(MachineOutlinerDefault, CallOverhead);

  // jr t0 = 4 bytes, 2 bytes if compressed instructions are enabled.
  unsigned FrameOverhead = 4;
  if (RepeatedSequenceLocs[0].getMF()->getSubtarget()
          .getFeatureBits()[Cramp::FeatureStdExtC])
    FrameOverhead = 2;

  return outliner::OutlinedFunction(RepeatedSequenceLocs, SequenceSize,
                                    FrameOverhead, MachineOutlinerDefault);
}

outliner::InstrType
CrampInstrInfo::getOutliningType(MachineBasicBlock::iterator &MBBI,
                                 unsigned Flags) const {
  MachineInstr &MI = *MBBI;
  MachineBasicBlock *MBB = MI.getParent();
  const TargetRegisterInfo *TRI =
      MBB->getParent()->getSubtarget().getRegisterInfo();

  // Positions generally can't safely be outlined.
  if (MI.isPosition()) {
    // We can manually strip out CFI instructions later.
    if (MI.isCFIInstruction())
      // If current function has exception handling code, we can't outline &
      // strip these CFI instructions since it may break .eh_frame section
      // needed in unwinding.
      return MI.getMF()->getFunction().needsUnwindTableEntry()
                 ? outliner::InstrType::Illegal
                 : outliner::InstrType::Invisible;

    return outliner::InstrType::Illegal;
  }

  // Don't trust the user to write safe inline assembly.
  if (MI.isInlineAsm())
    return outliner::InstrType::Illegal;

  // We can't outline branches to other basic blocks.
  if (MI.isTerminator() && !MBB->succ_empty())
    return outliner::InstrType::Illegal;

  // We need support for tail calls to outlined functions before return
  // statements can be allowed.
  if (MI.isReturn())
    return outliner::InstrType::Illegal;

  // Don't allow modifying the X5 register which we use for return addresses for
  // these outlined functions.
  if (MI.modifiesRegister(Cramp::X5, TRI) ||
      MI.getDesc().hasImplicitDefOfPhysReg(Cramp::X5))
    return outliner::InstrType::Illegal;

  // Make sure the operands don't reference something unsafe.
  for (const auto &MO : MI.operands())
    if (MO.isMBB() || MO.isBlockAddress() || MO.isCPI() || MO.isJTI())
      return outliner::InstrType::Illegal;

  // Don't allow instructions which won't be materialized to impact outlining
  // analysis.
  if (MI.isMetaInstruction())
    return outliner::InstrType::Invisible;

  return outliner::InstrType::Legal;
}

void CrampInstrInfo::buildOutlinedFrame(
    MachineBasicBlock &MBB, MachineFunction &MF,
    const outliner::OutlinedFunction &OF) const {

  // Strip out any CFI instructions
  bool Changed = true;
  while (Changed) {
    Changed = false;
    auto I = MBB.begin();
    auto E = MBB.end();
    for (; I != E; ++I) {
      if (I->isCFIInstruction()) {
        I->removeFromParent();
        Changed = true;
        break;
      }
    }
  }

  MBB.addLiveIn(Cramp::X5);

  // Add in a return instruction to the end of the outlined frame.
  MBB.insert(MBB.end(), BuildMI(MF, DebugLoc(), get(Cramp::JALR))
      .addReg(Cramp::X0, RegState::Define)
      .addReg(Cramp::X5)
      .addImm(0));
}

MachineBasicBlock::iterator CrampInstrInfo::insertOutlinedCall(
    Module &M, MachineBasicBlock &MBB, MachineBasicBlock::iterator &It,
    MachineFunction &MF, outliner::Candidate &C) const {

  // Add in a call instruction to the outlined function at the given location.
  It = MBB.insert(It,
                  BuildMI(MF, DebugLoc(), get(Cramp::PseudoCALLReg), Cramp::X5)
                      .addGlobalAddress(M.getNamedValue(MF.getName()), 0,
                                        CrampII::MO_CALL));
  return It;
}

// MIR printer helper function to annotate Operands with a comment.
std::string CrampInstrInfo::createMIROperandComment(
    const MachineInstr &MI, const MachineOperand &Op, unsigned OpIdx,
    const TargetRegisterInfo *TRI) const {
  // Print a generic comment for this operand if there is one.
  std::string GenericComment =
      TargetInstrInfo::createMIROperandComment(MI, Op, OpIdx, TRI);
  if (!GenericComment.empty())
    return GenericComment;

  // If not, we must have an immediate operand.
  if (!Op.isImm())
    return std::string();

  std::string Comment;
  raw_string_ostream OS(Comment);

  uint64_t TSFlags = MI.getDesc().TSFlags;

  // Print the full VType operand of vsetvli/vsetivli instructions, and the SEW
  // operand of vector codegen pseudos.
  if ((MI.getOpcode() == Cramp::VSETVLI || MI.getOpcode() == Cramp::VSETIVLI ||
       MI.getOpcode() == Cramp::PseudoVSETVLI ||
       MI.getOpcode() == Cramp::PseudoVSETIVLI ||
       MI.getOpcode() == Cramp::PseudoVSETVLIX0) &&
      OpIdx == 2) {
    unsigned Imm = MI.getOperand(OpIdx).getImm();
    CrampVType::printVType(Imm, OS);
  } else if (CrampII::hasSEWOp(TSFlags)) {
    unsigned NumOperands = MI.getNumExplicitOperands();
    bool HasPolicy = CrampII::hasVecPolicyOp(TSFlags);

    // The SEW operand is before any policy operand.
    if (OpIdx != NumOperands - HasPolicy - 1)
      return std::string();

    unsigned Log2SEW = MI.getOperand(OpIdx).getImm();
    unsigned SEW = Log2SEW ? 1 << Log2SEW : 8;
    assert(CrampVType::isValidSEW(SEW) && "Unexpected SEW");

    OS << "e" << SEW;
  }

  OS.flush();
  return Comment;
}

// clang-format off
#define CASE_VFMA_OPCODE_COMMON(OP, TYPE, LMUL)                                \
  Cramp::PseudoV##OP##_##TYPE##_##LMUL

#define CASE_VFMA_OPCODE_LMULS_M1(OP, TYPE)                                    \
  CASE_VFMA_OPCODE_COMMON(OP, TYPE, M1):                                       \
  case CASE_VFMA_OPCODE_COMMON(OP, TYPE, M2):                                  \
  case CASE_VFMA_OPCODE_COMMON(OP, TYPE, M4):                                  \
  case CASE_VFMA_OPCODE_COMMON(OP, TYPE, M8)

#define CASE_VFMA_OPCODE_LMULS_MF2(OP, TYPE)                                   \
  CASE_VFMA_OPCODE_COMMON(OP, TYPE, MF2):                                      \
  case CASE_VFMA_OPCODE_LMULS_M1(OP, TYPE)

#define CASE_VFMA_OPCODE_LMULS_MF4(OP, TYPE)                                   \
  CASE_VFMA_OPCODE_COMMON(OP, TYPE, MF4):                                      \
  case CASE_VFMA_OPCODE_LMULS_MF2(OP, TYPE)

#define CASE_VFMA_OPCODE_LMULS(OP, TYPE)                                       \
  CASE_VFMA_OPCODE_COMMON(OP, TYPE, MF8):                                      \
  case CASE_VFMA_OPCODE_LMULS_MF4(OP, TYPE)

#define CASE_VFMA_SPLATS(OP)                                                   \
  CASE_VFMA_OPCODE_LMULS_MF4(OP, VF16):                                        \
  case CASE_VFMA_OPCODE_LMULS_MF2(OP, VF32):                                   \
  case CASE_VFMA_OPCODE_LMULS_M1(OP, VF64)
// clang-format on

bool CrampInstrInfo::findCommutedOpIndices(const MachineInstr &MI,
                                           unsigned &SrcOpIdx1,
                                           unsigned &SrcOpIdx2) const {
  const MCInstrDesc &Desc = MI.getDesc();
  if (!Desc.isCommutable())
    return false;

  switch (MI.getOpcode()) {
  case CASE_VFMA_SPLATS(FMADD):
  case CASE_VFMA_SPLATS(FMSUB):
  case CASE_VFMA_SPLATS(FMACC):
  case CASE_VFMA_SPLATS(FMSAC):
  case CASE_VFMA_SPLATS(FNMADD):
  case CASE_VFMA_SPLATS(FNMSUB):
  case CASE_VFMA_SPLATS(FNMACC):
  case CASE_VFMA_SPLATS(FNMSAC):
  case CASE_VFMA_OPCODE_LMULS_MF4(FMACC, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FMSAC, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FNMACC, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FNMSAC, VV):
  case CASE_VFMA_OPCODE_LMULS(MADD, VX):
  case CASE_VFMA_OPCODE_LMULS(NMSUB, VX):
  case CASE_VFMA_OPCODE_LMULS(MACC, VX):
  case CASE_VFMA_OPCODE_LMULS(NMSAC, VX):
  case CASE_VFMA_OPCODE_LMULS(MACC, VV):
  case CASE_VFMA_OPCODE_LMULS(NMSAC, VV): {
    // If the tail policy is undisturbed we can't commute.
    assert(CrampII::hasVecPolicyOp(MI.getDesc().TSFlags));
    if ((MI.getOperand(MI.getNumExplicitOperands() - 1).getImm() & 1) == 0)
      return false;

    // For these instructions we can only swap operand 1 and operand 3 by
    // changing the opcode.
    unsigned CommutableOpIdx1 = 1;
    unsigned CommutableOpIdx2 = 3;
    if (!fixCommutedOpIndices(SrcOpIdx1, SrcOpIdx2, CommutableOpIdx1,
                              CommutableOpIdx2))
      return false;
    return true;
  }
  case CASE_VFMA_OPCODE_LMULS_MF4(FMADD, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FMSUB, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FNMADD, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FNMSUB, VV):
  case CASE_VFMA_OPCODE_LMULS(MADD, VV):
  case CASE_VFMA_OPCODE_LMULS(NMSUB, VV): {
    // If the tail policy is undisturbed we can't commute.
    assert(CrampII::hasVecPolicyOp(MI.getDesc().TSFlags));
    if ((MI.getOperand(MI.getNumExplicitOperands() - 1).getImm() & 1) == 0)
      return false;

    // For these instructions we have more freedom. We can commute with the
    // other multiplicand or with the addend/subtrahend/minuend.

    // Any fixed operand must be from source 1, 2 or 3.
    if (SrcOpIdx1 != CommuteAnyOperandIndex && SrcOpIdx1 > 3)
      return false;
    if (SrcOpIdx2 != CommuteAnyOperandIndex && SrcOpIdx2 > 3)
      return false;

    // It both ops are fixed one must be the tied source.
    if (SrcOpIdx1 != CommuteAnyOperandIndex &&
        SrcOpIdx2 != CommuteAnyOperandIndex && SrcOpIdx1 != 1 && SrcOpIdx2 != 1)
      return false;

    // Look for two different register operands assumed to be commutable
    // regardless of the FMA opcode. The FMA opcode is adjusted later if
    // needed.
    if (SrcOpIdx1 == CommuteAnyOperandIndex ||
        SrcOpIdx2 == CommuteAnyOperandIndex) {
      // At least one of operands to be commuted is not specified and
      // this method is free to choose appropriate commutable operands.
      unsigned CommutableOpIdx1 = SrcOpIdx1;
      if (SrcOpIdx1 == SrcOpIdx2) {
        // Both of operands are not fixed. Set one of commutable
        // operands to the tied source.
        CommutableOpIdx1 = 1;
      } else if (SrcOpIdx1 == CommuteAnyOperandIndex) {
        // Only one of the operands is not fixed.
        CommutableOpIdx1 = SrcOpIdx2;
      }

      // CommutableOpIdx1 is well defined now. Let's choose another commutable
      // operand and assign its index to CommutableOpIdx2.
      unsigned CommutableOpIdx2;
      if (CommutableOpIdx1 != 1) {
        // If we haven't already used the tied source, we must use it now.
        CommutableOpIdx2 = 1;
      } else {
        Register Op1Reg = MI.getOperand(CommutableOpIdx1).getReg();

        // The commuted operands should have different registers.
        // Otherwise, the commute transformation does not change anything and
        // is useless. We use this as a hint to make our decision.
        if (Op1Reg != MI.getOperand(2).getReg())
          CommutableOpIdx2 = 2;
        else
          CommutableOpIdx2 = 3;
      }

      // Assign the found pair of commutable indices to SrcOpIdx1 and
      // SrcOpIdx2 to return those values.
      if (!fixCommutedOpIndices(SrcOpIdx1, SrcOpIdx2, CommutableOpIdx1,
                                CommutableOpIdx2))
        return false;
    }

    return true;
  }
  }

  return TargetInstrInfo::findCommutedOpIndices(MI, SrcOpIdx1, SrcOpIdx2);
}

#define CASE_VFMA_CHANGE_OPCODE_COMMON(OLDOP, NEWOP, TYPE, LMUL)               \
  case Cramp::PseudoV##OLDOP##_##TYPE##_##LMUL:                                \
    Opc = Cramp::PseudoV##NEWOP##_##TYPE##_##LMUL;                             \
    break;

#define CASE_VFMA_CHANGE_OPCODE_LMULS_M1(OLDOP, NEWOP, TYPE)                   \
  CASE_VFMA_CHANGE_OPCODE_COMMON(OLDOP, NEWOP, TYPE, M1)                       \
  CASE_VFMA_CHANGE_OPCODE_COMMON(OLDOP, NEWOP, TYPE, M2)                       \
  CASE_VFMA_CHANGE_OPCODE_COMMON(OLDOP, NEWOP, TYPE, M4)                       \
  CASE_VFMA_CHANGE_OPCODE_COMMON(OLDOP, NEWOP, TYPE, M8)

#define CASE_VFMA_CHANGE_OPCODE_LMULS_MF2(OLDOP, NEWOP, TYPE)                  \
  CASE_VFMA_CHANGE_OPCODE_COMMON(OLDOP, NEWOP, TYPE, MF2)                      \
  CASE_VFMA_CHANGE_OPCODE_LMULS_M1(OLDOP, NEWOP, TYPE)

#define CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(OLDOP, NEWOP, TYPE)                  \
  CASE_VFMA_CHANGE_OPCODE_COMMON(OLDOP, NEWOP, TYPE, MF4)                      \
  CASE_VFMA_CHANGE_OPCODE_LMULS_MF2(OLDOP, NEWOP, TYPE)

#define CASE_VFMA_CHANGE_OPCODE_LMULS(OLDOP, NEWOP, TYPE)                      \
  CASE_VFMA_CHANGE_OPCODE_COMMON(OLDOP, NEWOP, TYPE, MF8)                      \
  CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(OLDOP, NEWOP, TYPE)

#define CASE_VFMA_CHANGE_OPCODE_SPLATS(OLDOP, NEWOP)                           \
  CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(OLDOP, NEWOP, VF16)                        \
  CASE_VFMA_CHANGE_OPCODE_LMULS_MF2(OLDOP, NEWOP, VF32)                        \
  CASE_VFMA_CHANGE_OPCODE_LMULS_M1(OLDOP, NEWOP, VF64)

MachineInstr *CrampInstrInfo::commuteInstructionImpl(MachineInstr &MI,
                                                     bool NewMI,
                                                     unsigned OpIdx1,
                                                     unsigned OpIdx2) const {
  auto cloneIfNew = [NewMI](MachineInstr &MI) -> MachineInstr & {
    if (NewMI)
      return *MI.getParent()->getParent()->CloneMachineInstr(&MI);
    return MI;
  };

  switch (MI.getOpcode()) {
  case CASE_VFMA_SPLATS(FMACC):
  case CASE_VFMA_SPLATS(FMADD):
  case CASE_VFMA_SPLATS(FMSAC):
  case CASE_VFMA_SPLATS(FMSUB):
  case CASE_VFMA_SPLATS(FNMACC):
  case CASE_VFMA_SPLATS(FNMADD):
  case CASE_VFMA_SPLATS(FNMSAC):
  case CASE_VFMA_SPLATS(FNMSUB):
  case CASE_VFMA_OPCODE_LMULS_MF4(FMACC, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FMSAC, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FNMACC, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FNMSAC, VV):
  case CASE_VFMA_OPCODE_LMULS(MADD, VX):
  case CASE_VFMA_OPCODE_LMULS(NMSUB, VX):
  case CASE_VFMA_OPCODE_LMULS(MACC, VX):
  case CASE_VFMA_OPCODE_LMULS(NMSAC, VX):
  case CASE_VFMA_OPCODE_LMULS(MACC, VV):
  case CASE_VFMA_OPCODE_LMULS(NMSAC, VV): {
    // It only make sense to toggle these between clobbering the
    // addend/subtrahend/minuend one of the multiplicands.
    assert((OpIdx1 == 1 || OpIdx2 == 1) && "Unexpected opcode index");
    assert((OpIdx1 == 3 || OpIdx2 == 3) && "Unexpected opcode index");
    unsigned Opc;
    switch (MI.getOpcode()) {
      default:
        llvm_unreachable("Unexpected opcode");
      CASE_VFMA_CHANGE_OPCODE_SPLATS(FMACC, FMADD)
      CASE_VFMA_CHANGE_OPCODE_SPLATS(FMADD, FMACC)
      CASE_VFMA_CHANGE_OPCODE_SPLATS(FMSAC, FMSUB)
      CASE_VFMA_CHANGE_OPCODE_SPLATS(FMSUB, FMSAC)
      CASE_VFMA_CHANGE_OPCODE_SPLATS(FNMACC, FNMADD)
      CASE_VFMA_CHANGE_OPCODE_SPLATS(FNMADD, FNMACC)
      CASE_VFMA_CHANGE_OPCODE_SPLATS(FNMSAC, FNMSUB)
      CASE_VFMA_CHANGE_OPCODE_SPLATS(FNMSUB, FNMSAC)
      CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(FMACC, FMADD, VV)
      CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(FMSAC, FMSUB, VV)
      CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(FNMACC, FNMADD, VV)
      CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(FNMSAC, FNMSUB, VV)
      CASE_VFMA_CHANGE_OPCODE_LMULS(MACC, MADD, VX)
      CASE_VFMA_CHANGE_OPCODE_LMULS(MADD, MACC, VX)
      CASE_VFMA_CHANGE_OPCODE_LMULS(NMSAC, NMSUB, VX)
      CASE_VFMA_CHANGE_OPCODE_LMULS(NMSUB, NMSAC, VX)
      CASE_VFMA_CHANGE_OPCODE_LMULS(MACC, MADD, VV)
      CASE_VFMA_CHANGE_OPCODE_LMULS(NMSAC, NMSUB, VV)
    }

    auto &WorkingMI = cloneIfNew(MI);
    WorkingMI.setDesc(get(Opc));
    return TargetInstrInfo::commuteInstructionImpl(WorkingMI, /*NewMI=*/false,
                                                   OpIdx1, OpIdx2);
  }
  case CASE_VFMA_OPCODE_LMULS_MF4(FMADD, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FMSUB, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FNMADD, VV):
  case CASE_VFMA_OPCODE_LMULS_MF4(FNMSUB, VV):
  case CASE_VFMA_OPCODE_LMULS(MADD, VV):
  case CASE_VFMA_OPCODE_LMULS(NMSUB, VV): {
    assert((OpIdx1 == 1 || OpIdx2 == 1) && "Unexpected opcode index");
    // If one of the operands, is the addend we need to change opcode.
    // Otherwise we're just swapping 2 of the multiplicands.
    if (OpIdx1 == 3 || OpIdx2 == 3) {
      unsigned Opc;
      switch (MI.getOpcode()) {
        default:
          llvm_unreachable("Unexpected opcode");
        CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(FMADD, FMACC, VV)
        CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(FMSUB, FMSAC, VV)
        CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(FNMADD, FNMACC, VV)
        CASE_VFMA_CHANGE_OPCODE_LMULS_MF4(FNMSUB, FNMSAC, VV)
        CASE_VFMA_CHANGE_OPCODE_LMULS(MADD, MACC, VV)
        CASE_VFMA_CHANGE_OPCODE_LMULS(NMSUB, NMSAC, VV)
      }

      auto &WorkingMI = cloneIfNew(MI);
      WorkingMI.setDesc(get(Opc));
      return TargetInstrInfo::commuteInstructionImpl(WorkingMI, /*NewMI=*/false,
                                                     OpIdx1, OpIdx2);
    }
    // Let the default code handle it.
    break;
  }
  }

  return TargetInstrInfo::commuteInstructionImpl(MI, NewMI, OpIdx1, OpIdx2);
}

#undef CASE_VFMA_CHANGE_OPCODE_SPLATS
#undef CASE_VFMA_CHANGE_OPCODE_LMULS
#undef CASE_VFMA_CHANGE_OPCODE_COMMON
#undef CASE_VFMA_SPLATS
#undef CASE_VFMA_OPCODE_LMULS
#undef CASE_VFMA_OPCODE_COMMON

// clang-format off
#define CASE_WIDEOP_OPCODE_COMMON(OP, LMUL)                                    \
  Cramp::PseudoV##OP##_##LMUL##_TIED

#define CASE_WIDEOP_OPCODE_LMULS_MF4(OP)                                       \
  CASE_WIDEOP_OPCODE_COMMON(OP, MF4):                                          \
  case CASE_WIDEOP_OPCODE_COMMON(OP, MF2):                                     \
  case CASE_WIDEOP_OPCODE_COMMON(OP, M1):                                      \
  case CASE_WIDEOP_OPCODE_COMMON(OP, M2):                                      \
  case CASE_WIDEOP_OPCODE_COMMON(OP, M4)

#define CASE_WIDEOP_OPCODE_LMULS(OP)                                           \
  CASE_WIDEOP_OPCODE_COMMON(OP, MF8):                                          \
  case CASE_WIDEOP_OPCODE_LMULS_MF4(OP)
// clang-format on

#define CASE_WIDEOP_CHANGE_OPCODE_COMMON(OP, LMUL)                             \
  case Cramp::PseudoV##OP##_##LMUL##_TIED:                                     \
    NewOpc = Cramp::PseudoV##OP##_##LMUL;                                      \
    break;

#define CASE_WIDEOP_CHANGE_OPCODE_LMULS_MF4(OP)                                 \
  CASE_WIDEOP_CHANGE_OPCODE_COMMON(OP, MF4)                                    \
  CASE_WIDEOP_CHANGE_OPCODE_COMMON(OP, MF2)                                    \
  CASE_WIDEOP_CHANGE_OPCODE_COMMON(OP, M1)                                     \
  CASE_WIDEOP_CHANGE_OPCODE_COMMON(OP, M2)                                     \
  CASE_WIDEOP_CHANGE_OPCODE_COMMON(OP, M4)

#define CASE_WIDEOP_CHANGE_OPCODE_LMULS(OP)                                    \
  CASE_WIDEOP_CHANGE_OPCODE_COMMON(OP, MF8)                                    \
  CASE_WIDEOP_CHANGE_OPCODE_LMULS_MF4(OP)

MachineInstr *CrampInstrInfo::convertToThreeAddress(MachineInstr &MI,
                                                    LiveVariables *LV,
                                                    LiveIntervals *LIS) const {
  switch (MI.getOpcode()) {
  default:
    break;
  case CASE_WIDEOP_OPCODE_LMULS_MF4(FWADD_WV):
  case CASE_WIDEOP_OPCODE_LMULS_MF4(FWSUB_WV):
  case CASE_WIDEOP_OPCODE_LMULS(WADD_WV):
  case CASE_WIDEOP_OPCODE_LMULS(WADDU_WV):
  case CASE_WIDEOP_OPCODE_LMULS(WSUB_WV):
  case CASE_WIDEOP_OPCODE_LMULS(WSUBU_WV): {
    // If the tail policy is undisturbed we can't convert.
    assert(CrampII::hasVecPolicyOp(MI.getDesc().TSFlags) &&
           MI.getNumExplicitOperands() == 6);
    if ((MI.getOperand(5).getImm() & 1) == 0)
      return nullptr;

    // clang-format off
    unsigned NewOpc;
    switch (MI.getOpcode()) {
    default:
      llvm_unreachable("Unexpected opcode");
    CASE_WIDEOP_CHANGE_OPCODE_LMULS_MF4(FWADD_WV)
    CASE_WIDEOP_CHANGE_OPCODE_LMULS_MF4(FWSUB_WV)
    CASE_WIDEOP_CHANGE_OPCODE_LMULS(WADD_WV)
    CASE_WIDEOP_CHANGE_OPCODE_LMULS(WADDU_WV)
    CASE_WIDEOP_CHANGE_OPCODE_LMULS(WSUB_WV)
    CASE_WIDEOP_CHANGE_OPCODE_LMULS(WSUBU_WV)
    }
    // clang-format on

    MachineBasicBlock &MBB = *MI.getParent();
    MachineInstrBuilder MIB = BuildMI(MBB, MI, MI.getDebugLoc(), get(NewOpc))
                                  .add(MI.getOperand(0))
                                  .add(MI.getOperand(1))
                                  .add(MI.getOperand(2))
                                  .add(MI.getOperand(3))
                                  .add(MI.getOperand(4));
    MIB.copyImplicitOps(MI);

    if (LV) {
      unsigned NumOps = MI.getNumOperands();
      for (unsigned I = 1; I < NumOps; ++I) {
        MachineOperand &Op = MI.getOperand(I);
        if (Op.isReg() && Op.isKill())
          LV->replaceKillInstruction(Op.getReg(), MI, *MIB);
      }
    }

    if (LIS) {
      SlotIndex Idx = LIS->ReplaceMachineInstrInMaps(MI, *MIB);

      if (MI.getOperand(0).isEarlyClobber()) {
        // Use operand 1 was tied to early-clobber def operand 0, so its live
        // interval could have ended at an early-clobber slot. Now they are not
        // tied we need to update it to the normal register slot.
        LiveInterval &LI = LIS->getInterval(MI.getOperand(1).getReg());
        LiveRange::Segment *S = LI.getSegmentContaining(Idx);
        if (S->end == Idx.getRegSlot(true))
          S->end = Idx.getRegSlot();
      }
    }

    return MIB;
  }
  }

  return nullptr;
}

#undef CASE_WIDEOP_CHANGE_OPCODE_LMULS
#undef CASE_WIDEOP_CHANGE_OPCODE_COMMON
#undef CASE_WIDEOP_OPCODE_LMULS
#undef CASE_WIDEOP_OPCODE_COMMON

Register CrampInstrInfo::getVLENFactoredAmount(MachineFunction &MF,
                                               MachineBasicBlock &MBB,
                                               MachineBasicBlock::iterator II,
                                               const DebugLoc &DL,
                                               int64_t Amount,
                                               MachineInstr::MIFlag Flag) const {
  assert(Amount > 0 && "There is no need to get VLEN scaled value.");
  assert(Amount % 8 == 0 &&
         "Reserve the stack by the multiple of one vector size.");

  MachineRegisterInfo &MRI = MF.getRegInfo();
  int64_t NumOfVReg = Amount / 8;

  Register VL = MRI.createVirtualRegister(&Cramp::GPRRegClass);
  BuildMI(MBB, II, DL, get(Cramp::PseudoReadVLENB), VL)
    .setMIFlag(Flag);
  assert(isInt<32>(NumOfVReg) &&
         "Expect the number of vector registers within 32-bits.");
  if (isPowerOf2_32(NumOfVReg)) {
    uint32_t ShiftAmount = Log2_32(NumOfVReg);
    if (ShiftAmount == 0)
      return VL;
    BuildMI(MBB, II, DL, get(Cramp::SLLI), VL)
        .addReg(VL, RegState::Kill)
        .addImm(ShiftAmount)
        .setMIFlag(Flag);
  } else if (STI.hasStdExtZba() &&
             ((NumOfVReg % 3 == 0 && isPowerOf2_64(NumOfVReg / 3)) ||
              (NumOfVReg % 5 == 0 && isPowerOf2_64(NumOfVReg / 5)) ||
              (NumOfVReg % 9 == 0 && isPowerOf2_64(NumOfVReg / 9)))) {
    // We can use Zba SHXADD+SLLI instructions for multiply in some cases.
    unsigned Opc;
    uint32_t ShiftAmount;
    if (NumOfVReg % 9 == 0) {
      Opc = Cramp::SH3ADD;
      ShiftAmount = Log2_64(NumOfVReg / 9);
    } else if (NumOfVReg % 5 == 0) {
      Opc = Cramp::SH2ADD;
      ShiftAmount = Log2_64(NumOfVReg / 5);
    } else if (NumOfVReg % 3 == 0) {
      Opc = Cramp::SH1ADD;
      ShiftAmount = Log2_64(NumOfVReg / 3);
    } else {
      llvm_unreachable("Unexpected number of vregs");
    }
    if (ShiftAmount)
      BuildMI(MBB, II, DL, get(Cramp::SLLI), VL)
          .addReg(VL, RegState::Kill)
          .addImm(ShiftAmount)
          .setMIFlag(Flag);
    BuildMI(MBB, II, DL, get(Opc), VL)
        .addReg(VL, RegState::Kill)
        .addReg(VL)
        .setMIFlag(Flag);
  } else if (isPowerOf2_32(NumOfVReg - 1)) {
    Register ScaledRegister = MRI.createVirtualRegister(&Cramp::GPRRegClass);
    uint32_t ShiftAmount = Log2_32(NumOfVReg - 1);
    BuildMI(MBB, II, DL, get(Cramp::SLLI), ScaledRegister)
        .addReg(VL)
        .addImm(ShiftAmount)
        .setMIFlag(Flag);
    BuildMI(MBB, II, DL, get(Cramp::ADD), VL)
        .addReg(ScaledRegister, RegState::Kill)
        .addReg(VL, RegState::Kill)
        .setMIFlag(Flag);
  } else if (isPowerOf2_32(NumOfVReg + 1)) {
    Register ScaledRegister = MRI.createVirtualRegister(&Cramp::GPRRegClass);
    uint32_t ShiftAmount = Log2_32(NumOfVReg + 1);
    BuildMI(MBB, II, DL, get(Cramp::SLLI), ScaledRegister)
        .addReg(VL)
        .addImm(ShiftAmount)
        .setMIFlag(Flag);
    BuildMI(MBB, II, DL, get(Cramp::SUB), VL)
        .addReg(ScaledRegister, RegState::Kill)
        .addReg(VL, RegState::Kill)
        .setMIFlag(Flag);
  } else {
    Register N = MRI.createVirtualRegister(&Cramp::GPRRegClass);
    movImm(MBB, II, DL, N, NumOfVReg, Flag);
    if (!STI.hasStdExtM() && !STI.hasStdExtZmmul())
      MF.getFunction().getContext().diagnose(DiagnosticInfoUnsupported{
          MF.getFunction(),
          "M- or Zmmul-extension must be enabled to calculate the vscaled size/"
          "offset."});
    BuildMI(MBB, II, DL, get(Cramp::MUL), VL)
        .addReg(VL, RegState::Kill)
        .addReg(N, RegState::Kill)
        .setMIFlag(Flag);
  }

  return VL;
}

// Returns true if this is the sext.w pattern, addiw rd, rs1, 0.
bool Cramp::isSEXT_W(const MachineInstr &MI) {
  return MI.getOpcode() == Cramp::ADDIW && MI.getOperand(1).isReg() &&
         MI.getOperand(2).isImm() && MI.getOperand(2).getImm() == 0;
}

// Returns true if this is the zext.w pattern, adduw rd, rs1, x0.
bool Cramp::isZEXT_W(const MachineInstr &MI) {
  return MI.getOpcode() == Cramp::ADD_UW && MI.getOperand(1).isReg() &&
         MI.getOperand(2).isReg() && MI.getOperand(2).getReg() == Cramp::X0;
}

// Returns true if this is the zext.b pattern, andi rd, rs1, 255.
bool Cramp::isZEXT_B(const MachineInstr &MI) {
  return MI.getOpcode() == Cramp::ANDI && MI.getOperand(1).isReg() &&
         MI.getOperand(2).isImm() && MI.getOperand(2).getImm() == 255;
}

static bool isRVVWholeLoadStore(unsigned Opcode) {
  switch (Opcode) {
  default:
    return false;
  case Cramp::VS1R_V:
  case Cramp::VS2R_V:
  case Cramp::VS4R_V:
  case Cramp::VS8R_V:
  case Cramp::VL1RE8_V:
  case Cramp::VL2RE8_V:
  case Cramp::VL4RE8_V:
  case Cramp::VL8RE8_V:
  case Cramp::VL1RE16_V:
  case Cramp::VL2RE16_V:
  case Cramp::VL4RE16_V:
  case Cramp::VL8RE16_V:
  case Cramp::VL1RE32_V:
  case Cramp::VL2RE32_V:
  case Cramp::VL4RE32_V:
  case Cramp::VL8RE32_V:
  case Cramp::VL1RE64_V:
  case Cramp::VL2RE64_V:
  case Cramp::VL4RE64_V:
  case Cramp::VL8RE64_V:
    return true;
  }
}

bool Cramp::isRVVSpill(const MachineInstr &MI) {
  // RVV lacks any support for immediate addressing for stack addresses, so be
  // conservative.
  unsigned Opcode = MI.getOpcode();
  if (!CrampVPseudosTable::getPseudoInfo(Opcode) &&
      !isRVVWholeLoadStore(Opcode) && !isRVVSpillForZvlsseg(Opcode))
    return false;
  return true;
}

Optional<std::pair<unsigned, unsigned>>
Cramp::isRVVSpillForZvlsseg(unsigned Opcode) {
  switch (Opcode) {
  default:
    return None;
  case Cramp::PseudoVSPILL2_M1:
  case Cramp::PseudoVRELOAD2_M1:
    return std::make_pair(2u, 1u);
  case Cramp::PseudoVSPILL2_M2:
  case Cramp::PseudoVRELOAD2_M2:
    return std::make_pair(2u, 2u);
  case Cramp::PseudoVSPILL2_M4:
  case Cramp::PseudoVRELOAD2_M4:
    return std::make_pair(2u, 4u);
  case Cramp::PseudoVSPILL3_M1:
  case Cramp::PseudoVRELOAD3_M1:
    return std::make_pair(3u, 1u);
  case Cramp::PseudoVSPILL3_M2:
  case Cramp::PseudoVRELOAD3_M2:
    return std::make_pair(3u, 2u);
  case Cramp::PseudoVSPILL4_M1:
  case Cramp::PseudoVRELOAD4_M1:
    return std::make_pair(4u, 1u);
  case Cramp::PseudoVSPILL4_M2:
  case Cramp::PseudoVRELOAD4_M2:
    return std::make_pair(4u, 2u);
  case Cramp::PseudoVSPILL5_M1:
  case Cramp::PseudoVRELOAD5_M1:
    return std::make_pair(5u, 1u);
  case Cramp::PseudoVSPILL6_M1:
  case Cramp::PseudoVRELOAD6_M1:
    return std::make_pair(6u, 1u);
  case Cramp::PseudoVSPILL7_M1:
  case Cramp::PseudoVRELOAD7_M1:
    return std::make_pair(7u, 1u);
  case Cramp::PseudoVSPILL8_M1:
  case Cramp::PseudoVRELOAD8_M1:
    return std::make_pair(8u, 1u);
  }
}

bool Cramp::isFaultFirstLoad(const MachineInstr &MI) {
  return MI.getNumExplicitDefs() == 2 && MI.modifiesRegister(Cramp::VL) &&
         !MI.isInlineAsm();
}
