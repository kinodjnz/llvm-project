//===-- CrampRegisterInfo.cpp - Cramp Register Information ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Cramp implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "CrampRegisterInfo.h"
#include "Cramp.h"
#include "CrampMachineFunctionInfo.h"
#include "CrampSubtarget.h"
#include "llvm/BinaryFormat/Dwarf.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/Support/ErrorHandling.h"

#define GET_REGINFO_TARGET_DESC
#include "CrampGenRegisterInfo.inc"

using namespace llvm;

static_assert(Cramp::X1 == Cramp::X0 + 1, "Register list not consecutive");
static_assert(Cramp::X31 == Cramp::X0 + 31, "Register list not consecutive");
static_assert(Cramp::F1_H == Cramp::F0_H + 1, "Register list not consecutive");
static_assert(Cramp::F31_H == Cramp::F0_H + 31,
              "Register list not consecutive");
static_assert(Cramp::F1_F == Cramp::F0_F + 1, "Register list not consecutive");
static_assert(Cramp::F31_F == Cramp::F0_F + 31,
              "Register list not consecutive");
static_assert(Cramp::F1_D == Cramp::F0_D + 1, "Register list not consecutive");
static_assert(Cramp::F31_D == Cramp::F0_D + 31,
              "Register list not consecutive");
static_assert(Cramp::V1 == Cramp::V0 + 1, "Register list not consecutive");
static_assert(Cramp::V31 == Cramp::V0 + 31, "Register list not consecutive");

CrampRegisterInfo::CrampRegisterInfo(unsigned HwMode)
    : CrampGenRegisterInfo(Cramp::X1, /*DwarfFlavour*/0, /*EHFlavor*/0,
                           /*PC*/0, HwMode) {}

const MCPhysReg *
CrampRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  auto &Subtarget = MF->getSubtarget<CrampSubtarget>();
  if (MF->getFunction().getCallingConv() == CallingConv::GHC)
    return CSR_NoRegs_SaveList;
  if (MF->getFunction().hasFnAttribute("interrupt")) {
    if (Subtarget.hasStdExtD())
      return CSR_XLEN_F64_Interrupt_SaveList;
    if (Subtarget.hasStdExtF())
      return CSR_XLEN_F32_Interrupt_SaveList;
    return CSR_Interrupt_SaveList;
  }

  switch (Subtarget.getTargetABI()) {
  default:
    llvm_unreachable("Unrecognized ABI");
  case CrampABI::ABI_ILP32:
  case CrampABI::ABI_LP64:
    return CSR_ILP32_LP64_SaveList;
  case CrampABI::ABI_ILP32F:
  case CrampABI::ABI_LP64F:
    return CSR_ILP32F_LP64F_SaveList;
  case CrampABI::ABI_ILP32D:
  case CrampABI::ABI_LP64D:
    return CSR_ILP32D_LP64D_SaveList;
  }
}

BitVector CrampRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  const CrampFrameLowering *TFI = getFrameLowering(MF);
  BitVector Reserved(getNumRegs());

  // Mark any registers requested to be reserved as such
  for (size_t Reg = 0; Reg < getNumRegs(); Reg++) {
    if (MF.getSubtarget<CrampSubtarget>().isRegisterReservedByUser(Reg))
      markSuperRegs(Reserved, Reg);
  }

  // Use markSuperRegs to ensure any register aliases are also reserved
  markSuperRegs(Reserved, Cramp::X0); // zero
  markSuperRegs(Reserved, Cramp::X2); // sp
  markSuperRegs(Reserved, Cramp::X3); // gp
  markSuperRegs(Reserved, Cramp::X4); // tp
  if (TFI->hasFP(MF))
    markSuperRegs(Reserved, Cramp::X8); // fp
  // Reserve the base register if we need to realign the stack and allocate
  // variable-sized objects at runtime.
  if (TFI->hasBP(MF))
    markSuperRegs(Reserved, CrampABI::getBPReg()); // bp

  // V registers for code generation. We handle them manually.
  markSuperRegs(Reserved, Cramp::VL);
  markSuperRegs(Reserved, Cramp::VTYPE);
  markSuperRegs(Reserved, Cramp::VXSAT);
  markSuperRegs(Reserved, Cramp::VXRM);
  markSuperRegs(Reserved, Cramp::VLENB); // vlenb (constant)

  // Floating point environment registers.
  markSuperRegs(Reserved, Cramp::FRM);
  markSuperRegs(Reserved, Cramp::FFLAGS);

  assert(checkAllSuperRegsMarked(Reserved));
  return Reserved;
}

bool CrampRegisterInfo::isAsmClobberable(const MachineFunction &MF,
                                         MCRegister PhysReg) const {
  return !MF.getSubtarget<CrampSubtarget>().isRegisterReservedByUser(PhysReg);
}

bool CrampRegisterInfo::isConstantPhysReg(MCRegister PhysReg) const {
  return PhysReg == Cramp::X0 || PhysReg == Cramp::VLENB;
}

const uint32_t *CrampRegisterInfo::getNoPreservedMask() const {
  return CSR_NoRegs_RegMask;
}

// Frame indexes representing locations of CSRs which are given a fixed location
// by save/restore libcalls.
static const std::pair<unsigned, int> FixedCSRFIMap[] = {
  {/*ra*/  Cramp::X1,   -1},
  {/*s0*/  Cramp::X8,   -2},
  {/*s1*/  Cramp::X9,   -3},
  {/*s2*/  Cramp::X18,  -4},
  {/*s3*/  Cramp::X19,  -5},
  {/*s4*/  Cramp::X20,  -6},
  {/*s5*/  Cramp::X21,  -7},
  {/*s6*/  Cramp::X22,  -8},
  {/*s7*/  Cramp::X23,  -9},
  {/*s8*/  Cramp::X24,  -10},
  {/*s9*/  Cramp::X25,  -11},
  {/*s10*/ Cramp::X26,  -12},
  {/*s11*/ Cramp::X27,  -13}
};

bool CrampRegisterInfo::hasReservedSpillSlot(const MachineFunction &MF,
                                             Register Reg,
                                             int &FrameIdx) const {
  const auto *RVFI = MF.getInfo<CrampMachineFunctionInfo>();
  if (!RVFI->useSaveRestoreLibCalls(MF))
    return false;

  const auto *FII =
      llvm::find_if(FixedCSRFIMap, [&](auto P) { return P.first == Reg; });
  if (FII == std::end(FixedCSRFIMap))
    return false;

  FrameIdx = FII->second;
  return true;
}

void CrampRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                            int SPAdj, unsigned FIOperandNum,
                                            RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected non-zero SPAdj value");

  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const CrampInstrInfo *TII = MF.getSubtarget<CrampSubtarget>().getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  Register FrameReg;
  StackOffset Offset =
      getFrameLowering(MF)->getFrameIndexReference(MF, FrameIndex, FrameReg);
  bool IsRVVSpill = Cramp::isRVVSpill(MI);
  if (!IsRVVSpill)
    Offset += StackOffset::getFixed(MI.getOperand(FIOperandNum + 1).getImm());

  if (!isInt<32>(Offset.getFixed())) {
    report_fatal_error(
        "Frame offsets outside of the signed 32-bit range not supported");
  }

  MachineBasicBlock &MBB = *MI.getParent();
  bool FrameRegIsKill = false;

  // If required, pre-compute the scalable factor amount which will be used in
  // later offset computation. Since this sequence requires up to two scratch
  // registers -- after which one is made free -- this grants us better
  // scavenging of scratch registers as only up to two are live at one time,
  // rather than three.
  Register ScalableFactorRegister;
  unsigned ScalableAdjOpc = Cramp::ADD;
  if (Offset.getScalable()) {
    int64_t ScalableValue = Offset.getScalable();
    if (ScalableValue < 0) {
      ScalableValue = -ScalableValue;
      ScalableAdjOpc = Cramp::SUB;
    }
    // 1. Get vlenb && multiply vlen with the number of vector registers.
    ScalableFactorRegister =
        TII->getVLENFactoredAmount(MF, MBB, II, DL, ScalableValue);
  }

  if (!isInt<12>(Offset.getFixed())) {
    // The offset won't fit in an immediate, so use a scratch register instead
    // Modify Offset and FrameReg appropriately
    Register ScratchReg = MRI.createVirtualRegister(&Cramp::GPRRegClass);
    TII->movImm(MBB, II, DL, ScratchReg, Offset.getFixed());
    if (MI.getOpcode() == Cramp::ADDI && !Offset.getScalable()) {
      BuildMI(MBB, II, DL, TII->get(Cramp::ADD), MI.getOperand(0).getReg())
        .addReg(FrameReg)
        .addReg(ScratchReg, RegState::Kill);
      MI.eraseFromParent();
      return;
    }
    BuildMI(MBB, II, DL, TII->get(Cramp::ADD), ScratchReg)
        .addReg(FrameReg)
        .addReg(ScratchReg, RegState::Kill);
    Offset = StackOffset::get(0, Offset.getScalable());
    FrameReg = ScratchReg;
    FrameRegIsKill = true;
  }

  if (!Offset.getScalable()) {
    // Offset = (fixed offset, 0)
    MI.getOperand(FIOperandNum)
        .ChangeToRegister(FrameReg, false, false, FrameRegIsKill);
    if (!IsRVVSpill)
      MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset.getFixed());
    else {
      if (Offset.getFixed()) {
        Register ScratchReg = MRI.createVirtualRegister(&Cramp::GPRRegClass);
        BuildMI(MBB, II, DL, TII->get(Cramp::ADDI), ScratchReg)
          .addReg(FrameReg, getKillRegState(FrameRegIsKill))
          .addImm(Offset.getFixed());
        MI.getOperand(FIOperandNum)
          .ChangeToRegister(ScratchReg, false, false, true);
      }
    }
  } else {
    // Offset = (fixed offset, scalable offset)
    // Step 1, the scalable offset, has already been computed.
    assert(ScalableFactorRegister &&
           "Expected pre-computation of scalable factor in earlier step");

    // 2. Calculate address: FrameReg + result of multiply
    if (MI.getOpcode() == Cramp::ADDI && !Offset.getFixed()) {
      BuildMI(MBB, II, DL, TII->get(ScalableAdjOpc), MI.getOperand(0).getReg())
          .addReg(FrameReg, getKillRegState(FrameRegIsKill))
          .addReg(ScalableFactorRegister, RegState::Kill);
      MI.eraseFromParent();
      return;
    }
    Register VL = MRI.createVirtualRegister(&Cramp::GPRRegClass);
    BuildMI(MBB, II, DL, TII->get(ScalableAdjOpc), VL)
        .addReg(FrameReg, getKillRegState(FrameRegIsKill))
        .addReg(ScalableFactorRegister, RegState::Kill);

    if (IsRVVSpill && Offset.getFixed()) {
      // Scalable load/store has no immediate argument. We need to add the
      // fixed part into the load/store base address.
      BuildMI(MBB, II, DL, TII->get(Cramp::ADDI), VL)
          .addReg(VL)
          .addImm(Offset.getFixed());
    }

    // 3. Replace address register with calculated address register
    MI.getOperand(FIOperandNum).ChangeToRegister(VL, false, false, true);
    if (!IsRVVSpill)
      MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset.getFixed());
  }

  auto ZvlssegInfo = Cramp::isRVVSpillForZvlsseg(MI.getOpcode());
  if (ZvlssegInfo) {
    Register VL = MRI.createVirtualRegister(&Cramp::GPRRegClass);
    BuildMI(MBB, II, DL, TII->get(Cramp::PseudoReadVLENB), VL);
    uint32_t ShiftAmount = Log2_32(ZvlssegInfo->second);
    if (ShiftAmount != 0)
      BuildMI(MBB, II, DL, TII->get(Cramp::SLLI), VL)
          .addReg(VL)
          .addImm(ShiftAmount);
    // The last argument of pseudo spilling opcode for zvlsseg is the length of
    // one element of zvlsseg types. For example, for vint32m2x2_t, it will be
    // the length of vint32m2_t.
    MI.getOperand(FIOperandNum + 1).ChangeToRegister(VL, /*isDef=*/false);
  }
}

Register CrampRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? Cramp::X8 : Cramp::X2;
}

const uint32_t *
CrampRegisterInfo::getCallPreservedMask(const MachineFunction & MF,
                                        CallingConv::ID CC) const {
  auto &Subtarget = MF.getSubtarget<CrampSubtarget>();

  if (CC == CallingConv::GHC)
    return CSR_NoRegs_RegMask;
  switch (Subtarget.getTargetABI()) {
  default:
    llvm_unreachable("Unrecognized ABI");
  case CrampABI::ABI_ILP32:
  case CrampABI::ABI_LP64:
    return CSR_ILP32_LP64_RegMask;
  case CrampABI::ABI_ILP32F:
  case CrampABI::ABI_LP64F:
    return CSR_ILP32F_LP64F_RegMask;
  case CrampABI::ABI_ILP32D:
  case CrampABI::ABI_LP64D:
    return CSR_ILP32D_LP64D_RegMask;
  }
}

const TargetRegisterClass *
CrampRegisterInfo::getLargestLegalSuperClass(const TargetRegisterClass *RC,
                                             const MachineFunction &) const {
  if (RC == &Cramp::VMV0RegClass)
    return &Cramp::VRRegClass;
  return RC;
}

void CrampRegisterInfo::getOffsetOpcodes(const StackOffset &Offset,
                                         SmallVectorImpl<uint64_t> &Ops) const {
  // VLENB is the length of a vector register in bytes. We use <vscale x 8 x i8>
  // to represent one vector register. The dwarf offset is
  // VLENB * scalable_offset / 8.
  assert(Offset.getScalable() % 8 == 0 && "Invalid frame offset");

  // Add fixed-sized offset using existing DIExpression interface.
  DIExpression::appendOffset(Ops, Offset.getFixed());

  unsigned VLENB = getDwarfRegNum(Cramp::VLENB, true);
  int64_t VLENBSized = Offset.getScalable() / 8;
  if (VLENBSized > 0) {
    Ops.push_back(dwarf::DW_OP_constu);
    Ops.push_back(VLENBSized);
    Ops.append({dwarf::DW_OP_bregx, VLENB, 0ULL});
    Ops.push_back(dwarf::DW_OP_mul);
    Ops.push_back(dwarf::DW_OP_plus);
  } else if (VLENBSized < 0) {
    Ops.push_back(dwarf::DW_OP_constu);
    Ops.push_back(-VLENBSized);
    Ops.append({dwarf::DW_OP_bregx, VLENB, 0ULL});
    Ops.push_back(dwarf::DW_OP_mul);
    Ops.push_back(dwarf::DW_OP_minus);
  }
}

unsigned
CrampRegisterInfo::getRegisterCostTableIndex(const MachineFunction &MF) const {
  return MF.getSubtarget<CrampSubtarget>().hasStdExtC() ? 1 : 0;
}
