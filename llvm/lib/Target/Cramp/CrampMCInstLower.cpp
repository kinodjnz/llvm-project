//===-- CrampMCInstLower.cpp - Convert Cramp MachineInstr to an MCInst ------=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower Cramp MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "Cramp.h"
#include "CrampSubtarget.h"
#include "MCTargetDesc/CrampMCExpr.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

static MCOperand lowerSymbolOperand(const MachineOperand &MO, MCSymbol *Sym,
                                    const AsmPrinter &AP) {
  MCContext &Ctx = AP.OutContext;
  CrampMCExpr::VariantKind Kind;

  switch (MO.getTargetFlags()) {
  default:
    llvm_unreachable("Unknown target flag on GV operand");
  case CrampII::MO_None:
    Kind = CrampMCExpr::VK_Cramp_None;
    break;
  case CrampII::MO_CALL:
    Kind = CrampMCExpr::VK_Cramp_CALL;
    break;
  case CrampII::MO_PLT:
    Kind = CrampMCExpr::VK_Cramp_CALL_PLT;
    break;
  case CrampII::MO_LO:
    Kind = CrampMCExpr::VK_Cramp_LO;
    break;
  case CrampII::MO_HI:
    Kind = CrampMCExpr::VK_Cramp_HI;
    break;
  case CrampII::MO_PCREL_LO:
    Kind = CrampMCExpr::VK_Cramp_PCREL_LO;
    break;
  case CrampII::MO_PCREL_HI:
    Kind = CrampMCExpr::VK_Cramp_PCREL_HI;
    break;
  case CrampII::MO_GOT_HI:
    Kind = CrampMCExpr::VK_Cramp_GOT_HI;
    break;
  case CrampII::MO_TPREL_LO:
    Kind = CrampMCExpr::VK_Cramp_TPREL_LO;
    break;
  case CrampII::MO_TPREL_HI:
    Kind = CrampMCExpr::VK_Cramp_TPREL_HI;
    break;
  case CrampII::MO_TPREL_ADD:
    Kind = CrampMCExpr::VK_Cramp_TPREL_ADD;
    break;
  case CrampII::MO_TLS_GOT_HI:
    Kind = CrampMCExpr::VK_Cramp_TLS_GOT_HI;
    break;
  case CrampII::MO_TLS_GD_HI:
    Kind = CrampMCExpr::VK_Cramp_TLS_GD_HI;
    break;
  }

  const MCExpr *ME =
      MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, Ctx);

  if (!MO.isJTI() && !MO.isMBB() && MO.getOffset())
    ME = MCBinaryExpr::createAdd(
        ME, MCConstantExpr::create(MO.getOffset(), Ctx), Ctx);

  if (Kind != CrampMCExpr::VK_Cramp_None)
    ME = CrampMCExpr::create(ME, Kind, Ctx);
  return MCOperand::createExpr(ME);
}

bool llvm::lowerCrampMachineOperandToMCOperand(const MachineOperand &MO,
                                               MCOperand &MCOp,
                                               const AsmPrinter &AP) {
  switch (MO.getType()) {
  default:
    report_fatal_error("LowerCrampMachineInstrToMCInst: unknown operand type");
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit())
      return false;
    MCOp = MCOperand::createReg(MO.getReg());
    break;
  case MachineOperand::MO_RegisterMask:
    // Regmasks are like implicit defs.
    return false;
  case MachineOperand::MO_Immediate:
    MCOp = MCOperand::createImm(MO.getImm());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MCOp = lowerSymbolOperand(MO, MO.getMBB()->getSymbol(), AP);
    break;
  case MachineOperand::MO_GlobalAddress:
    MCOp = lowerSymbolOperand(MO, AP.getSymbolPreferLocal(*MO.getGlobal()), AP);
    break;
  case MachineOperand::MO_BlockAddress:
    MCOp = lowerSymbolOperand(
        MO, AP.GetBlockAddressSymbol(MO.getBlockAddress()), AP);
    break;
  case MachineOperand::MO_ExternalSymbol:
    MCOp = lowerSymbolOperand(
        MO, AP.GetExternalSymbolSymbol(MO.getSymbolName()), AP);
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    MCOp = lowerSymbolOperand(MO, AP.GetCPISymbol(MO.getIndex()), AP);
    break;
  case MachineOperand::MO_JumpTableIndex:
    MCOp = lowerSymbolOperand(MO, AP.GetJTISymbol(MO.getIndex()), AP);
    break;
  }
  return true;
}

static bool lowerCrampVMachineInstrToMCInst(const MachineInstr *MI,
                                            MCInst &OutMI) {
  const CrampVPseudosTable::PseudoInfo *RVV =
      CrampVPseudosTable::getPseudoInfo(MI->getOpcode());
  if (!RVV)
    return false;

  OutMI.setOpcode(RVV->BaseInstr);

  const MachineBasicBlock *MBB = MI->getParent();
  assert(MBB && "MI expected to be in a basic block");
  const MachineFunction *MF = MBB->getParent();
  assert(MF && "MBB expected to be in a machine function");

  const TargetRegisterInfo *TRI =
      MF->getSubtarget<CrampSubtarget>().getRegisterInfo();

  assert(TRI && "TargetRegisterInfo expected");

  uint64_t TSFlags = MI->getDesc().TSFlags;
  unsigned NumOps = MI->getNumExplicitOperands();

  // Skip policy, VL and SEW operands which are the last operands if present.
  if (CrampII::hasVecPolicyOp(TSFlags))
    --NumOps;
  if (CrampII::hasVLOp(TSFlags))
    --NumOps;
  if (CrampII::hasSEWOp(TSFlags))
    --NumOps;

  bool hasVLOutput = Cramp::isFaultFirstLoad(*MI);
  for (unsigned OpNo = 0; OpNo != NumOps; ++OpNo) {
    const MachineOperand &MO = MI->getOperand(OpNo);
    // Skip vl ouput. It should be the second output.
    if (hasVLOutput && OpNo == 1)
      continue;

    // Skip merge op. It should be the first operand after the result.
    if (CrampII::hasMergeOp(TSFlags) && OpNo == 1U + hasVLOutput) {
      assert(MI->getNumExplicitDefs() == 1U + hasVLOutput);
      continue;
    }

    MCOperand MCOp;
    switch (MO.getType()) {
    default:
      llvm_unreachable("Unknown operand type");
    case MachineOperand::MO_Register: {
      Register Reg = MO.getReg();

      if (Cramp::VRM2RegClass.contains(Reg) ||
          Cramp::VRM4RegClass.contains(Reg) ||
          Cramp::VRM8RegClass.contains(Reg)) {
        Reg = TRI->getSubReg(Reg, Cramp::sub_vrm1_0);
        assert(Reg && "Subregister does not exist");
      } else if (Cramp::FPR16RegClass.contains(Reg)) {
        Reg = TRI->getMatchingSuperReg(Reg, Cramp::sub_16, &Cramp::FPR32RegClass);
        assert(Reg && "Subregister does not exist");
      } else if (Cramp::FPR64RegClass.contains(Reg)) {
        Reg = TRI->getSubReg(Reg, Cramp::sub_32);
        assert(Reg && "Superregister does not exist");
      }

      MCOp = MCOperand::createReg(Reg);
      break;
    }
    case MachineOperand::MO_Immediate:
      MCOp = MCOperand::createImm(MO.getImm());
      break;
    }
    OutMI.addOperand(MCOp);
  }

  // Unmasked pseudo instructions need to append dummy mask operand to
  // V instructions. All V instructions are modeled as the masked version.
  if (CrampII::hasDummyMaskOp(TSFlags))
    OutMI.addOperand(MCOperand::createReg(Cramp::NoRegister));

  return true;
}

bool llvm::lowerCrampMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                          AsmPrinter &AP) {
  if (lowerCrampVMachineInstrToMCInst(MI, OutMI))
    return false;

  OutMI.setOpcode(MI->getOpcode());

  for (const MachineOperand &MO : MI->operands()) {
    MCOperand MCOp;
    if (lowerCrampMachineOperandToMCOperand(MO, MCOp, AP))
      OutMI.addOperand(MCOp);
  }

  switch (OutMI.getOpcode()) {
  case TargetOpcode::PATCHABLE_FUNCTION_ENTER: {
    const Function &F = MI->getParent()->getParent()->getFunction();
    if (F.hasFnAttribute("patchable-function-entry")) {
      unsigned Num;
      if (F.getFnAttribute("patchable-function-entry")
              .getValueAsString()
              .getAsInteger(10, Num))
        return false;
      AP.emitNops(Num);
      return true;
    }
    break;
  }
  case Cramp::PseudoReadVLENB:
    OutMI.setOpcode(Cramp::CSRRS);
    OutMI.addOperand(MCOperand::createImm(
        CrampSysReg::lookupSysRegByName("VLENB")->Encoding));
    OutMI.addOperand(MCOperand::createReg(Cramp::X0));
    break;
  case Cramp::PseudoReadVL:
    OutMI.setOpcode(Cramp::CSRRS);
    OutMI.addOperand(
        MCOperand::createImm(CrampSysReg::lookupSysRegByName("VL")->Encoding));
    OutMI.addOperand(MCOperand::createReg(Cramp::X0));
    break;
  }
  return false;
}
