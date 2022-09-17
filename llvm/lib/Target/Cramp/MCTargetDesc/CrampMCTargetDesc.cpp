//===-- CrampMCTargetDesc.cpp - Cramp Target Descriptions -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// This file provides Cramp-specific target descriptions.
///
//===----------------------------------------------------------------------===//

#include "CrampMCTargetDesc.h"
#include "CrampBaseInfo.h"
#include "CrampELFStreamer.h"
#include "CrampInstPrinter.h"
#include "CrampMCAsmInfo.h"
#include "CrampMCObjectFileInfo.h"
#include "CrampTargetStreamer.h"
#include "TargetInfo/CrampTargetInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCInstrAnalysis.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

#define GET_INSTRINFO_MC_DESC
#define ENABLE_INSTR_PREDICATE_VERIFIER
#include "CrampGenInstrInfo.inc"

#define GET_REGINFO_MC_DESC
#include "CrampGenRegisterInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "CrampGenSubtargetInfo.inc"

using namespace llvm;

static MCInstrInfo *createCrampMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitCrampMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createCrampMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitCrampMCRegisterInfo(X, Cramp::X1);
  return X;
}

static MCAsmInfo *createCrampMCAsmInfo(const MCRegisterInfo &MRI,
                                       const Triple &TT,
                                       const MCTargetOptions &Options) {
  MCAsmInfo *MAI = new CrampMCAsmInfo(TT);

  MCRegister SP = MRI.getDwarfRegNum(Cramp::X2, true);
  MCCFIInstruction Inst = MCCFIInstruction::cfiDefCfa(nullptr, SP, 0);
  MAI->addInitialFrameState(Inst);

  return MAI;
}

static MCObjectFileInfo *
createCrampMCObjectFileInfo(MCContext &Ctx, bool PIC,
                            bool LargeCodeModel = false) {
  MCObjectFileInfo *MOFI = new CrampMCObjectFileInfo();
  MOFI->initMCObjectFileInfo(Ctx, PIC, LargeCodeModel);
  return MOFI;
}

static MCSubtargetInfo *createCrampMCSubtargetInfo(const Triple &TT,
                                                   StringRef CPU, StringRef FS) {
  if (CPU.empty() || CPU == "generic")
    CPU = TT.isArch64Bit() ? "generic-cramp64" : "generic-cramp32";

  return createCrampMCSubtargetInfoImpl(TT, CPU, /*TuneCPU*/ CPU, FS);
}

static MCInstPrinter *createCrampMCInstPrinter(const Triple &T,
                                               unsigned SyntaxVariant,
                                               const MCAsmInfo &MAI,
                                               const MCInstrInfo &MII,
                                               const MCRegisterInfo &MRI) {
  return new CrampInstPrinter(MAI, MII, MRI);
}

static MCTargetStreamer *
createCrampObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  const Triple &TT = STI.getTargetTriple();
  if (TT.isOSBinFormatELF())
    return new CrampTargetELFStreamer(S, STI);
  return nullptr;
}

static MCTargetStreamer *createCrampAsmTargetStreamer(MCStreamer &S,
                                                      formatted_raw_ostream &OS,
                                                      MCInstPrinter *InstPrint,
                                                      bool isVerboseAsm) {
  return new CrampTargetAsmStreamer(S, OS);
}

static MCTargetStreamer *createCrampNullTargetStreamer(MCStreamer &S) {
  return new CrampTargetStreamer(S);
}

namespace {

class CrampMCInstrAnalysis : public MCInstrAnalysis {
public:
  explicit CrampMCInstrAnalysis(const MCInstrInfo *Info)
      : MCInstrAnalysis(Info) {}

  bool evaluateBranch(const MCInst &Inst, uint64_t Addr, uint64_t Size,
                      uint64_t &Target) const override {
    if (isConditionalBranch(Inst)) {
      int64_t Imm;
      if (Size == 2)
        Imm = Inst.getOperand(1).getImm();
      else
        Imm = Inst.getOperand(2).getImm();
      Target = Addr + Imm;
      return true;
    }

    if (Inst.getOpcode() == Cramp::C_JAL || Inst.getOpcode() == Cramp::C_J) {
      Target = Addr + Inst.getOperand(0).getImm();
      return true;
    }

    if (Inst.getOpcode() == Cramp::JAL) {
      Target = Addr + Inst.getOperand(1).getImm();
      return true;
    }

    return false;
  }
};

} // end anonymous namespace

static MCInstrAnalysis *createCrampInstrAnalysis(const MCInstrInfo *Info) {
  return new CrampMCInstrAnalysis(Info);
}

namespace {
MCStreamer *createCrampELFStreamer(const Triple &T, MCContext &Context,
                                   std::unique_ptr<MCAsmBackend> &&MAB,
                                   std::unique_ptr<MCObjectWriter> &&MOW,
                                   std::unique_ptr<MCCodeEmitter> &&MCE,
                                   bool RelaxAll) {
  return createCrampELFStreamer(Context, std::move(MAB), std::move(MOW),
                                std::move(MCE), RelaxAll);
}
} // end anonymous namespace

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeCrampTargetMC() {
  for (Target *T : {&getTheCramp32Target(), &getTheCramp64Target()}) {
    TargetRegistry::RegisterMCAsmInfo(*T, createCrampMCAsmInfo);
    TargetRegistry::RegisterMCObjectFileInfo(*T, createCrampMCObjectFileInfo);
    TargetRegistry::RegisterMCInstrInfo(*T, createCrampMCInstrInfo);
    TargetRegistry::RegisterMCRegInfo(*T, createCrampMCRegisterInfo);
    TargetRegistry::RegisterMCAsmBackend(*T, createCrampAsmBackend);
    TargetRegistry::RegisterMCCodeEmitter(*T, createCrampMCCodeEmitter);
    TargetRegistry::RegisterMCInstPrinter(*T, createCrampMCInstPrinter);
    TargetRegistry::RegisterMCSubtargetInfo(*T, createCrampMCSubtargetInfo);
    TargetRegistry::RegisterELFStreamer(*T, createCrampELFStreamer);
    TargetRegistry::RegisterObjectTargetStreamer(
        *T, createCrampObjectTargetStreamer);
    TargetRegistry::RegisterMCInstrAnalysis(*T, createCrampInstrAnalysis);

    // Register the asm target streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createCrampAsmTargetStreamer);
    // Register the null target streamer.
    TargetRegistry::RegisterNullTargetStreamer(*T,
                                               createCrampNullTargetStreamer);
  }
}
