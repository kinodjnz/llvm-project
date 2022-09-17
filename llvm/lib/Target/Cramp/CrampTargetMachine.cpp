//===-- CrampTargetMachine.cpp - Define TargetMachine for Cramp -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Implements the info about Cramp target spec.
//
//===----------------------------------------------------------------------===//

#include "CrampTargetMachine.h"
#include "MCTargetDesc/CrampBaseInfo.h"
#include "Cramp.h"
#include "CrampMachineFunctionInfo.h"
#include "CrampMacroFusion.h"
#include "CrampTargetObjectFile.h"
#include "CrampTargetTransformInfo.h"
#include "TargetInfo/CrampTargetInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/MIRParser/MIParser.h"
#include "llvm/CodeGen/MIRYamlMapping.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/InitializePasses.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Transforms/IPO.h"
using namespace llvm;

static cl::opt<bool> EnableRedundantCopyElimination(
    "cramp-enable-copyelim",
    cl::desc("Enable the redundant copy elimination pass"), cl::init(true),
    cl::Hidden);

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeCrampTarget() {
  RegisterTargetMachine<CrampTargetMachine> X(getTheCramp32Target());
  RegisterTargetMachine<CrampTargetMachine> Y(getTheCramp64Target());
  auto *PR = PassRegistry::getPassRegistry();
  initializeGlobalISel(*PR);
  initializeCrampMakeCompressibleOptPass(*PR);
  initializeCrampGatherScatterLoweringPass(*PR);
  initializeCrampCodeGenPreparePass(*PR);
  initializeCrampMergeBaseOffsetOptPass(*PR);
  initializeCrampSExtWRemovalPass(*PR);
  initializeCrampExpandPseudoPass(*PR);
  initializeCrampInsertVSETVLIPass(*PR);
}

static StringRef computeDataLayout(const Triple &TT) {
  if (TT.isArch64Bit())
    return "e-m:e-p:64:64-i64:64-i128:128-n64-S128";
  assert(TT.isArch32Bit() && "only RV32 and RV64 are currently supported");
  return "e-m:e-p:32:32-i64:64-n32-S128";
}

static Reloc::Model getEffectiveRelocModel(const Triple &TT,
                                           Optional<Reloc::Model> RM) {
  return RM.value_or(Reloc::Static);
}

CrampTargetMachine::CrampTargetMachine(const Target &T, const Triple &TT,
                                       StringRef CPU, StringRef FS,
                                       const TargetOptions &Options,
                                       Optional<Reloc::Model> RM,
                                       Optional<CodeModel::Model> CM,
                                       CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, computeDataLayout(TT), TT, CPU, FS, Options,
                        getEffectiveRelocModel(TT, RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      TLOF(std::make_unique<CrampELFTargetObjectFile>()) {
  initAsmInfo();

  // RISC-V supports the MachineOutliner.
  setMachineOutliner(true);
  setSupportsDefaultOutlining(true);
}

const CrampSubtarget *
CrampTargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute TuneAttr = F.getFnAttribute("tune-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  std::string CPU =
      CPUAttr.isValid() ? CPUAttr.getValueAsString().str() : TargetCPU;
  std::string TuneCPU =
      TuneAttr.isValid() ? TuneAttr.getValueAsString().str() : CPU;
  std::string FS =
      FSAttr.isValid() ? FSAttr.getValueAsString().str() : TargetFS;
  std::string Key = CPU + TuneCPU + FS;
  auto &I = SubtargetMap[Key];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    auto ABIName = Options.MCOptions.getABIName();
    if (const MDString *ModuleTargetABI = dyn_cast_or_null<MDString>(
            F.getParent()->getModuleFlag("target-abi"))) {
      auto TargetABI = CrampABI::getTargetABI(ABIName);
      if (TargetABI != CrampABI::ABI_Unknown &&
          ModuleTargetABI->getString() != ABIName) {
        report_fatal_error("-target-abi option != target-abi module flag");
      }
      ABIName = ModuleTargetABI->getString();
    }
    I = std::make_unique<CrampSubtarget>(TargetTriple, CPU, TuneCPU, FS, ABIName, *this);
  }
  return I.get();
}

TargetTransformInfo
CrampTargetMachine::getTargetTransformInfo(const Function &F) const {
  return TargetTransformInfo(CrampTTIImpl(this, F));
}

// A RISC-V hart has a single byte-addressable address space of 2^XLEN bytes
// for all memory accesses, so it is reasonable to assume that an
// implementation has no-op address space casts. If an implementation makes a
// change to this, they can override it here.
bool CrampTargetMachine::isNoopAddrSpaceCast(unsigned SrcAS,
                                             unsigned DstAS) const {
  return true;
}

namespace {
class CrampPassConfig : public TargetPassConfig {
public:
  CrampPassConfig(CrampTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  CrampTargetMachine &getCrampTargetMachine() const {
    return getTM<CrampTargetMachine>();
  }

  ScheduleDAGInstrs *
  createMachineScheduler(MachineSchedContext *C) const override {
    const CrampSubtarget &ST = C->MF->getSubtarget<CrampSubtarget>();
    if (ST.hasMacroFusion()) {
      ScheduleDAGMILive *DAG = createGenericSchedLive(C);
      DAG->addMutation(createCrampMacroFusionDAGMutation());
      return DAG;
    }
    return nullptr;
  }

  ScheduleDAGInstrs *
  createPostMachineScheduler(MachineSchedContext *C) const override {
    const CrampSubtarget &ST = C->MF->getSubtarget<CrampSubtarget>();
    if (ST.hasMacroFusion()) {
      ScheduleDAGMI *DAG = createGenericSchedPostRA(C);
      DAG->addMutation(createCrampMacroFusionDAGMutation());
      return DAG;
    }
    return nullptr;
  }

  void addIRPasses() override;
  bool addPreISel() override;
  bool addInstSelector() override;
  bool addIRTranslator() override;
  bool addLegalizeMachineIR() override;
  bool addRegBankSelect() override;
  bool addGlobalInstructionSelect() override;
  void addPreEmitPass() override;
  void addPreEmitPass2() override;
  void addPreSched2() override;
  void addMachineSSAOptimization() override;
  void addPreRegAlloc() override;
  void addPostRegAlloc() override;
};
} // namespace

TargetPassConfig *CrampTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new CrampPassConfig(*this, PM);
}

void CrampPassConfig::addIRPasses() {
  addPass(createAtomicExpandPass());

  if (getOptLevel() != CodeGenOpt::None)
    addPass(createCrampGatherScatterLoweringPass());

  if (getOptLevel() != CodeGenOpt::None)
    addPass(createCrampCodeGenPreparePass());

  TargetPassConfig::addIRPasses();
}

bool CrampPassConfig::addPreISel() {
  if (TM->getOptLevel() != CodeGenOpt::None) {
    // Add a barrier before instruction selection so that we will not get
    // deleted block address after enabling default outlining. See D99707 for
    // more details.
    addPass(createBarrierNoopPass());
  }
  return false;
}

bool CrampPassConfig::addInstSelector() {
  addPass(createCrampISelDag(getCrampTargetMachine(), getOptLevel()));

  return false;
}

bool CrampPassConfig::addIRTranslator() {
  addPass(new IRTranslator(getOptLevel()));
  return false;
}

bool CrampPassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  return false;
}

bool CrampPassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

bool CrampPassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect(getOptLevel()));
  return false;
}

void CrampPassConfig::addPreSched2() {}

void CrampPassConfig::addPreEmitPass() {
  addPass(&BranchRelaxationPassID);
  addPass(createCrampMakeCompressibleOptPass());
}

void CrampPassConfig::addPreEmitPass2() {
  addPass(createCrampExpandPseudoPass());
  // Schedule the expansion of AMOs at the last possible moment, avoiding the
  // possibility for other passes to break the requirements for forward
  // progress in the LR/SC block.
  addPass(createCrampExpandAtomicPseudoPass());
}

void CrampPassConfig::addMachineSSAOptimization() {
  TargetPassConfig::addMachineSSAOptimization();

  if (TM->getTargetTriple().getArch() == Triple::cramp64)
    addPass(createCrampSExtWRemovalPass());
}

void CrampPassConfig::addPreRegAlloc() {
  if (TM->getOptLevel() != CodeGenOpt::None)
    addPass(createCrampMergeBaseOffsetOptPass());
  addPass(createCrampInsertVSETVLIPass());
}

void CrampPassConfig::addPostRegAlloc() {
  if (TM->getOptLevel() != CodeGenOpt::None && EnableRedundantCopyElimination)
    addPass(createCrampRedundantCopyEliminationPass());
}

yaml::MachineFunctionInfo *
CrampTargetMachine::createDefaultFuncInfoYAML() const {
  return new yaml::CrampMachineFunctionInfo();
}

yaml::MachineFunctionInfo *
CrampTargetMachine::convertFuncInfoToYAML(const MachineFunction &MF) const {
  const auto *MFI = MF.getInfo<CrampMachineFunctionInfo>();
  return new yaml::CrampMachineFunctionInfo(*MFI);
}

bool CrampTargetMachine::parseMachineFunctionInfo(
    const yaml::MachineFunctionInfo &MFI, PerFunctionMIParsingState &PFS,
    SMDiagnostic &Error, SMRange &SourceRange) const {
  const auto &YamlMFI =
      static_cast<const yaml::CrampMachineFunctionInfo &>(MFI);
  PFS.MF.getInfo<CrampMachineFunctionInfo>()->initializeBaseYamlFields(YamlMFI);
  return false;
}
