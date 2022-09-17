//===-- CrampSubtarget.cpp - Cramp Subtarget Information ------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the Cramp specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "CrampSubtarget.h"
#include "Cramp.h"
#include "CrampCallLowering.h"
#include "CrampFrameLowering.h"
#include "CrampLegalizerInfo.h"
#include "CrampMacroFusion.h"
#include "CrampRegisterBankInfo.h"
#include "CrampTargetMachine.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "cramp-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "CrampGenSubtargetInfo.inc"

static cl::opt<bool> EnableSubRegLiveness("cramp-enable-subreg-liveness",
                                          cl::init(false), cl::Hidden);

static cl::opt<int> RVVVectorBitsMax(
    "cramp-v-vector-bits-max",
    cl::desc("Assume V extension vector registers are at most this big, "
             "with zero meaning no maximum size is assumed."),
    cl::init(0), cl::Hidden);

static cl::opt<int> RVVVectorBitsMin(
    "cramp-v-vector-bits-min",
    cl::desc("Assume V extension vector registers are at least this big, "
             "with zero meaning no minimum size is assumed. A value of -1 "
             "means use Zvl*b extension. This is primarily used to enable "
             "autovectorization with fixed width vectors."),
    cl::init(0), cl::Hidden);

static cl::opt<unsigned> RVVVectorLMULMax(
    "cramp-v-fixed-length-vector-lmul-max",
    cl::desc("The maximum LMUL value to use for fixed length vectors. "
             "Fractional LMUL values are not supported."),
    cl::init(8), cl::Hidden);

static cl::opt<bool> CrampDisableUsingConstantPoolForLargeInts(
    "cramp-disable-using-constant-pool-for-large-ints",
    cl::desc("Disable using constant pool for large integers."),
    cl::init(false), cl::Hidden);

static cl::opt<unsigned> CrampMaxBuildIntsCost(
    "cramp-max-build-ints-cost",
    cl::desc("The maximum cost used for building integers."), cl::init(0),
    cl::Hidden);

void CrampSubtarget::anchor() {}

CrampSubtarget &
CrampSubtarget::initializeSubtargetDependencies(const Triple &TT, StringRef CPU,
                                                StringRef TuneCPU, StringRef FS,
                                                StringRef ABIName) {
  // Determine default and user-specified characteristics
  bool Is64Bit = TT.isArch64Bit();
  if (CPU.empty() || CPU == "generic")
    CPU = Is64Bit ? "generic-cramp64" : "generic-cramp32";

  if (TuneCPU.empty())
    TuneCPU = CPU;

  ParseSubtargetFeatures(CPU, TuneCPU, FS);
  if (Is64Bit) {
    XLenVT = MVT::i64;
    XLen = 64;
  }

  TargetABI = CrampABI::computeTargetABI(TT, getFeatureBits(), ABIName);
  CrampFeatures::validate(TT, getFeatureBits());
  return *this;
}

CrampSubtarget::CrampSubtarget(const Triple &TT, StringRef CPU,
                               StringRef TuneCPU, StringRef FS,
                               StringRef ABIName, const TargetMachine &TM)
    : CrampGenSubtargetInfo(TT, CPU, TuneCPU, FS),
      UserReservedRegister(Cramp::NUM_TARGET_REGS),
      FrameLowering(initializeSubtargetDependencies(TT, CPU, TuneCPU, FS, ABIName)),
      InstrInfo(*this), RegInfo(getHwMode()), TLInfo(TM, *this) {
  CallLoweringInfo.reset(new CrampCallLowering(*getTargetLowering()));
  Legalizer.reset(new CrampLegalizerInfo(*this));

  auto *RBI = new CrampRegisterBankInfo(*getRegisterInfo());
  RegBankInfo.reset(RBI);
  InstSelector.reset(createCrampInstructionSelector(
      *static_cast<const CrampTargetMachine *>(&TM), *this, *RBI));
}

const CallLowering *CrampSubtarget::getCallLowering() const {
  return CallLoweringInfo.get();
}

InstructionSelector *CrampSubtarget::getInstructionSelector() const {
  return InstSelector.get();
}

const LegalizerInfo *CrampSubtarget::getLegalizerInfo() const {
  return Legalizer.get();
}

const RegisterBankInfo *CrampSubtarget::getRegBankInfo() const {
  return RegBankInfo.get();
}

bool CrampSubtarget::useConstantPoolForLargeInts() const {
  return !CrampDisableUsingConstantPoolForLargeInts;
}

unsigned CrampSubtarget::getMaxBuildIntsCost() const {
  // Loading integer from constant pool needs two instructions (the reason why
  // the minimum cost is 2): an address calculation instruction and a load
  // instruction. Usually, address calculation and instructions used for
  // building integers (addi, slli, etc.) can be done in one cycle, so here we
  // set the default cost to (LoadLatency + 1) if no threshold is provided.
  return CrampMaxBuildIntsCost == 0
             ? getSchedModel().LoadLatency + 1
             : std::max<unsigned>(2, CrampMaxBuildIntsCost);
}

unsigned CrampSubtarget::getMaxRVVVectorSizeInBits() const {
  assert(hasVInstructions() &&
         "Tried to get vector length without Zve or V extension support!");
  if (RVVVectorBitsMax == 0)
    return 0;

  // ZvlLen specifies the minimum required vlen. The upper bound provided by
  // cramp-v-vector-bits-max should be no less than it.
  if (RVVVectorBitsMax < (int)ZvlLen)
    report_fatal_error("cramp-v-vector-bits-max specified is lower "
                       "than the Zvl*b limitation");

  // FIXME: Change to >= 32 when VLEN = 32 is supported
  assert(
      RVVVectorBitsMax >= 64 && RVVVectorBitsMax <= 65536 &&
      isPowerOf2_32(RVVVectorBitsMax) &&
      "V or Zve* extension requires vector length to be in the range of 64 to "
      "65536 and a power of 2!");
  assert(RVVVectorBitsMax >= RVVVectorBitsMin &&
         "Minimum V extension vector length should not be larger than its "
         "maximum!");
  unsigned Max = std::max(RVVVectorBitsMin, RVVVectorBitsMax);
  return PowerOf2Floor((Max < 64 || Max > 65536) ? 0 : Max);
}

unsigned CrampSubtarget::getMinRVVVectorSizeInBits() const {
  assert(hasVInstructions() &&
         "Tried to get vector length without Zve or V extension support!");

  if (RVVVectorBitsMin == -1)
    return ZvlLen;

  // ZvlLen specifies the minimum required vlen. The lower bound provided by
  // cramp-v-vector-bits-min should be no less than it.
  if (RVVVectorBitsMin != 0 && RVVVectorBitsMin < (int)ZvlLen)
    report_fatal_error("cramp-v-vector-bits-min specified is lower "
                       "than the Zvl*b limitation");

  // FIXME: Change to >= 32 when VLEN = 32 is supported
  assert(
      (RVVVectorBitsMin == 0 ||
       (RVVVectorBitsMin >= 64 && RVVVectorBitsMin <= 65536 &&
        isPowerOf2_32(RVVVectorBitsMin))) &&
      "V or Zve* extension requires vector length to be in the range of 64 to "
      "65536 and a power of 2!");
  assert((RVVVectorBitsMax >= RVVVectorBitsMin || RVVVectorBitsMax == 0) &&
         "Minimum V extension vector length should not be larger than its "
         "maximum!");
  unsigned Min = RVVVectorBitsMin;
  if (RVVVectorBitsMax != 0)
    Min = std::min(RVVVectorBitsMin, RVVVectorBitsMax);
  return PowerOf2Floor((Min < 64 || Min > 65536) ? 0 : Min);
}

unsigned CrampSubtarget::getMaxLMULForFixedLengthVectors() const {
  assert(hasVInstructions() &&
         "Tried to get vector length without Zve or V extension support!");
  assert(RVVVectorLMULMax <= 8 && isPowerOf2_32(RVVVectorLMULMax) &&
         "V extension requires a LMUL to be at most 8 and a power of 2!");
  return PowerOf2Floor(
      std::max<unsigned>(std::min<unsigned>(RVVVectorLMULMax, 8), 1));
}

bool CrampSubtarget::useRVVForFixedLengthVectors() const {
  return hasVInstructions() && getMinRVVVectorSizeInBits() != 0;
}

bool CrampSubtarget::enableSubRegLiveness() const {
  // FIXME: Enable subregister liveness by default for RVV to better handle
  // LMUL>1 and segment load/store.
  return EnableSubRegLiveness;
}

void CrampSubtarget::getPostRAMutations(
    std::vector<std::unique_ptr<ScheduleDAGMutation>> &Mutations) const {
  Mutations.push_back(createCrampMacroFusionDAGMutation());
}
