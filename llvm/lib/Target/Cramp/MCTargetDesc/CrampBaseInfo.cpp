//===-- CrampBaseInfo.cpp - Top level definitions for Cramp MC ------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone enum definitions for the Cramp target
// useful for the compiler back-end and the MC libraries.
//
//===----------------------------------------------------------------------===//

#include "CrampBaseInfo.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/Triple.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/CrampISAInfo.h"
#include "llvm/Support/TargetParser.h"
#include "llvm/Support/raw_ostream.h"

namespace llvm {

extern const SubtargetFeatureKV CrampFeatureKV[Cramp::NumSubtargetFeatures];

namespace CrampSysReg {
#define GET_SysRegsList_IMPL
#include "CrampGenSearchableTables.inc"
} // namespace CrampSysReg

namespace CrampInsnOpcode {
#define GET_CrampOpcodesList_IMPL
#include "CrampGenSearchableTables.inc"
} // namespace CrampInsnOpcode

namespace CrampABI {
ABI computeTargetABI(const Triple &TT, FeatureBitset FeatureBits,
                     StringRef ABIName) {
  auto TargetABI = getTargetABI(ABIName);
  bool IsRV64 = TT.isArch64Bit();
  bool IsRV32E = FeatureBits[Cramp::FeatureRV32E];

  if (!ABIName.empty() && TargetABI == ABI_Unknown) {
    errs()
        << "'" << ABIName
        << "' is not a recognized ABI for this target (ignoring target-abi)\n";
  } else if (ABIName.startswith("ilp32") && IsRV64) {
    errs() << "32-bit ABIs are not supported for 64-bit targets (ignoring "
              "target-abi)\n";
    TargetABI = ABI_Unknown;
  } else if (ABIName.startswith("lp64") && !IsRV64) {
    errs() << "64-bit ABIs are not supported for 32-bit targets (ignoring "
              "target-abi)\n";
    TargetABI = ABI_Unknown;
  } else if (IsRV32E && TargetABI != ABI_ILP32E && TargetABI != ABI_Unknown) {
    // TODO: move this checking to CrampTargetLowering and CrampAsmParser
    errs()
        << "Only the ilp32e ABI is supported for RV32E (ignoring target-abi)\n";
    TargetABI = ABI_Unknown;
  }

  if (TargetABI != ABI_Unknown)
    return TargetABI;

  // If no explicit ABI is given, try to compute the default ABI.
  auto ISAInfo = CrampFeatures::parseFeatureBits(IsRV64, FeatureBits);
  if (!ISAInfo)
    report_fatal_error(ISAInfo.takeError());
  return getTargetABI((*ISAInfo)->computeDefaultABI());
}

ABI getTargetABI(StringRef ABIName) {
  auto TargetABI = StringSwitch<ABI>(ABIName)
                       .Case("ilp32", ABI_ILP32)
                       .Case("ilp32f", ABI_ILP32F)
                       .Case("ilp32d", ABI_ILP32D)
                       .Case("ilp32e", ABI_ILP32E)
                       .Case("lp64", ABI_LP64)
                       .Case("lp64f", ABI_LP64F)
                       .Case("lp64d", ABI_LP64D)
                       .Default(ABI_Unknown);
  return TargetABI;
}

// To avoid the BP value clobbered by a function call, we need to choose a
// callee saved register to save the value. RV32E only has X8 and X9 as callee
// saved registers and X8 will be used as fp. So we choose X9 as bp.
MCRegister getBPReg() { return Cramp::X9; }

// Returns the register holding shadow call stack pointer.
MCRegister getSCSPReg() { return Cramp::X18; }

} // namespace CrampABI

namespace CrampFeatures {

void validate(const Triple &TT, const FeatureBitset &FeatureBits) {
  if (TT.isArch64Bit() && !FeatureBits[Cramp::Feature64Bit])
    report_fatal_error("RV64 target requires an RV64 CPU");
  if (!TT.isArch64Bit() && FeatureBits[Cramp::Feature64Bit])
    report_fatal_error("RV32 target requires an RV32 CPU");
  if (TT.isArch64Bit() && FeatureBits[Cramp::FeatureRV32E])
    report_fatal_error("RV32E can't be enabled for an RV64 target");
}

llvm::Expected<std::unique_ptr<CrampISAInfo>>
parseFeatureBits(bool IsRV64, const FeatureBitset &FeatureBits) {
  unsigned XLen = IsRV64 ? 64 : 32;
  std::vector<std::string> FeatureVector;
  // Convert FeatureBitset to FeatureVector.
  for (auto Feature : CrampFeatureKV) {
    if (FeatureBits[Feature.Value] &&
        llvm::CrampISAInfo::isSupportedExtensionFeature(Feature.Key))
      FeatureVector.push_back(std::string("+") + Feature.Key);
  }
  return llvm::CrampISAInfo::parseFeatures(XLen, FeatureVector);
}

} // namespace CrampFeatures

// Encode VTYPE into the binary format used by the the VSETVLI instruction which
// is used by our MC layer representation.
//
// Bits | Name       | Description
// -----+------------+------------------------------------------------
// 7    | vma        | Vector mask agnostic
// 6    | vta        | Vector tail agnostic
// 5:3  | vsew[2:0]  | Standard element width (SEW) setting
// 2:0  | vlmul[2:0] | Vector register group multiplier (LMUL) setting
unsigned CrampVType::encodeVTYPE(CrampII::VLMUL VLMUL, unsigned SEW,
                                 bool TailAgnostic, bool MaskAgnostic) {
  assert(isValidSEW(SEW) && "Invalid SEW");
  unsigned VLMULBits = static_cast<unsigned>(VLMUL);
  unsigned VSEWBits = encodeSEW(SEW);
  unsigned VTypeI = (VSEWBits << 3) | (VLMULBits & 0x7);
  if (TailAgnostic)
    VTypeI |= 0x40;
  if (MaskAgnostic)
    VTypeI |= 0x80;

  return VTypeI;
}

std::pair<unsigned, bool> CrampVType::decodeVLMUL(CrampII::VLMUL VLMUL) {
  switch (VLMUL) {
  default:
    llvm_unreachable("Unexpected LMUL value!");
  case CrampII::VLMUL::LMUL_1:
  case CrampII::VLMUL::LMUL_2:
  case CrampII::VLMUL::LMUL_4:
  case CrampII::VLMUL::LMUL_8:
    return std::make_pair(1 << static_cast<unsigned>(VLMUL), false);
  case CrampII::VLMUL::LMUL_F2:
  case CrampII::VLMUL::LMUL_F4:
  case CrampII::VLMUL::LMUL_F8:
    return std::make_pair(1 << (8 - static_cast<unsigned>(VLMUL)), true);
  }
}

void CrampVType::printVType(unsigned VType, raw_ostream &OS) {
  unsigned Sew = getSEW(VType);
  OS << "e" << Sew;

  unsigned LMul;
  bool Fractional;
  std::tie(LMul, Fractional) = decodeVLMUL(getVLMUL(VType));

  if (Fractional)
    OS << ", mf";
  else
    OS << ", m";
  OS << LMul;

  if (isTailAgnostic(VType))
    OS << ", ta";
  else
    OS << ", tu";

  if (isMaskAgnostic(VType))
    OS << ", ma";
  else
    OS << ", mu";
}

} // namespace llvm
