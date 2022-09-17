//===-- CrampELFStreamer.cpp - Cramp ELF Target Streamer Methods ----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides Cramp specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "CrampELFStreamer.h"
#include "CrampAsmBackend.h"
#include "CrampBaseInfo.h"
#include "CrampMCTargetDesc.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/LEB128.h"
#include "llvm/Support/CrampAttributes.h"

using namespace llvm;

// This part is for ELF object output.
CrampTargetELFStreamer::CrampTargetELFStreamer(MCStreamer &S,
                                               const MCSubtargetInfo &STI)
    : CrampTargetStreamer(S), CurrentVendor("cramp"), STI(STI) {
  MCAssembler &MCA = getStreamer().getAssembler();
  const FeatureBitset &Features = STI.getFeatureBits();
  auto &MAB = static_cast<CrampAsmBackend &>(MCA.getBackend());
  setTargetABI(CrampABI::computeTargetABI(STI.getTargetTriple(), Features,
                                          MAB.getTargetOptions().getABIName()));
}

MCELFStreamer &CrampTargetELFStreamer::getStreamer() {
  return static_cast<MCELFStreamer &>(Streamer);
}

void CrampTargetELFStreamer::emitDirectiveOptionPush() {}
void CrampTargetELFStreamer::emitDirectiveOptionPop() {}
void CrampTargetELFStreamer::emitDirectiveOptionPIC() {}
void CrampTargetELFStreamer::emitDirectiveOptionNoPIC() {}
void CrampTargetELFStreamer::emitDirectiveOptionRVC() {}
void CrampTargetELFStreamer::emitDirectiveOptionNoRVC() {}
void CrampTargetELFStreamer::emitDirectiveOptionRelax() {}
void CrampTargetELFStreamer::emitDirectiveOptionNoRelax() {}

void CrampTargetELFStreamer::emitAttribute(unsigned Attribute, unsigned Value) {
  setAttributeItem(Attribute, Value, /*OverwriteExisting=*/true);
}

void CrampTargetELFStreamer::emitTextAttribute(unsigned Attribute,
                                               StringRef String) {
  setAttributeItem(Attribute, String, /*OverwriteExisting=*/true);
}

void CrampTargetELFStreamer::emitIntTextAttribute(unsigned Attribute,
                                                  unsigned IntValue,
                                                  StringRef StringValue) {
  setAttributeItems(Attribute, IntValue, StringValue,
                    /*OverwriteExisting=*/true);
}

void CrampTargetELFStreamer::finishAttributeSection() {
  if (Contents.empty())
    return;

  if (AttributeSection) {
    Streamer.switchSection(AttributeSection);
  } else {
    MCAssembler &MCA = getStreamer().getAssembler();
    AttributeSection = MCA.getContext().getELFSection(
        ".cramp.attributes", ELF::SHT_CRAMP_ATTRIBUTES, 0);
    Streamer.switchSection(AttributeSection);

    Streamer.emitInt8(ELFAttrs::Format_Version);
  }

  // Vendor size + Vendor name + '\0'
  const size_t VendorHeaderSize = 4 + CurrentVendor.size() + 1;

  // Tag + Tag Size
  const size_t TagHeaderSize = 1 + 4;

  const size_t ContentsSize = calculateContentSize();

  Streamer.emitInt32(VendorHeaderSize + TagHeaderSize + ContentsSize);
  Streamer.emitBytes(CurrentVendor);
  Streamer.emitInt8(0); // '\0'

  Streamer.emitInt8(ELFAttrs::File);
  Streamer.emitInt32(TagHeaderSize + ContentsSize);

  // Size should have been accounted for already, now
  // emit each field as its type (ULEB or String).
  for (AttributeItem item : Contents) {
    Streamer.emitULEB128IntValue(item.Tag);
    switch (item.Type) {
    default:
      llvm_unreachable("Invalid attribute type");
    case AttributeType::Numeric:
      Streamer.emitULEB128IntValue(item.IntValue);
      break;
    case AttributeType::Text:
      Streamer.emitBytes(item.StringValue);
      Streamer.emitInt8(0); // '\0'
      break;
    case AttributeType::NumericAndText:
      Streamer.emitULEB128IntValue(item.IntValue);
      Streamer.emitBytes(item.StringValue);
      Streamer.emitInt8(0); // '\0'
      break;
    }
  }

  Contents.clear();
}

size_t CrampTargetELFStreamer::calculateContentSize() const {
  size_t Result = 0;
  for (AttributeItem item : Contents) {
    switch (item.Type) {
    case AttributeType::Hidden:
      break;
    case AttributeType::Numeric:
      Result += getULEB128Size(item.Tag);
      Result += getULEB128Size(item.IntValue);
      break;
    case AttributeType::Text:
      Result += getULEB128Size(item.Tag);
      Result += item.StringValue.size() + 1; // string + '\0'
      break;
    case AttributeType::NumericAndText:
      Result += getULEB128Size(item.Tag);
      Result += getULEB128Size(item.IntValue);
      Result += item.StringValue.size() + 1; // string + '\0';
      break;
    }
  }
  return Result;
}

void CrampTargetELFStreamer::finish() {
  CrampTargetStreamer::finish();
  MCAssembler &MCA = getStreamer().getAssembler();
  const FeatureBitset &Features = STI.getFeatureBits();
  CrampABI::ABI ABI = getTargetABI();

  unsigned EFlags = MCA.getELFHeaderEFlags();

  if (Features[Cramp::FeatureStdExtC])
    EFlags |= ELF::EF_CRAMP_RVC;

  switch (ABI) {
  case CrampABI::ABI_ILP32:
  case CrampABI::ABI_LP64:
    break;
  case CrampABI::ABI_ILP32F:
  case CrampABI::ABI_LP64F:
    EFlags |= ELF::EF_CRAMP_FLOAT_ABI_SINGLE;
    break;
  case CrampABI::ABI_ILP32D:
  case CrampABI::ABI_LP64D:
    EFlags |= ELF::EF_CRAMP_FLOAT_ABI_DOUBLE;
    break;
  case CrampABI::ABI_ILP32E:
    EFlags |= ELF::EF_CRAMP_RVE;
    break;
  case CrampABI::ABI_Unknown:
    llvm_unreachable("Improperly initialised target ABI");
  }

  MCA.setELFHeaderEFlags(EFlags);
}

void CrampTargetELFStreamer::reset() {
  AttributeSection = nullptr;
  Contents.clear();
}

namespace {
class CrampELFStreamer : public MCELFStreamer {
  static std::pair<unsigned, unsigned> getRelocPairForSize(unsigned Size) {
    switch (Size) {
    default:
      llvm_unreachable("unsupported fixup size");
    case 1:
      return std::make_pair(Cramp::fixup_cramp_add_8, Cramp::fixup_cramp_sub_8);
    case 2:
      return std::make_pair(Cramp::fixup_cramp_add_16,
                            Cramp::fixup_cramp_sub_16);
    case 4:
      return std::make_pair(Cramp::fixup_cramp_add_32,
                            Cramp::fixup_cramp_sub_32);
    case 8:
      return std::make_pair(Cramp::fixup_cramp_add_64,
                            Cramp::fixup_cramp_sub_64);
    }
  }

  static bool requiresFixups(MCContext &C, const MCExpr *Value,
                             const MCExpr *&LHS, const MCExpr *&RHS) {
    auto IsMetadataOrEHFrameSection = [](const MCSection &S) -> bool {
      // Additionally check .apple_names/.apple_types. They are fixed-size and
      // do not need fixups. llvm-dwarfdump --apple-names does not process
      // R_Cramp_{ADD,SUB}32 in them.
      return S.getKind().isMetadata() || S.getName() == ".eh_frame" ||
             S.getName() == ".apple_names" || S.getName() == ".apple_types";
    };

    const auto *MBE = dyn_cast<MCBinaryExpr>(Value);
    if (MBE == nullptr)
      return false;

    MCValue E;
    if (!Value->evaluateAsRelocatable(E, nullptr, nullptr))
      return false;
    if (E.getSymA() == nullptr || E.getSymB() == nullptr)
      return false;

    const auto &A = E.getSymA()->getSymbol();
    const auto &B = E.getSymB()->getSymbol();

    LHS =
        MCBinaryExpr::create(MCBinaryExpr::Add, MCSymbolRefExpr::create(&A, C),
                             MCConstantExpr::create(E.getConstant(), C), C);
    RHS = E.getSymB();

    // TODO: when available, R_Cramp_n_PCREL should be preferred.

    // Avoid pairwise relocations for symbolic difference in debug and .eh_frame
    if (A.isInSection())
      return !IsMetadataOrEHFrameSection(A.getSection());
    if (B.isInSection())
      return !IsMetadataOrEHFrameSection(B.getSection());
    // as well as for absolute symbols.
    return !A.getName().empty() || !B.getName().empty();
  }

  void reset() override {
    static_cast<CrampTargetStreamer *>(getTargetStreamer())->reset();
    MCELFStreamer::reset();
  }

public:
  CrampELFStreamer(MCContext &C, std::unique_ptr<MCAsmBackend> MAB,
                   std::unique_ptr<MCObjectWriter> MOW,
                   std::unique_ptr<MCCodeEmitter> MCE)
      : MCELFStreamer(C, std::move(MAB), std::move(MOW), std::move(MCE)) {}

  void emitValueImpl(const MCExpr *Value, unsigned Size, SMLoc Loc) override {
    const MCExpr *A, *B;
    if (!requiresFixups(getContext(), Value, A, B))
      return MCELFStreamer::emitValueImpl(Value, Size, Loc);

    MCStreamer::emitValueImpl(Value, Size, Loc);

    MCDataFragment *DF = getOrCreateDataFragment();
    flushPendingLabels(DF, DF->getContents().size());
    MCDwarfLineEntry::make(this, getCurrentSectionOnly());

    unsigned Add, Sub;
    std::tie(Add, Sub) = getRelocPairForSize(Size);

    DF->getFixups().push_back(MCFixup::create(
        DF->getContents().size(), A, static_cast<MCFixupKind>(Add), Loc));
    DF->getFixups().push_back(MCFixup::create(
        DF->getContents().size(), B, static_cast<MCFixupKind>(Sub), Loc));

    DF->getContents().resize(DF->getContents().size() + Size, 0);
  }
};
} // namespace

namespace llvm {
MCELFStreamer *createCrampELFStreamer(MCContext &C,
                                      std::unique_ptr<MCAsmBackend> MAB,
                                      std::unique_ptr<MCObjectWriter> MOW,
                                      std::unique_ptr<MCCodeEmitter> MCE,
                                      bool RelaxAll) {
  CrampELFStreamer *S =
      new CrampELFStreamer(C, std::move(MAB), std::move(MOW), std::move(MCE));
  S->getAssembler().setRelaxAll(RelaxAll);
  return S;
}
} // namespace llvm
