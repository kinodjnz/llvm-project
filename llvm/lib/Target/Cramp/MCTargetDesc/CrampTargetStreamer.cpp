//===-- CrampTargetStreamer.cpp - Cramp Target Streamer Methods -----------===//
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

#include "CrampTargetStreamer.h"
#include "CrampBaseInfo.h"
#include "CrampMCTargetDesc.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/CrampAttributes.h"
#include "llvm/Support/CrampISAInfo.h"

using namespace llvm;

CrampTargetStreamer::CrampTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

void CrampTargetStreamer::finish() { finishAttributeSection(); }
void CrampTargetStreamer::reset() {}

void CrampTargetStreamer::emitDirectiveOptionPush() {}
void CrampTargetStreamer::emitDirectiveOptionPop() {}
void CrampTargetStreamer::emitDirectiveOptionPIC() {}
void CrampTargetStreamer::emitDirectiveOptionNoPIC() {}
void CrampTargetStreamer::emitDirectiveOptionRVC() {}
void CrampTargetStreamer::emitDirectiveOptionNoRVC() {}
void CrampTargetStreamer::emitDirectiveOptionRelax() {}
void CrampTargetStreamer::emitDirectiveOptionNoRelax() {}
void CrampTargetStreamer::emitAttribute(unsigned Attribute, unsigned Value) {}
void CrampTargetStreamer::finishAttributeSection() {}
void CrampTargetStreamer::emitTextAttribute(unsigned Attribute,
                                            StringRef String) {}
void CrampTargetStreamer::emitIntTextAttribute(unsigned Attribute,
                                               unsigned IntValue,
                                               StringRef StringValue) {}
void CrampTargetStreamer::setTargetABI(CrampABI::ABI ABI) {
  assert(ABI != CrampABI::ABI_Unknown && "Improperly initialized target ABI");
  TargetABI = ABI;
}

void CrampTargetStreamer::emitTargetAttributes(const MCSubtargetInfo &STI) {
  if (STI.hasFeature(Cramp::FeatureRV32E))
    emitAttribute(CrampAttrs::STACK_ALIGN, CrampAttrs::ALIGN_4);
  else
    emitAttribute(CrampAttrs::STACK_ALIGN, CrampAttrs::ALIGN_16);

  auto ParseResult = CrampFeatures::parseFeatureBits(
      STI.hasFeature(Cramp::Feature64Bit), STI.getFeatureBits());
  if (!ParseResult) {
    report_fatal_error(ParseResult.takeError());
  } else {
    auto &ISAInfo = *ParseResult;
    emitTextAttribute(CrampAttrs::ARCH, ISAInfo->toString());
  }
}

// This part is for ascii assembly output
CrampTargetAsmStreamer::CrampTargetAsmStreamer(MCStreamer &S,
                                               formatted_raw_ostream &OS)
    : CrampTargetStreamer(S), OS(OS) {}

void CrampTargetAsmStreamer::emitDirectiveOptionPush() {
  OS << "\t.option\tpush\n";
}

void CrampTargetAsmStreamer::emitDirectiveOptionPop() {
  OS << "\t.option\tpop\n";
}

void CrampTargetAsmStreamer::emitDirectiveOptionPIC() {
  OS << "\t.option\tpic\n";
}

void CrampTargetAsmStreamer::emitDirectiveOptionNoPIC() {
  OS << "\t.option\tnopic\n";
}

void CrampTargetAsmStreamer::emitDirectiveOptionRVC() {
  OS << "\t.option\trvc\n";
}

void CrampTargetAsmStreamer::emitDirectiveOptionNoRVC() {
  OS << "\t.option\tnorvc\n";
}

void CrampTargetAsmStreamer::emitDirectiveOptionRelax() {
  OS << "\t.option\trelax\n";
}

void CrampTargetAsmStreamer::emitDirectiveOptionNoRelax() {
  OS << "\t.option\tnorelax\n";
}

void CrampTargetAsmStreamer::emitAttribute(unsigned Attribute, unsigned Value) {
  OS << "\t.attribute\t" << Attribute << ", " << Twine(Value) << "\n";
}

void CrampTargetAsmStreamer::emitTextAttribute(unsigned Attribute,
                                               StringRef String) {
  OS << "\t.attribute\t" << Attribute << ", \"" << String << "\"\n";
}

void CrampTargetAsmStreamer::emitIntTextAttribute(unsigned Attribute,
                                                  unsigned IntValue,
                                                  StringRef StringValue) {}

void CrampTargetAsmStreamer::finishAttributeSection() {}
