//===-- CrampELFObjectWriter.cpp - Cramp ELF Writer -----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/CrampFixupKinds.h"
#include "MCTargetDesc/CrampMCExpr.h"
#include "MCTargetDesc/CrampMCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

namespace {
class CrampELFObjectWriter : public MCELFObjectTargetWriter {
public:
  CrampELFObjectWriter(uint8_t OSABI, bool Is64Bit);

  ~CrampELFObjectWriter() override;

  // Return true if the given relocation must be with a symbol rather than
  // section plus offset.
  bool needsRelocateWithSymbol(const MCSymbol &Sym,
                               unsigned Type) const override {
    // TODO: this is very conservative, update once RISC-V psABI requirements
    //       are clarified.
    return true;
  }

protected:
  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
};
}

CrampELFObjectWriter::CrampELFObjectWriter(uint8_t OSABI, bool Is64Bit)
    : MCELFObjectTargetWriter(Is64Bit, OSABI, ELF::EM_CRAMP,
                              /*HasRelocationAddend*/ true) {}

CrampELFObjectWriter::~CrampELFObjectWriter() = default;

unsigned CrampELFObjectWriter::getRelocType(MCContext &Ctx,
                                            const MCValue &Target,
                                            const MCFixup &Fixup,
                                            bool IsPCRel) const {
  const MCExpr *Expr = Fixup.getValue();
  // Determine the type of the relocation
  unsigned Kind = Fixup.getTargetKind();
  if (Kind >= FirstLiteralRelocationKind)
    return Kind - FirstLiteralRelocationKind;
  if (IsPCRel) {
    switch (Kind) {
    default:
      Ctx.reportError(Fixup.getLoc(), "Unsupported relocation type");
      return ELF::R_CRAMP_NONE;
    case FK_Data_4:
    case FK_PCRel_4:
      return ELF::R_CRAMP_32_PCREL;
    case Cramp::fixup_cramp_pcrel_hi20:
      return ELF::R_CRAMP_PCREL_HI20;
    case Cramp::fixup_cramp_pcrel_lo12_i:
      return ELF::R_CRAMP_PCREL_LO12_I;
    case Cramp::fixup_cramp_pcrel_lo12_s:
      return ELF::R_CRAMP_PCREL_LO12_S;
    case Cramp::fixup_cramp_got_hi20:
      return ELF::R_CRAMP_GOT_HI20;
    case Cramp::fixup_cramp_tls_got_hi20:
      return ELF::R_CRAMP_TLS_GOT_HI20;
    case Cramp::fixup_cramp_tls_gd_hi20:
      return ELF::R_CRAMP_TLS_GD_HI20;
    case Cramp::fixup_cramp_jal:
      return ELF::R_CRAMP_JAL;
    case Cramp::fixup_cramp_branch:
      return ELF::R_CRAMP_BRANCH;
    case Cramp::fixup_cramp_rvc_jump:
      return ELF::R_CRAMP_RVC_JUMP;
    case Cramp::fixup_cramp_rvc_branch:
      return ELF::R_CRAMP_RVC_BRANCH;
    case Cramp::fixup_cramp_call:
      return ELF::R_CRAMP_CALL;
    case Cramp::fixup_cramp_call_plt:
      return ELF::R_CRAMP_CALL_PLT;
    case Cramp::fixup_cramp_add_8:
      return ELF::R_CRAMP_ADD8;
    case Cramp::fixup_cramp_sub_8:
      return ELF::R_CRAMP_SUB8;
    case Cramp::fixup_cramp_add_16:
      return ELF::R_CRAMP_ADD16;
    case Cramp::fixup_cramp_sub_16:
      return ELF::R_CRAMP_SUB16;
    case Cramp::fixup_cramp_add_32:
      return ELF::R_CRAMP_ADD32;
    case Cramp::fixup_cramp_sub_32:
      return ELF::R_CRAMP_SUB32;
    case Cramp::fixup_cramp_add_64:
      return ELF::R_CRAMP_ADD64;
    case Cramp::fixup_cramp_sub_64:
      return ELF::R_CRAMP_SUB64;
    }
  }

  switch (Kind) {
  default:
    Ctx.reportError(Fixup.getLoc(), "Unsupported relocation type");
    return ELF::R_CRAMP_NONE;
  case FK_Data_1:
    Ctx.reportError(Fixup.getLoc(), "1-byte data relocations not supported");
    return ELF::R_CRAMP_NONE;
  case FK_Data_2:
    Ctx.reportError(Fixup.getLoc(), "2-byte data relocations not supported");
    return ELF::R_CRAMP_NONE;
  case FK_Data_4:
    if (Expr->getKind() == MCExpr::Target &&
        cast<CrampMCExpr>(Expr)->getKind() == CrampMCExpr::VK_Cramp_32_PCREL)
      return ELF::R_CRAMP_32_PCREL;
    return ELF::R_CRAMP_32;
  case FK_Data_8:
    return ELF::R_CRAMP_64;
  case Cramp::fixup_cramp_hi20:
    return ELF::R_CRAMP_HI20;
  case Cramp::fixup_cramp_lo12_i:
    return ELF::R_CRAMP_LO12_I;
  case Cramp::fixup_cramp_lo12_s:
    return ELF::R_CRAMP_LO12_S;
  case Cramp::fixup_cramp_tprel_hi20:
    return ELF::R_CRAMP_TPREL_HI20;
  case Cramp::fixup_cramp_tprel_lo12_i:
    return ELF::R_CRAMP_TPREL_LO12_I;
  case Cramp::fixup_cramp_tprel_lo12_s:
    return ELF::R_CRAMP_TPREL_LO12_S;
  case Cramp::fixup_cramp_tprel_add:
    return ELF::R_CRAMP_TPREL_ADD;
  case Cramp::fixup_cramp_relax:
    return ELF::R_CRAMP_RELAX;
  case Cramp::fixup_cramp_align:
    return ELF::R_CRAMP_ALIGN;
  case Cramp::fixup_cramp_set_6b:
    return ELF::R_CRAMP_SET6;
  case Cramp::fixup_cramp_sub_6b:
    return ELF::R_CRAMP_SUB6;
  case Cramp::fixup_cramp_add_8:
    return ELF::R_CRAMP_ADD8;
  case Cramp::fixup_cramp_set_8:
    return ELF::R_CRAMP_SET8;
  case Cramp::fixup_cramp_sub_8:
    return ELF::R_CRAMP_SUB8;
  case Cramp::fixup_cramp_set_16:
    return ELF::R_CRAMP_SET16;
  case Cramp::fixup_cramp_add_16:
    return ELF::R_CRAMP_ADD16;
  case Cramp::fixup_cramp_sub_16:
    return ELF::R_CRAMP_SUB16;
  case Cramp::fixup_cramp_set_32:
    return ELF::R_CRAMP_SET32;
  case Cramp::fixup_cramp_add_32:
    return ELF::R_CRAMP_ADD32;
  case Cramp::fixup_cramp_sub_32:
    return ELF::R_CRAMP_SUB32;
  case Cramp::fixup_cramp_add_64:
    return ELF::R_CRAMP_ADD64;
  case Cramp::fixup_cramp_sub_64:
    return ELF::R_CRAMP_SUB64;
  }
}

std::unique_ptr<MCObjectTargetWriter>
llvm::createCrampELFObjectWriter(uint8_t OSABI, bool Is64Bit) {
  return std::make_unique<CrampELFObjectWriter>(OSABI, Is64Bit);
}
