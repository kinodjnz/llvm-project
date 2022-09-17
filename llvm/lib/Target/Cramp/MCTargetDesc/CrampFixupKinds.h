//===-- CrampFixupKinds.h - Cramp Specific Fixup Entries --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_Cramp_MCTARGETDESC_CrampFIXUPKINDS_H
#define LLVM_LIB_TARGET_Cramp_MCTARGETDESC_CrampFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

#undef Cramp

namespace llvm {
namespace Cramp {
enum Fixups {
  // 20-bit fixup corresponding to %hi(foo) for instructions like lui
  fixup_cramp_hi20 = FirstTargetFixupKind,
  // 12-bit fixup corresponding to %lo(foo) for instructions like addi
  fixup_cramp_lo12_i,
  // 12-bit fixup corresponding to %lo(foo) for the S-type store instructions
  fixup_cramp_lo12_s,
  // 20-bit fixup corresponding to %pcrel_hi(foo) for instructions like auipc
  fixup_cramp_pcrel_hi20,
  // 12-bit fixup corresponding to %pcrel_lo(foo) for instructions like addi
  fixup_cramp_pcrel_lo12_i,
  // 12-bit fixup corresponding to %pcrel_lo(foo) for the S-type store
  // instructions
  fixup_cramp_pcrel_lo12_s,
  // 20-bit fixup corresponding to %got_pcrel_hi(foo) for instructions like
  // auipc
  fixup_cramp_got_hi20,
  // 20-bit fixup corresponding to %tprel_hi(foo) for instructions like lui
  fixup_cramp_tprel_hi20,
  // 12-bit fixup corresponding to %tprel_lo(foo) for instructions like addi
  fixup_cramp_tprel_lo12_i,
  // 12-bit fixup corresponding to %tprel_lo(foo) for the S-type store
  // instructions
  fixup_cramp_tprel_lo12_s,
  // Fixup corresponding to %tprel_add(foo) for PseudoAddTPRel, used as a linker
  // hint
  fixup_cramp_tprel_add,
  // 20-bit fixup corresponding to %tls_ie_pcrel_hi(foo) for instructions like
  // auipc
  fixup_cramp_tls_got_hi20,
  // 20-bit fixup corresponding to %tls_gd_pcrel_hi(foo) for instructions like
  // auipc
  fixup_cramp_tls_gd_hi20,
  // 20-bit fixup for symbol references in the jal instruction
  fixup_cramp_jal,
  // 12-bit fixup for symbol references in the branch instructions
  fixup_cramp_branch,
  // 11-bit fixup for symbol references in the compressed jump instruction
  fixup_cramp_rvc_jump,
  // 8-bit fixup for symbol references in the compressed branch instruction
  fixup_cramp_rvc_branch,
  // Fixup representing a legacy no-pic function call attached to the auipc
  // instruction in a pair composed of adjacent auipc+jalr instructions.
  fixup_cramp_call,
  // Fixup representing a function call attached to the auipc instruction in a
  // pair composed of adjacent auipc+jalr instructions.
  fixup_cramp_call_plt,
  // Used to generate an R_Cramp_RELAX relocation, which indicates the linker
  // may relax the instruction pair.
  fixup_cramp_relax,
  // Used to generate an R_Cramp_ALIGN relocation, which indicates the linker
  // should fixup the alignment after linker relaxation.
  fixup_cramp_align,
  // 8-bit fixup corresponding to R_Cramp_SET8 for local label assignment.
  fixup_cramp_set_8,
  // 8-bit fixup corresponding to R_Cramp_ADD8 for 8-bit symbolic difference
  // paired relocations.
  fixup_cramp_add_8,
  // 8-bit fixup corresponding to R_Cramp_SUB8 for 8-bit symbolic difference
  // paired relocations.
  fixup_cramp_sub_8,
  // 16-bit fixup corresponding to R_Cramp_SET16 for local label assignment.
  fixup_cramp_set_16,
  // 16-bit fixup corresponding to R_Cramp_ADD16 for 16-bit symbolic difference
  // paired reloctions.
  fixup_cramp_add_16,
  // 16-bit fixup corresponding to R_Cramp_SUB16 for 16-bit symbolic difference
  // paired reloctions.
  fixup_cramp_sub_16,
  // 32-bit fixup corresponding to R_Cramp_SET32 for local label assignment.
  fixup_cramp_set_32,
  // 32-bit fixup corresponding to R_Cramp_ADD32 for 32-bit symbolic difference
  // paired relocations.
  fixup_cramp_add_32,
  // 32-bit fixup corresponding to R_Cramp_SUB32 for 32-bit symbolic difference
  // paired relocations.
  fixup_cramp_sub_32,
  // 64-bit fixup corresponding to R_Cramp_ADD64 for 64-bit symbolic difference
  // paired relocations.
  fixup_cramp_add_64,
  // 64-bit fixup corresponding to R_Cramp_SUB64 for 64-bit symbolic difference
  // paired relocations.
  fixup_cramp_sub_64,
  // 6-bit fixup corresponding to R_Cramp_SET6 for local label assignment in
  // DWARF CFA.
  fixup_cramp_set_6b,
  // 6-bit fixup corresponding to R_Cramp_SUB6 for local label assignment in
  // DWARF CFA.
  fixup_cramp_sub_6b,

  // Used as a sentinel, must be the last
  fixup_cramp_invalid,
  NumTargetFixupKinds = fixup_cramp_invalid - FirstTargetFixupKind
};
} // end namespace Cramp
} // end namespace llvm

#endif
