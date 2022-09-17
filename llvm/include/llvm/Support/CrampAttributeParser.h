//===-- CrampAttributeParser.h - Cramp Attribute Parser ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_SUPPORT_CRAMPATTRIBUTEPARSER_H
#define LLVM_SUPPORT_CRAMPATTRIBUTEPARSER_H

#include "llvm/Support/ELFAttributeParser.h"
#include "llvm/Support/CrampAttributes.h"

namespace llvm {
class CrampAttributeParser : public ELFAttributeParser {
  struct DisplayHandler {
    CrampAttrs::AttrType attribute;
    Error (CrampAttributeParser::*routine)(unsigned);
  };
  static const DisplayHandler displayRoutines[];

  Error handler(uint64_t tag, bool &handled) override;

  Error unalignedAccess(unsigned tag);
  Error stackAlign(unsigned tag);

public:
  CrampAttributeParser(ScopedPrinter *sw)
      : ELFAttributeParser(sw, CrampAttrs::getCrampAttributeTags(), "cramp") {}
  CrampAttributeParser()
      : ELFAttributeParser(CrampAttrs::getCrampAttributeTags(), "cramp") {}
};

} // namespace llvm

#endif
