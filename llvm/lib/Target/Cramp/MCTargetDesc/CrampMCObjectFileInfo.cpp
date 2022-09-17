//===-- CrampMCObjectFileInfo.cpp - Cramp object file properties ----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the CrampMCObjectFileInfo properties.
//
//===----------------------------------------------------------------------===//

#include "CrampMCObjectFileInfo.h"
#include "CrampMCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSubtargetInfo.h"

using namespace llvm;

unsigned CrampMCObjectFileInfo::getTextSectionAlignment() const {
  const MCSubtargetInfo *STI = getContext().getSubtargetInfo();
  return STI->hasFeature(Cramp::FeatureStdExtC) ? 2 : 4;
}
