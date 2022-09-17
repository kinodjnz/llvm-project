//===-- CrampTargetInfo.cpp - Cramp Target Implementation -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/CrampTargetInfo.h"
#include "llvm/MC/TargetRegistry.h"
using namespace llvm;

Target &llvm::getTheCramp32Target() {
  static Target TheCramp32Target;
  return TheCramp32Target;
}

Target &llvm::getTheCramp64Target() {
  static Target TheCramp64Target;
  return TheCramp64Target;
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeCrampTargetInfo() {
  RegisterTarget<Triple::cramp32, /*HasJIT=*/true> X(
      getTheCramp32Target(), "cramp32", "32-bit Cramp", "Cramp");
  RegisterTarget<Triple::cramp64, /*HasJIT=*/true> Y(
      getTheCramp64Target(), "cramp64", "64-bit Cramp", "Cramp");
}
