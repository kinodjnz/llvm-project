//=- CrampMachineFunctionInfo.cpp - Cramp machine function info ---*- C++ -*-=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares Cramp-specific per-machine-function information.
//
//===----------------------------------------------------------------------===//

#include "CrampMachineFunctionInfo.h"

using namespace llvm;

yaml::CrampMachineFunctionInfo::CrampMachineFunctionInfo(
    const llvm::CrampMachineFunctionInfo &MFI)
    : VarArgsFrameIndex(MFI.getVarArgsFrameIndex()),
      VarArgsSaveSize(MFI.getVarArgsSaveSize()) {}

MachineFunctionInfo *CrampMachineFunctionInfo::clone(
    BumpPtrAllocator &Allocator, MachineFunction &DestMF,
    const DenseMap<MachineBasicBlock *, MachineBasicBlock *> &Src2DstMBB)
    const {
  return DestMF.cloneInfo<CrampMachineFunctionInfo>(*this);
}

void yaml::CrampMachineFunctionInfo::mappingImpl(yaml::IO &YamlIO) {
  MappingTraits<CrampMachineFunctionInfo>::mapping(YamlIO, *this);
}

void CrampMachineFunctionInfo::initializeBaseYamlFields(
    const yaml::CrampMachineFunctionInfo &YamlMFI) {
  VarArgsFrameIndex = YamlMFI.VarArgsFrameIndex;
  VarArgsSaveSize = YamlMFI.VarArgsSaveSize;
}
