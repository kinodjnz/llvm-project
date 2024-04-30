//===-- RISCVSelectionDAGInfo.cpp - RISCV SelectionDAG Info -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the RISCVSelectionDAGInfo class.
//
//===----------------------------------------------------------------------===//

#include "RISCVTargetMachine.h"
#include "RISCVTargetTransformInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/Support/CommandLine.h"
using namespace llvm;

#define DEBUG_TYPE "riscv-selectiondag-info"

static bool shouldGenerateAlignedMemcpy4(SelectionDAG &DAG,
    ConstantSDNode *ConstantSize, Align Alignment) {

  auto &F = DAG.getMachineFunction().getFunction();

  if (F.hasOptNone())
    return false;
  if (!(ConstantSize && ConstantSize->getZExtValue() > 0 && (ConstantSize->getZExtValue() & 3) == 0))
    return false;
  if (!((Alignment.value() & 3) == 0))
    return false;

  return true;
}

static bool shouldGenerateAlignedMemcpy4Vsize(SelectionDAG &DAG, Align Alignment) {

  auto &F = DAG.getMachineFunction().getFunction();

  if (F.hasOptNone())
    return false;
  if (!((Alignment.value() & 3) == 0))
    return false;

  return true;
}

SDValue RISCVSelectionDAGInfo::EmitTargetCodeForMemcpy(
    SelectionDAG &DAG, const SDLoc &dl, SDValue Chain, SDValue Dst, SDValue Src,
    SDValue Size, Align Alignment, bool isVolatile, bool AlwaysInline,
    MachinePointerInfo DstPtrInfo, MachinePointerInfo SrcPtrInfo) const {
  ConstantSDNode *ConstantSize = dyn_cast<ConstantSDNode>(Size);

  if (!AlwaysInline && shouldGenerateAlignedMemcpy4(DAG, ConstantSize, Alignment))
    return DAG.getNode(RISCVISD::ALIGNED_MEMCPY4, dl, MVT::Other, Chain, Dst, Src,
                       DAG.getMemBasePlusOffset(Src, DAG.getZExtOrTrunc(Size, dl, MVT::i32), dl));
  if (!AlwaysInline && shouldGenerateAlignedMemcpy4Vsize(DAG, Alignment))
    return DAG.getNode(RISCVISD::ALIGNED_MEMCPY4_VSIZE, dl, MVT::Other, Chain, Dst, Src,
                       DAG.getMemBasePlusOffset(Src, DAG.getZExtOrTrunc(Size, dl, MVT::i32), dl));

  return SDValue();
}

static bool shouldGenerateAlignedFixedSmallMemmove(SelectionDAG &DAG, ConstantSDNode *ConstantSize, Align Alignment) {
  auto &F = DAG.getMachineFunction().getFunction();

  if (F.hasOptNone())
    return false;
  if (!(ConstantSize && ConstantSize->getZExtValue() > 0 && ConstantSize->getZExtValue() <= 32))
    return false;
  if (!((Alignment.value() & 3) == 0))
    return false;

  return true;
}

SDValue RISCVSelectionDAGInfo::EmitTargetCodeForMemmove(
    SelectionDAG &DAG, const SDLoc &dl, SDValue Chain, SDValue Dst,
    SDValue Src, SDValue Size, Align Alignment, bool isVolatile,
    MachinePointerInfo DstPtrInfo, MachinePointerInfo SrcPtrInfo) const {
  ConstantSDNode *ConstantSize = dyn_cast<ConstantSDNode>(Size);
  if (shouldGenerateAlignedFixedSmallMemmove(DAG, ConstantSize, Alignment)) {
    return DAG.getNode(RISCVISD::ALIGNED_FIXED_SMALL_MEMMOVE, dl, MVT::Other, Chain, Dst, Src,
                       DAG.getZExtOrTrunc(Size, dl, MVT::i32));
  }

  return SDValue();
}

static bool shouldGenerateAlignedBzero4(SelectionDAG &DAG,
    ConstantSDNode *ConstantSrc, ConstantSDNode *ConstantSize, Align Alignment) {

  auto &F = DAG.getMachineFunction().getFunction();

  if (F.hasOptNone())
    return false;
  if (!(ConstantSrc && ConstantSrc->getZExtValue() == 0))
    return false;
  if (!(ConstantSize && ConstantSize->getZExtValue() > 0 /*&& (ConstantSize->getZExtValue() & 3) == 0*/))
    return false;
  if (!((Alignment.value() & 3) == 0))
    return false;

  return true;
}

SDValue RISCVSelectionDAGInfo::EmitTargetCodeForMemset(
    SelectionDAG &DAG, const SDLoc &dl, SDValue Chain, SDValue Dst, SDValue Src,
    SDValue Size, Align Alignment, bool isVolatile, bool AlwaysInline,
    MachinePointerInfo DstPtrInfo) const {

  ConstantSDNode *ConstantSrc = dyn_cast<ConstantSDNode>(Src);
  ConstantSDNode *ConstantSize = dyn_cast<ConstantSDNode>(Size);

  if (!AlwaysInline && shouldGenerateAlignedBzero4(DAG, ConstantSrc, ConstantSize, Alignment)) {
    SDValue SizeValue = DAG.getZExtOrTrunc(Size, dl, MVT::i32);
    EVT SizeVT = SizeValue.getValueType();
    SDValue TruncatedSizeValue = DAG.getNode(ISD::AND, dl, SizeVT, SizeValue, DAG.getConstant(APInt::getHighBitsSet(SizeVT.getScalarSizeInBits(), SizeVT.getScalarSizeInBits() - 2), dl, SizeVT));
    return DAG.getNode(RISCVISD::ALIGNED_BZERO4, dl, MVT::Other, Chain, Dst,
                       DAG.getMemBasePlusOffset(Dst, TruncatedSizeValue, dl),
                       DAG.getZExtOrTrunc(Size, dl, MVT::i32));
  }

  return SDValue();
}

static bool shouldGenerateAlignedFixedMemcmp(SelectionDAG &DAG, ConstantSDNode *ConstantSize, Align Alignment) {
  auto &F = DAG.getMachineFunction().getFunction();

  if (F.hasOptNone())
    return false;
  if (!(ConstantSize && ConstantSize->getZExtValue() > 0))
    return false;
  // if (!((Alignment.value() & 3) == 0))
  //   return false;

  return true;
}

std::pair<SDValue, SDValue>
RISCVSelectionDAGInfo::EmitTargetCodeForMemcmp(SelectionDAG &DAG, const SDLoc &dl, SDValue Chain,
    SDValue Ptr1, SDValue Ptr2, SDValue Size,
    MachinePointerInfo Op1PtrInfo,
    MachinePointerInfo Op2PtrInfo,
    Align Alignment) const {
  ConstantSDNode *ConstantSize = dyn_cast<ConstantSDNode>(Size);

  if (shouldGenerateAlignedFixedMemcmp(DAG, ConstantSize, Alignment)) {
    SDVTList VTs = DAG.getVTList(MVT::i32, MVT::Other);
    SDValue Ret = DAG.getNode(RISCVISD::ALIGNED_FIXED_MEMCMP, dl, VTs, Chain, Ptr1, Ptr2, Ptr2,
                       DAG.getZExtOrTrunc(Size, dl, MVT::i32));
    Chain = Ret.getValue(1);
    return std::make_pair(Ret, Chain);
  }

  return std::make_pair(SDValue(), SDValue());
}
