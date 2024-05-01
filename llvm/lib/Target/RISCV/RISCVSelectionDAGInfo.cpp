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

static bool shouldGenerateAlignedFixedMemcpy(SelectionDAG &DAG, ConstantSDNode *ConstantSize, Align Alignment) {

  auto &F = DAG.getMachineFunction().getFunction();

  if (F.hasOptNone())
    return false;
  if (!(ConstantSize && ConstantSize->getZExtValue() >= 16))
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

  if (!AlwaysInline && shouldGenerateAlignedFixedMemcpy(DAG, ConstantSize, Alignment)) {
    SDValue SizeValue = DAG.getZExtOrTrunc(Size, dl, MVT::i32);
    EVT SizeVT = SizeValue.getValueType();
    SDValue SizeMask = DAG.getConstant(APInt::getHighBitsSet(SizeVT.getScalarSizeInBits(), SizeVT.getScalarSizeInBits() - 4), dl, SizeVT);
    SDValue TruncatedSize = DAG.getNode(ISD::AND, dl, SizeVT, SizeValue, SizeMask);
    SDValue Ops[] = {
      Chain, Dst, Src,
      DAG.getMemBasePlusOffset(Src, TruncatedSize, dl),
      DAG.getZExtOrTrunc(Size, dl, MVT::i32),
      DAG.getConstant(4, dl, MVT::i32)
    };
    return DAG.getNode(RISCVISD::ALIGNED_FIXED_MEMCPY, dl, MVT::Other, Ops);
  }

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
    SDValue SizeMask = DAG.getConstant(APInt::getHighBitsSet(SizeVT.getScalarSizeInBits(), SizeVT.getScalarSizeInBits() - 2), dl, SizeVT);
    SDValue TruncatedSizeValue = DAG.getNode(ISD::AND, dl, SizeVT, SizeValue, SizeMask);
    return DAG.getNode(RISCVISD::ALIGNED_BZERO4, dl, MVT::Other, Chain, Dst,
                       DAG.getMemBasePlusOffset(Dst, TruncatedSizeValue, dl),
                       DAG.getZExtOrTrunc(Size, dl, MVT::i32));
  }

  return SDValue();
}

std::pair<SDValue, SDValue>
RISCVSelectionDAGInfo::EmitTargetCodeForMemcmp(SelectionDAG &DAG, const SDLoc &dl, SDValue Chain,
    SDValue Ptr1, SDValue Ptr2, SDValue Size,
    MachinePointerInfo Op1PtrInfo,
    MachinePointerInfo Op2PtrInfo,
    bool IsOnlyUsedInZeroEqualityComparison) const {
  auto &F = DAG.getMachineFunction().getFunction();
  ConstantSDNode *ConstantSize = dyn_cast<ConstantSDNode>(Size);
  if (!F.hasOptNone() && ConstantSize) {
    SDVTList VTs = DAG.getVTList(MVT::i32, MVT::i32, MVT::Other);
    SDValue SizeValue = DAG.getZExtOrTrunc(Size, dl, MVT::i32);
    EVT SizeVT = SizeValue.getValueType();
    SDValue SizeMask = DAG.getConstant(APInt::getHighBitsSet(SizeVT.getScalarSizeInBits(), SizeVT.getScalarSizeInBits() - 2), dl, SizeVT);
    SDValue WordTruncatedSize = DAG.getNode(ISD::AND, dl, SizeVT, SizeValue, SizeMask);
    SDValue Data1 = DAG.getNode(RISCVISD::ALIGNED_FIXED_MEMCMP, dl, VTs, Chain, Ptr1, Ptr2,
                      DAG.getMemBasePlusOffset(Ptr2, WordTruncatedSize, dl),
                      DAG.getZExtOrTrunc(Size, dl, MVT::i32));
    SDValue Data2 = Data1.getValue(1);
    Chain = Data1.getValue(2);
    SDValue Cmp;
    if (IsOnlyUsedInZeroEqualityComparison) {
      Cmp = DAG.getNode(ISD::SETCC, dl, MVT::i32, Data1, Data2, DAG.getCondCode(ISD::CondCode::SETNE));
    } else {
      Data1 = DAG.getNode(ISD::BSWAP, dl, MVT::i32, Data1);
      Data2 = DAG.getNode(ISD::BSWAP, dl, MVT::i32, Data2);
      Cmp = DAG.getNode(ISD::SUB, dl, MVT::i32, Data1, Data2);
    }
    return std::make_pair(Cmp, Chain);
  }

    // SDValue CmpPtr = DAG.CreateStackTemporary(MVT::i32);
    // SDValue Data1 = DAG.getLoad(MVT::i32, dl, Chain, Ptr1, MachinePointerInfo());
    // Chain = Data1.getValue(1);
    // SDValue Data2 = DAG.getLoad(MVT::i32, dl, Chain, Ptr2, MachinePointerInfo());
    // Chain = Data2.getValue(1);
    // SDValue Cmp = DAG.getNode(ISD::SUB, dl, MVT::i32, Data1, Data2);
    // // int SPFI = cast<FrameIndexSDNode>(CmpPtr.getNode())->getIndex();
    // // auto MPI = MachinePointerInfo::getFixedStack(DAG.getMachineFunction(), SPFI);
    // // Chain = DAG.getStore(Chain, dl, Cmp, CmpPtr, MPI);
    // // Cmp = DAG.getLoad(MVT::i32, dl, Chain, CmpPtr, MPI);
    // // Chain = Cmp.getValue(1);
    // // Ptr1 = DAG.getMemBasePlusOffset(Ptr1, TypeSize::Fixed(4), dl);
    // return std::make_pair(Cmp, Chain);

  return std::make_pair(SDValue(), SDValue());
}
