//===-- CrampSelectionDAGInfo.h - Cramp SelectionDAG Info -----------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the Cramp subclass for SelectionDAGTargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_CRAMP_CRAMPSELECTIONDAGINFO_H
#define LLVM_LIB_TARGET_CRAMP_CRAMPSELECTIONDAGINFO_H

#include "llvm/CodeGen/RuntimeLibcalls.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"

namespace llvm {

class CrampSelectionDAGInfo : public SelectionDAGTargetInfo {
public:
  // SDValue EmitTargetCodeForMemcpy(SelectionDAG &DAG, const SDLoc &dl,
  //                                 SDValue Chain, SDValue Dst, SDValue Src,
  //                                 SDValue Size, Align Alignment,
  //                                 bool isVolatile, bool AlwaysInline,
  //                                 MachinePointerInfo DstPtrInfo,
  //                                 MachinePointerInfo SrcPtrInfo) const override;

  // SDValue
  // EmitTargetCodeForMemmove(SelectionDAG &DAG, const SDLoc &dl, SDValue Chain,
  //                          SDValue Dst, SDValue Src, SDValue Size,
  //                          Align Alignment, bool isVolatile,
  //                          MachinePointerInfo DstPtrInfo,
  //                          MachinePointerInfo SrcPtrInfo) const override;

  // Adjust parameters for memset, see RTABI section 4.3.4
  SDValue EmitTargetCodeForMemset(SelectionDAG &DAG, const SDLoc &dl,
                                  SDValue Chain, SDValue Op1, SDValue Op2,
                                  SDValue Op3, Align Alignment, bool isVolatile,
                                  bool AlwaysInline,
                                  MachinePointerInfo DstPtrInfo) const override;

  SDValue EmitSpecializedMemclr(SelectionDAG &DAG, const SDLoc &dl,
                                SDValue Chain, SDValue Dst, SDValue Size) const;

  // SDValue EmitSpecializedLibcall(SelectionDAG &DAG, const SDLoc &dl,
  //                                SDValue Chain, SDValue Dst, SDValue Src,
  //                                SDValue Size, unsigned Align,
  //                                RTLIB::Libcall LC) const;
};

}

#endif
