//===- IntrinsicsCramp.td - Defines Cramp intrinsics -------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines all of the Cramp-specific intrinsics.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Atomics

// Atomic Intrinsics have multiple versions for different access widths, which
// all follow one of the following signatures (depending on how many arguments
// they require). We carefully instantiate only specific versions of these for
// specific integer widths, rather than using `llvm_anyint_ty`.
//
// In fact, as these intrinsics take `llvm_anyptr_ty`, the given names are the
// canonical names, and the intrinsics used in the code will have a name
// suffixed with the pointer type they are specialised for (denoted `<p>` in the
// names below), in order to avoid type conflicts.

let TargetPrefix = "cramp" in {

  // T @llvm.<name>.T.<p>(any*, T, T, T imm);
  class CrampMaskedAtomicRMWFourArg<LLVMType itype>
      : Intrinsic<[itype], [llvm_anyptr_ty, itype, itype, itype],
                  [IntrArgMemOnly, NoCapture<ArgIndex<0>>, ImmArg<ArgIndex<3>>]>;
  // T @llvm.<name>.T.<p>(any*, T, T, T, T imm);
  class CrampMaskedAtomicRMWFiveArg<LLVMType itype>
      : Intrinsic<[itype], [llvm_anyptr_ty, itype, itype, itype, itype],
                  [IntrArgMemOnly, NoCapture<ArgIndex<0>>, ImmArg<ArgIndex<4>>]>;

  // We define 32-bit and 64-bit variants of the above, where T stands for i32
  // or i64 respectively:
  multiclass CrampMaskedAtomicRMWFourArgIntrinsics {
    // i32 @llvm.<name>.i32.<p>(any*, i32, i32, i32 imm);
    def _i32 : CrampMaskedAtomicRMWFourArg<llvm_i32_ty>;
    // i64 @llvm.<name>.i32.<p>(any*, i64, i64, i64 imm);
    def _i64 : CrampMaskedAtomicRMWFourArg<llvm_i64_ty>;
  }

  multiclass CrampMaskedAtomicRMWFiveArgIntrinsics {
    // i32 @llvm.<name>.i32.<p>(any*, i32, i32, i32, i32 imm);
    def _i32 : CrampMaskedAtomicRMWFiveArg<llvm_i32_ty>;
    // i64 @llvm.<name>.i64.<p>(any*, i64, i64, i64, i64 imm);
    def _i64 : CrampMaskedAtomicRMWFiveArg<llvm_i64_ty>;
  }

  // @llvm.cramp.masked.atomicrmw.*.{i32,i64}.<p>(...)
  defm int_cramp_masked_atomicrmw_xchg : CrampMaskedAtomicRMWFourArgIntrinsics;
  defm int_cramp_masked_atomicrmw_add : CrampMaskedAtomicRMWFourArgIntrinsics;
  defm int_cramp_masked_atomicrmw_sub : CrampMaskedAtomicRMWFourArgIntrinsics;
  defm int_cramp_masked_atomicrmw_nand : CrampMaskedAtomicRMWFourArgIntrinsics;
  // Signed min and max need an extra operand to do sign extension with.
  defm int_cramp_masked_atomicrmw_max : CrampMaskedAtomicRMWFiveArgIntrinsics;
  defm int_cramp_masked_atomicrmw_min : CrampMaskedAtomicRMWFiveArgIntrinsics;
  // Unsigned min and max don't need the extra operand.
  defm int_cramp_masked_atomicrmw_umax : CrampMaskedAtomicRMWFourArgIntrinsics;
  defm int_cramp_masked_atomicrmw_umin : CrampMaskedAtomicRMWFourArgIntrinsics;

  // @llvm.cramp.masked.cmpxchg.{i32,i64}.<p>(...)
  defm int_cramp_masked_cmpxchg : CrampMaskedAtomicRMWFiveArgIntrinsics;

} // TargetPrefix = "cramp"

//===----------------------------------------------------------------------===//
// Bitmanip (Bit Manipulation) Extension

let TargetPrefix = "cramp" in {

  class CrampBitManipGPRIntrinsics
      : Intrinsic<[llvm_any_ty],
                  [LLVMMatchType<0>],
                  [IntrNoMem, IntrSpeculatable, IntrWillReturn]>;
  class CrampBitManipGPRGPRIntrinsics
      : Intrinsic<[llvm_any_ty],
                  [LLVMMatchType<0>, LLVMMatchType<0>],
                  [IntrNoMem, IntrSpeculatable, IntrWillReturn]>;
  class CrampBitManipGPRGPRGRIntrinsics
      : Intrinsic<[llvm_any_ty],
                  [LLVMMatchType<0>, LLVMMatchType<0>, LLVMMatchType<0>],
                  [IntrNoMem, IntrSpeculatable, IntrWillReturn]>;

  // Zbb
  def int_cramp_orc_b : CrampBitManipGPRIntrinsics;

  // Zbc or Zbkc
  def int_cramp_clmul  : CrampBitManipGPRGPRIntrinsics;
  def int_cramp_clmulh : CrampBitManipGPRGPRIntrinsics;

  // Zbc
  def int_cramp_clmulr : CrampBitManipGPRGPRIntrinsics;

  // Zbe
  def int_cramp_bcompress   : CrampBitManipGPRGPRIntrinsics;
  def int_cramp_bdecompress : CrampBitManipGPRGPRIntrinsics;

  // Zbf
  def int_cramp_bfp  : CrampBitManipGPRGPRIntrinsics;

  // Zbp
  def int_cramp_grev  : CrampBitManipGPRGPRIntrinsics;
  def int_cramp_gorc  : CrampBitManipGPRGPRIntrinsics;
  def int_cramp_shfl  : CrampBitManipGPRGPRIntrinsics;
  def int_cramp_unshfl  : CrampBitManipGPRGPRIntrinsics;
  def int_cramp_xperm_n  : CrampBitManipGPRGPRIntrinsics;
  def int_cramp_xperm_b  : CrampBitManipGPRGPRIntrinsics;
  def int_cramp_xperm_h  : CrampBitManipGPRGPRIntrinsics;
  def int_cramp_xperm_w  : CrampBitManipGPRGPRIntrinsics;

  // Zbr
  def int_cramp_crc32_b : CrampBitManipGPRIntrinsics;
  def int_cramp_crc32_h : CrampBitManipGPRIntrinsics;
  def int_cramp_crc32_w : CrampBitManipGPRIntrinsics;
  def int_cramp_crc32_d : CrampBitManipGPRIntrinsics;
  def int_cramp_crc32c_b : CrampBitManipGPRIntrinsics;
  def int_cramp_crc32c_h : CrampBitManipGPRIntrinsics;
  def int_cramp_crc32c_w : CrampBitManipGPRIntrinsics;
  def int_cramp_crc32c_d : CrampBitManipGPRIntrinsics;

  // Zbt
  def int_cramp_fsl : CrampBitManipGPRGPRGRIntrinsics;
  def int_cramp_fsr : CrampBitManipGPRGPRGRIntrinsics;

  // Zbkb
  def int_cramp_brev8 : CrampBitManipGPRIntrinsics;
  def int_cramp_zip   : CrampBitManipGPRIntrinsics;
  def int_cramp_unzip : CrampBitManipGPRIntrinsics;

  // Zbkx
  def int_cramp_xperm4  : CrampBitManipGPRGPRIntrinsics;
  def int_cramp_xperm8  : CrampBitManipGPRGPRIntrinsics;
} // TargetPrefix = "cramp"

//===----------------------------------------------------------------------===//
// Vectors

// The intrinsic does not have any operand that must be extended.
defvar CrampNoScalarOperand = 0xF;

// The intrinsic does not have a VL operand.
// (e.g., cramp_vmv_x_s and cramp_vfmv_f_s)
defvar CrampNoVLOperand = 0x1F;

class CrampVIntrinsic {
  // These intrinsics may accept illegal integer values in their llvm_any_ty
  // operand, so they have to be extended.
  Intrinsic IntrinsicID = !cast<Intrinsic>(NAME);
  bits<4> ScalarOperand = CrampNoScalarOperand;
  bits<5> VLOperand = CrampNoVLOperand;
}

let TargetPrefix = "cramp" in {
  // We use anyint here but we only support XLen.
  def int_cramp_vsetvli   : Intrinsic<[llvm_anyint_ty],
                           /* AVL */  [LLVMMatchType<0>,
                           /* VSEW */  LLVMMatchType<0>,
                           /* VLMUL */ LLVMMatchType<0>],
                                      [IntrNoMem, IntrHasSideEffects,
                                       ImmArg<ArgIndex<1>>,
                                       ImmArg<ArgIndex<2>>]>;
  def int_cramp_vsetvlimax : Intrinsic<[llvm_anyint_ty],
                            /* VSEW */ [LLVMMatchType<0>,
                            /* VLMUL */ LLVMMatchType<0>],
                                      [IntrNoMem, IntrHasSideEffects,
                                       ImmArg<ArgIndex<0>>,
                                       ImmArg<ArgIndex<1>>]>;

  // Versions without side effects: better optimizable and usable if only the
  // returned vector length is important.
  def int_cramp_vsetvli_opt   : Intrinsic<[llvm_anyint_ty],
                               /* AVL */  [LLVMMatchType<0>,
                               /* VSEW */  LLVMMatchType<0>,
                               /* VLMUL */ LLVMMatchType<0>],
                                          [IntrNoMem,
                                           ImmArg<ArgIndex<1>>,
                                           ImmArg<ArgIndex<2>>]>;
  def int_cramp_vsetvlimax_opt : Intrinsic<[llvm_anyint_ty],
                                /* VSEW */ [LLVMMatchType<0>,
                                /* VLMUL */ LLVMMatchType<0>],
                                          [IntrNoMem,
                                           ImmArg<ArgIndex<0>>,
                                           ImmArg<ArgIndex<1>>]>;

  // For unit stride mask load
  // Input: (pointer, vl)
  class CrampUSMLoad
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMPointerType<LLVMMatchType<0>>,
                     llvm_anyint_ty],
                    [NoCapture<ArgIndex<0>>, IntrReadMem]>, CrampVIntrinsic {
    let VLOperand = 1;
  }
  // For unit stride load
  // Input: (passthru, pointer, vl)
  class CrampUSLoad
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>,
                     LLVMPointerType<LLVMMatchType<0>>,
                     llvm_anyint_ty],
                    [NoCapture<ArgIndex<1>>, IntrReadMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }
  // For unit stride fault-only-first load
  // Input: (passthru, pointer, vl)
  // Output: (data, vl)
  // NOTE: We model this with default memory properties since we model writing
  // VL as a side effect. IntrReadMem, IntrHasSideEffects does not work.
  class CrampUSLoadFF
        : Intrinsic<[llvm_anyvector_ty, llvm_anyint_ty],
                    [LLVMMatchType<0>,
                     LLVMPointerType<LLVMMatchType<0>>, LLVMMatchType<1>],
                    [NoCapture<ArgIndex<1>>]>,
                    CrampVIntrinsic {
    let VLOperand = 2;
  }
  // For unit stride load with mask
  // Input: (maskedoff, pointer, mask, vl, policy)
  class CrampUSLoadMasked
        : Intrinsic<[llvm_anyvector_ty ],
                    [LLVMMatchType<0>,
                     LLVMPointerType<LLVMMatchType<0>>,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                     llvm_anyint_ty, LLVMMatchType<1>],
                    [NoCapture<ArgIndex<1>>, ImmArg<ArgIndex<4>>, IntrReadMem]>,
                    CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For unit stride fault-only-first load with mask
  // Input: (maskedoff, pointer, mask, vl, policy)
  // Output: (data, vl)
  // NOTE: We model this with default memory properties since we model writing
  // VL as a side effect. IntrReadMem, IntrHasSideEffects does not work.
  class CrampUSLoadFFMasked
        : Intrinsic<[llvm_anyvector_ty, llvm_anyint_ty],
                    [LLVMMatchType<0>,
                     LLVMPointerType<LLVMMatchType<0>>,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                     LLVMMatchType<1>, LLVMMatchType<1>],
                    [NoCapture<ArgIndex<1>>, ImmArg<ArgIndex<4>>]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For strided load with passthru operand
  // Input: (passthru, pointer, stride, vl)
  class CrampSLoad
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>,
                     LLVMPointerType<LLVMMatchType<0>>,
                     llvm_anyint_ty, LLVMMatchType<1>],
                    [NoCapture<ArgIndex<1>>, IntrReadMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For strided load with mask
  // Input: (maskedoff, pointer, stride, mask, vl, policy)
  class CrampSLoadMasked
        : Intrinsic<[llvm_anyvector_ty ],
                    [LLVMMatchType<0>,
                     LLVMPointerType<LLVMMatchType<0>>, llvm_anyint_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, LLVMMatchType<1>,
                     LLVMMatchType<1>],
                    [NoCapture<ArgIndex<1>>, ImmArg<ArgIndex<5>>, IntrReadMem]>,
                    CrampVIntrinsic {
    let VLOperand = 4;
  }
  // For indexed load with passthru operand
  // Input: (passthru, pointer, index, vl)
  class CrampILoad
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>,
                     LLVMPointerType<LLVMMatchType<0>>,
                     llvm_anyvector_ty, llvm_anyint_ty],
                    [NoCapture<ArgIndex<1>>, IntrReadMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For indexed load with mask
  // Input: (maskedoff, pointer, index, mask, vl, policy)
  class CrampILoadMasked
        : Intrinsic<[llvm_anyvector_ty ],
                    [LLVMMatchType<0>,
                     LLVMPointerType<LLVMMatchType<0>>, llvm_anyvector_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                     LLVMMatchType<2>],
                    [NoCapture<ArgIndex<1>>, ImmArg<ArgIndex<5>>, IntrReadMem]>,
                    CrampVIntrinsic {
    let VLOperand = 4;
  }
  // For unit stride store
  // Input: (vector_in, pointer, vl)
  class CrampUSStore
        : Intrinsic<[],
                    [llvm_anyvector_ty,
                     LLVMPointerType<LLVMMatchType<0>>,
                     llvm_anyint_ty],
                    [NoCapture<ArgIndex<1>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }
  // For unit stride store with mask
  // Input: (vector_in, pointer, mask, vl)
  class CrampUSStoreMasked
        : Intrinsic<[],
                    [llvm_anyvector_ty,
                     LLVMPointerType<LLVMMatchType<0>>,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                     llvm_anyint_ty],
                    [NoCapture<ArgIndex<1>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For strided store
  // Input: (vector_in, pointer, stride, vl)
  class CrampSStore
        : Intrinsic<[],
                    [llvm_anyvector_ty,
                     LLVMPointerType<LLVMMatchType<0>>,
                     llvm_anyint_ty, LLVMMatchType<1>],
                    [NoCapture<ArgIndex<1>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For stride store with mask
  // Input: (vector_in, pointer, stirde, mask, vl)
  class CrampSStoreMasked
        : Intrinsic<[],
                    [llvm_anyvector_ty,
                     LLVMPointerType<LLVMMatchType<0>>, llvm_anyint_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, LLVMMatchType<1>],
                    [NoCapture<ArgIndex<1>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // For indexed store
  // Input: (vector_in, pointer, index, vl)
  class CrampIStore
        : Intrinsic<[],
                    [llvm_anyvector_ty,
                     LLVMPointerType<LLVMMatchType<0>>,
                     llvm_anyint_ty, llvm_anyint_ty],
                    [NoCapture<ArgIndex<1>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For indexed store with mask
  // Input: (vector_in, pointer, index, mask, vl)
  class CrampIStoreMasked
        : Intrinsic<[],
                    [llvm_anyvector_ty,
                     LLVMPointerType<LLVMMatchType<0>>, llvm_anyvector_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty],
                    [NoCapture<ArgIndex<1>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // For destination vector type is the same as source vector.
  // Input: (passthru, vector_in, vl)
  class CrampUnaryAAUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }
  // For destination vector type is the same as first source vector (with mask).
  // Input: (vector_in, vector_in, mask, vl, policy)
  class CrampUnaryAAMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                     LLVMMatchType<1>],
                    [ImmArg<ArgIndex<4>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // Input: (passthru, vector_in, vector_in, mask, vl)
  class CrampCompress
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For destination vector type is the same as first and second source vector.
  // Input: (vector_in, vector_in, vl)
  class CrampBinaryAAAUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }
  // For destination vector type is the same as first and second source vector.
  // Input: (passthru, vector_in, int_vector_in, vl)
  class CrampRGatherVVUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>,
                     LLVMVectorOfBitcastsToInt<0>, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For destination vector type is the same as first and second source vector.
  // Input: (vector_in, vector_in, int_vector_in, vl, policy)
  class CrampRGatherVVMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, LLVMVectorOfBitcastsToInt<0>,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                     LLVMMatchType<1>],
                    [ImmArg<ArgIndex<5>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // Input: (passthru, vector_in, int16_vector_in, vl)
  class CrampRGatherEI16VVUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>,
                     LLVMScalarOrSameVectorWidth<0, llvm_i16_ty>,
                     llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For destination vector type is the same as first and second source vector.
  // Input: (vector_in, vector_in, int16_vector_in, vl, policy)
  class CrampRGatherEI16VVMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>,
                     LLVMScalarOrSameVectorWidth<0, llvm_i16_ty>,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                     LLVMMatchType<1>],
                    [ImmArg<ArgIndex<5>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // For destination vector type is the same as first source vector, and the
  // second operand is XLen.
  // Input: (passthru, vector_in, xlen_in, vl)
  class CrampGatherVXUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_anyint_ty,
                     LLVMMatchType<1>],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For destination vector type is the same as first source vector (with mask).
  // Second operand is XLen.
  // Input: (maskedoff, vector_in, xlen_in, mask, vl, policy)
  class CrampGatherVXMasked
       : Intrinsic<[llvm_anyvector_ty],
                   [LLVMMatchType<0>, LLVMMatchType<0>, llvm_anyint_ty,
                    LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, LLVMMatchType<1>,
                    LLVMMatchType<1>],
                   [ImmArg<ArgIndex<5>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // For destination vector type is the same as first source vector.
  // Input: (passthru, vector_in, vector_in/scalar_in, vl)
  class CrampBinaryAAXUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_any_ty,
                     llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 2;
    let VLOperand = 3;
  }
  // For destination vector type is the same as first source vector (with mask).
  // Input: (maskedoff, vector_in, vector_in/scalar_in, mask, vl, policy)
  class CrampBinaryAAXMasked
       : Intrinsic<[llvm_anyvector_ty],
                   [LLVMMatchType<0>, LLVMMatchType<0>, llvm_any_ty,
                    LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                    LLVMMatchType<2>],
                   [ImmArg<ArgIndex<5>>, IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 2;
    let VLOperand = 4;
  }
  // For destination vector type is the same as first source vector. The
  // second source operand must match the destination type or be an XLen scalar.
  // Input: (passthru, vector_in, vector_in/scalar_in, vl)
  class CrampBinaryAAShiftUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_any_ty,
                     llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For destination vector type is the same as first source vector (with mask).
  // The second source operand must match the destination type or be an XLen scalar.
  // Input: (maskedoff, vector_in, vector_in/scalar_in, mask, vl, policy)
  class CrampBinaryAAShiftMasked
       : Intrinsic<[llvm_anyvector_ty],
                   [LLVMMatchType<0>, LLVMMatchType<0>, llvm_any_ty,
                    LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                    LLVMMatchType<2>],
                   [ImmArg<ArgIndex<5>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // For destination vector type is NOT the same as first source vector.
  // Input: (passthru, vector_in, vector_in/scalar_in, vl)
  class CrampBinaryABXUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty, llvm_any_ty,
                     llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 2;
    let VLOperand = 3;
  }
  // For destination vector type is NOT the same as first source vector (with mask).
  // Input: (maskedoff, vector_in, vector_in/scalar_in, mask, vl, policy)
  class CrampBinaryABXMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty, llvm_any_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                     LLVMMatchType<3>],
                    [ImmArg<ArgIndex<5>>, IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 2;
    let VLOperand = 4;
  }
  // For destination vector type is NOT the same as first source vector. The
  // second source operand must match the destination type or be an XLen scalar.
  // Input: (passthru, vector_in, vector_in/scalar_in, vl)
  class CrampBinaryABShiftUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty, llvm_any_ty,
                     llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For destination vector type is NOT the same as first source vector (with mask).
  // The second source operand must match the destination type or be an XLen scalar.
  // Input: (maskedoff, vector_in, vector_in/scalar_in, mask, vl, policy)
  class CrampBinaryABShiftMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty, llvm_any_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                     LLVMMatchType<3>],
                    [ImmArg<ArgIndex<5>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // For binary operations with V0 as input.
  // Input: (passthru, vector_in, vector_in/scalar_in, V0, vl)
  class CrampBinaryWithV0
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_any_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                     llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 2;
    let VLOperand = 4;
  }
  // For binary operations with mask type output and V0 as input.
  // Output: (mask type output)
  // Input: (vector_in, vector_in/scalar_in, V0, vl)
  class CrampBinaryMOutWithV0
        :Intrinsic<[LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>],
                   [llvm_anyvector_ty, llvm_any_ty,
                    LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                    llvm_anyint_ty],
                   [IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 1;
    let VLOperand = 3;
  }
  // For binary operations with mask type output.
  // Output: (mask type output)
  // Input: (vector_in, vector_in/scalar_in, vl)
  class CrampBinaryMOut
        : Intrinsic<[LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>],
                    [llvm_anyvector_ty, llvm_any_ty, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 1;
    let VLOperand = 2;
  }
  // For binary operations with mask type output without mask.
  // Output: (mask type output)
  // Input: (vector_in, vector_in/scalar_in, vl)
  class CrampCompareUnMasked
        : Intrinsic<[LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>],
                    [llvm_anyvector_ty, llvm_any_ty, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 1;
    let VLOperand = 2;
  }
  // For binary operations with mask type output with mask.
  // Output: (mask type output)
  // Input: (maskedoff, vector_in, vector_in/scalar_in, mask, vl)
  class CrampCompareMasked
        : Intrinsic<[LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>],
                    [LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                     llvm_anyvector_ty, llvm_any_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 2;
    let VLOperand = 4;
  }
  // For FP classify operations.
  // Output: (bit mask type output)
  // Input: (passthru, vector_in, vl)
  class CrampClassifyUnMasked
        : Intrinsic<[LLVMVectorOfBitcastsToInt<0>],
                    [LLVMVectorOfBitcastsToInt<0>, llvm_anyvector_ty,
                      llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 1;
  }
  // For FP classify operations with mask.
  // Output: (bit mask type output)
  // Input: (maskedoff, vector_in, mask, vl, policy)
  class CrampClassifyMasked
        : Intrinsic<[LLVMVectorOfBitcastsToInt<0>],
                    [LLVMVectorOfBitcastsToInt<0>, llvm_anyvector_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, 
                     llvm_anyint_ty, LLVMMatchType<1>],
                    [IntrNoMem, ImmArg<ArgIndex<4>>]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For Saturating binary operations.
  // The destination vector type is the same as first source vector.
  // Input: (passthru, vector_in, vector_in/scalar_in, vl)
  class CrampSaturatingBinaryAAXUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_any_ty,
                     llvm_anyint_ty],
                    [IntrNoMem, IntrHasSideEffects]>, CrampVIntrinsic {
    let ScalarOperand = 2;
    let VLOperand = 3;
  }
  // For Saturating binary operations with mask.
  // The destination vector type is the same as first source vector.
  // Input: (maskedoff, vector_in, vector_in/scalar_in, mask, vl, policy)
  class CrampSaturatingBinaryAAXMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_any_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                     LLVMMatchType<2>],
                    [ImmArg<ArgIndex<5>>, IntrNoMem, IntrHasSideEffects]>, CrampVIntrinsic {
    let ScalarOperand = 2;
    let VLOperand = 4;
  }
  // For Saturating binary operations.
  // The destination vector type is the same as first source vector.
  // The second source operand matches the destination type or is an XLen scalar.
  // Input: (passthru, vector_in, vector_in/scalar_in, vl)
  class CrampSaturatingBinaryAAShiftUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_any_ty,
                     llvm_anyint_ty],
                    [IntrNoMem, IntrHasSideEffects]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For Saturating binary operations with mask.
  // The destination vector type is the same as first source vector.
  // The second source operand matches the destination type or is an XLen scalar.
  // Input: (maskedoff, vector_in, vector_in/scalar_in, mask, vl, policy)
  class CrampSaturatingBinaryAAShiftMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_any_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                     LLVMMatchType<2>],
                    [ImmArg<ArgIndex<5>>, IntrNoMem, IntrHasSideEffects]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // For Saturating binary operations.
  // The destination vector type is NOT the same as first source vector.
  // The second source operand matches the destination type or is an XLen scalar.
  // Input: (passthru, vector_in, vector_in/scalar_in, vl)
  class CrampSaturatingBinaryABShiftUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty, llvm_any_ty,
                     llvm_anyint_ty],
                    [IntrNoMem, IntrHasSideEffects]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For Saturating binary operations with mask.
  // The destination vector type is NOT the same as first source vector (with mask).
  // The second source operand matches the destination type or is an XLen scalar.
  // Input: (maskedoff, vector_in, vector_in/scalar_in, mask, vl, policy)
  class CrampSaturatingBinaryABShiftMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty, llvm_any_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                     LLVMMatchType<3>],
                    [ImmArg<ArgIndex<5>>, IntrNoMem, IntrHasSideEffects]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // Input: (vector_in, vector_in, scalar_in, vl, policy)
  class CrampRVVSlideUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_anyint_ty,
                     LLVMMatchType<1>, LLVMMatchType<1>],
                    [ImmArg<ArgIndex<4>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // Input: (vector_in, vector_in, vector_in/scalar_in, mask, vl, policy)
  class CrampRVVSlideMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>, llvm_anyint_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                     LLVMMatchType<1>, LLVMMatchType<1>],
                    [ImmArg<ArgIndex<5>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // UnMasked Vector Multiply-Add operations, its first operand can not be undef.
  // Input: (vector_in, vector_in/scalar, vector_in, vl, policy)
  class CrampTernaryAAXAUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_any_ty, LLVMMatchType<0>,
                     llvm_anyint_ty, LLVMMatchType<2>],
                    [ImmArg<ArgIndex<4>>, IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 1;
    let VLOperand = 3;
  }
  // Masked Vector Multiply-Add operations, its first operand can not be undef.
  // Input: (vector_in, vector_in/scalar, vector_in, mask, vl, policy
  class CrampTernaryAAXAMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_any_ty, LLVMMatchType<0>,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                     llvm_anyint_ty, LLVMMatchType<2>],
                    [ImmArg<ArgIndex<5>>, IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 1;
    let VLOperand = 4;
  }
  // UnMasked Widening Vector Multiply-Add operations, its first operand can not be undef.
  // Input: (vector_in, vector_in/scalar, vector_in, vl, policy)
  class CrampTernaryWideUnMasked
        : Intrinsic< [llvm_anyvector_ty],
                     [LLVMMatchType<0>, llvm_any_ty, llvm_anyvector_ty,
                      llvm_anyint_ty, LLVMMatchType<3>],
                     [ImmArg<ArgIndex<4>>, IntrNoMem] >, CrampVIntrinsic {
    let ScalarOperand = 1;
    let VLOperand = 3;
  }
  // Masked Widening Vector Multiply-Add operations, its first operand can not be undef.
  // Input: (vector_in, vector_in/scalar, vector_in, mask, vl, policy
  class CrampTernaryWideMasked
        : Intrinsic< [llvm_anyvector_ty],
                     [LLVMMatchType<0>, llvm_any_ty, llvm_anyvector_ty,
                      LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                      llvm_anyint_ty, LLVMMatchType<3>],
                     [ImmArg<ArgIndex<5>>, IntrNoMem]>, CrampVIntrinsic {
    let ScalarOperand = 1;
    let VLOperand = 4;
  }
  // For Reduction ternary operations.
  // For destination vector type is the same as first and third source vector.
  // Input: (vector_in, vector_in, vector_in, vl)
  class CrampReductionUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty, LLVMMatchType<0>,
                     llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For Reduction ternary operations with mask.
  // For destination vector type is the same as first and third source vector.
  // The mask type come from second source vector.
  // Input: (maskedoff, vector_in, vector_in, vector_in, mask, vl)
  class CrampReductionMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty, LLVMMatchType<0>,
                     LLVMScalarOrSameVectorWidth<1, llvm_i1_ty>, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 4;
  }
  // For unary operations with scalar type output without mask
  // Output: (scalar type)
  // Input: (vector_in, vl)
  class CrampMaskedUnarySOutUnMasked
        : Intrinsic<[LLVMMatchType<1>],
                    [llvm_anyvector_ty, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 1;
  }
  // For unary operations with scalar type output with mask
  // Output: (scalar type)
  // Input: (vector_in, mask, vl)
  class CrampMaskedUnarySOutMasked
        : Intrinsic<[LLVMMatchType<1>],
                    [llvm_anyvector_ty, LLVMMatchType<0>, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }
  // For destination vector type is NOT the same as source vector.
  // Input: (passthru, vector_in, vl)
  class CrampUnaryABUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }
  // For destination vector type is NOT the same as source vector (with mask).
  // Input: (maskedoff, vector_in, mask, vl, policy)
  class CrampUnaryABMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty,
                     LLVMScalarOrSameVectorWidth<1, llvm_i1_ty>,
                     llvm_anyint_ty, LLVMMatchType<2>],
                    [ImmArg<ArgIndex<4>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // For unary operations with the same vector type in/out without mask
  // Output: (vector)
  // Input: (vector_in, vl)
  class CrampUnaryUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 1;
  }
  // For mask unary operations with mask type in/out with mask
  // Output: (mask type output)
  // Input: (mask type maskedoff, mask type vector_in, mask, vl)
  class CrampMaskedUnaryMOutMasked
        : Intrinsic<[llvm_anyint_ty],
                    [LLVMMatchType<0>, LLVMMatchType<0>,
                     LLVMMatchType<0>, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // Output: (vector)
  // Input: (vl)
  class CrampNullaryIntrinsic
        : Intrinsic<[llvm_anyvector_ty],
                    [llvm_anyint_ty], [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 1;
  }
  // Output: (vector)
  // Input: (passthru, vl)
  class CrampID
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 1;
  }
  // For Conversion unary operations.
  // Input: (passthru, vector_in, vl)
  class CrampConversionUnMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty, llvm_anyint_ty],
                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }
  // For Conversion unary operations with mask.
  // Input: (maskedoff, vector_in, mask, vl, policy)
  class CrampConversionMasked
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyvector_ty,
                     LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>, llvm_anyint_ty,
                     LLVMMatchType<2>],
                    [ImmArg<ArgIndex<4>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }

  // For unit stride segment load
  // Input: (passthru, pointer, vl)
  class CrampUSSegLoad<int nf>
        : Intrinsic<!listconcat([llvm_anyvector_ty], !listsplat(LLVMMatchType<0>,
                                !add(nf, -1))),
                    !listconcat(!listsplat(LLVMMatchType<0>, nf),
                                [LLVMPointerToElt<0>, llvm_anyint_ty]),
                    [NoCapture<ArgIndex<nf>>, IntrReadMem]>, CrampVIntrinsic {
    let VLOperand = !add(nf, 1);
  }
  // For unit stride segment load with mask
  // Input: (maskedoff, pointer, mask, vl, policy)
  class CrampUSSegLoadMasked<int nf>
        : Intrinsic<!listconcat([llvm_anyvector_ty], !listsplat(LLVMMatchType<0>,
                                !add(nf, -1))),
                    !listconcat(!listsplat(LLVMMatchType<0>, nf),
                                [LLVMPointerToElt<0>,
                                 LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                                 llvm_anyint_ty, LLVMMatchType<1>]),
                    [ImmArg<ArgIndex<!add(nf, 3)>>, NoCapture<ArgIndex<nf>>, IntrReadMem]>,
                    CrampVIntrinsic {
    let VLOperand = !add(nf, 2);
  }

  // For unit stride fault-only-first segment load
  // Input: (passthru, pointer, vl)
  // Output: (data, vl)
  // NOTE: We model this with default memory properties since we model writing
  // VL as a side effect. IntrReadMem, IntrHasSideEffects does not work.
  class CrampUSSegLoadFF<int nf>
        : Intrinsic<!listconcat([llvm_anyvector_ty], !listsplat(LLVMMatchType<0>,
                                !add(nf, -1)), [llvm_anyint_ty]),
                    !listconcat(!listsplat(LLVMMatchType<0>, nf),
                    [LLVMPointerToElt<0>, LLVMMatchType<1>]),
                    [NoCapture<ArgIndex<nf>>]>, CrampVIntrinsic {
    let VLOperand = !add(nf, 1);
  }
  // For unit stride fault-only-first segment load with mask
  // Input: (maskedoff, pointer, mask, vl, policy)
  // Output: (data, vl)
  // NOTE: We model this with default memory properties since we model writing
  // VL as a side effect. IntrReadMem, IntrHasSideEffects does not work.
  class CrampUSSegLoadFFMasked<int nf>
        : Intrinsic<!listconcat([llvm_anyvector_ty], !listsplat(LLVMMatchType<0>,
                                !add(nf, -1)), [llvm_anyint_ty]),
                    !listconcat(!listsplat(LLVMMatchType<0>, nf),
                     [LLVMPointerToElt<0>,
                      LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                      LLVMMatchType<1>, LLVMMatchType<1>]),
                    [ImmArg<ArgIndex<!add(nf, 3)>>, NoCapture<ArgIndex<nf>>]>,
                    CrampVIntrinsic {
    let VLOperand = !add(nf, 2);
  }

  // For stride segment load
  // Input: (passthru, pointer, offset, vl)
  class CrampSSegLoad<int nf>
        : Intrinsic<!listconcat([llvm_anyvector_ty], !listsplat(LLVMMatchType<0>,
                                !add(nf, -1))),
                    !listconcat(!listsplat(LLVMMatchType<0>, nf),
                    [LLVMPointerToElt<0>, llvm_anyint_ty, LLVMMatchType<1>]),
                    [NoCapture<ArgIndex<nf>>, IntrReadMem]>, CrampVIntrinsic {
    let VLOperand = !add(nf, 2);
  }
  // For stride segment load with mask
  // Input: (maskedoff, pointer, offset, mask, vl, policy)
  class CrampSSegLoadMasked<int nf>
        : Intrinsic<!listconcat([llvm_anyvector_ty], !listsplat(LLVMMatchType<0>,
                                !add(nf, -1))),
                    !listconcat(!listsplat(LLVMMatchType<0>, nf),
                                [LLVMPointerToElt<0>,
                                 llvm_anyint_ty,
                                 LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                                 LLVMMatchType<1>, LLVMMatchType<1>]),
                    [ImmArg<ArgIndex<!add(nf, 4)>>, NoCapture<ArgIndex<nf>>, IntrReadMem]>,
                    CrampVIntrinsic {
    let VLOperand = !add(nf, 3);
  }

  // For indexed segment load
  // Input: (passthru, pointer, index, vl)
  class CrampISegLoad<int nf>
        : Intrinsic<!listconcat([llvm_anyvector_ty], !listsplat(LLVMMatchType<0>,
                                !add(nf, -1))),
                    !listconcat(!listsplat(LLVMMatchType<0>, nf),
                    [LLVMPointerToElt<0>, llvm_anyvector_ty, llvm_anyint_ty]),
                    [NoCapture<ArgIndex<nf>>, IntrReadMem]>, CrampVIntrinsic {
    let VLOperand = !add(nf, 2);
  }
  // For indexed segment load with mask
  // Input: (maskedoff, pointer, index, mask, vl, policy)
  class CrampISegLoadMasked<int nf>
        : Intrinsic<!listconcat([llvm_anyvector_ty], !listsplat(LLVMMatchType<0>,
                                !add(nf, -1))),
                    !listconcat(!listsplat(LLVMMatchType<0>, nf),
                                [LLVMPointerToElt<0>,
                                 llvm_anyvector_ty,
                                 LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                                 llvm_anyint_ty, LLVMMatchType<2>]),
                    [ImmArg<ArgIndex<!add(nf, 4)>>, NoCapture<ArgIndex<nf>>, IntrReadMem]>,
                    CrampVIntrinsic {
    let VLOperand = !add(nf, 3);
  }

  // For unit stride segment store
  // Input: (value, pointer, vl)
  class CrampUSSegStore<int nf>
        : Intrinsic<[],
                    !listconcat([llvm_anyvector_ty],
                                !listsplat(LLVMMatchType<0>, !add(nf, -1)),
                                [LLVMPointerToElt<0>, llvm_anyint_ty]),
                    [NoCapture<ArgIndex<nf>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = !add(nf, 1);
  }
  // For unit stride segment store with mask
  // Input: (value, pointer, mask, vl)
  class CrampUSSegStoreMasked<int nf>
        : Intrinsic<[],
                    !listconcat([llvm_anyvector_ty],
                                !listsplat(LLVMMatchType<0>, !add(nf, -1)),
                                [LLVMPointerToElt<0>,
                                 LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                                 llvm_anyint_ty]),
                    [NoCapture<ArgIndex<nf>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = !add(nf, 2);
  }

  // For stride segment store
  // Input: (value, pointer, offset, vl)
  class CrampSSegStore<int nf>
        : Intrinsic<[],
                    !listconcat([llvm_anyvector_ty],
                                !listsplat(LLVMMatchType<0>, !add(nf, -1)),
                                [LLVMPointerToElt<0>, llvm_anyint_ty,
                                 LLVMMatchType<1>]),
                    [NoCapture<ArgIndex<nf>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = !add(nf, 2);
  }
  // For stride segment store with mask
  // Input: (value, pointer, offset, mask, vl)
  class CrampSSegStoreMasked<int nf>
        : Intrinsic<[],
                    !listconcat([llvm_anyvector_ty],
                                !listsplat(LLVMMatchType<0>, !add(nf, -1)),
                                [LLVMPointerToElt<0>, llvm_anyint_ty,
                                 LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                                 LLVMMatchType<1>]),
                    [NoCapture<ArgIndex<nf>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = !add(nf, 3);
  }

  // For indexed segment store
  // Input: (value, pointer, offset, vl)
  class CrampISegStore<int nf>
        : Intrinsic<[],
                    !listconcat([llvm_anyvector_ty],
                                !listsplat(LLVMMatchType<0>, !add(nf, -1)),
                                [LLVMPointerToElt<0>, llvm_anyvector_ty,
                                 llvm_anyint_ty]),
                    [NoCapture<ArgIndex<nf>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = !add(nf, 2);
  }
  // For indexed segment store with mask
  // Input: (value, pointer, offset, mask, vl)
  class CrampISegStoreMasked<int nf>
        : Intrinsic<[],
                    !listconcat([llvm_anyvector_ty],
                                !listsplat(LLVMMatchType<0>, !add(nf, -1)),
                                [LLVMPointerToElt<0>, llvm_anyvector_ty,
                                 LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                                 llvm_anyint_ty]),
                    [NoCapture<ArgIndex<nf>>, IntrWriteMem]>, CrampVIntrinsic {
    let VLOperand = !add(nf, 3);
  }

  multiclass CrampUSLoad {
    def "int_cramp_" # NAME : CrampUSLoad;
    def "int_cramp_" # NAME # "_mask" : CrampUSLoadMasked;
  }
  multiclass CrampUSLoadFF {
    def "int_cramp_" # NAME : CrampUSLoadFF;
    def "int_cramp_" # NAME # "_mask" : CrampUSLoadFFMasked;
  }
  multiclass CrampSLoad {
    def "int_cramp_" # NAME : CrampSLoad;
    def "int_cramp_" # NAME # "_mask" : CrampSLoadMasked;
  }
  multiclass CrampILoad {
    def "int_cramp_" # NAME : CrampILoad;
    def "int_cramp_" # NAME # "_mask" : CrampILoadMasked;
  }
  multiclass CrampUSStore {
    def "int_cramp_" # NAME : CrampUSStore;
    def "int_cramp_" # NAME # "_mask" : CrampUSStoreMasked;
  }
  multiclass CrampSStore {
    def "int_cramp_" # NAME : CrampSStore;
    def "int_cramp_" # NAME # "_mask" : CrampSStoreMasked;
  }

  multiclass CrampIStore {
    def "int_cramp_" # NAME : CrampIStore;
    def "int_cramp_" # NAME # "_mask" : CrampIStoreMasked;
  }
  multiclass CrampUnaryAA {
    def "int_cramp_" # NAME : CrampUnaryAAUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampUnaryAAMasked;
  }
  multiclass CrampUnaryAB {
    def "int_cramp_" # NAME : CrampUnaryABUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampUnaryABMasked;
  }
  // AAX means the destination type(A) is the same as the first source
  // type(A). X means any type for the second source operand.
  multiclass CrampBinaryAAX {
    def "int_cramp_" # NAME : CrampBinaryAAXUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampBinaryAAXMasked;
  }
  // Like CrampBinaryAAX, but the second operand is used a shift amount so it
  // must be a vector or an XLen scalar.
  multiclass CrampBinaryAAShift {
    def "int_cramp_" # NAME : CrampBinaryAAShiftUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampBinaryAAShiftMasked;
  }
  multiclass CrampRGatherVV {
    def "int_cramp_" # NAME : CrampRGatherVVUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampRGatherVVMasked;
  }
  multiclass CrampRGatherVX {
    def "int_cramp_" # NAME : CrampGatherVXUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampGatherVXMasked;
  }
  multiclass CrampRGatherEI16VV {
    def "int_cramp_" # NAME : CrampRGatherEI16VVUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampRGatherEI16VVMasked;
  }
  // ABX means the destination type(A) is different from the first source
  // type(B). X means any type for the second source operand.
  multiclass CrampBinaryABX {
    def "int_cramp_" # NAME : CrampBinaryABXUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampBinaryABXMasked;
  }
  // Like CrampBinaryABX, but the second operand is used a shift amount so it
  // must be a vector or an XLen scalar.
  multiclass CrampBinaryABShift {
    def "int_cramp_" # NAME : CrampBinaryABShiftUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampBinaryABShiftMasked;
  }
  multiclass CrampBinaryWithV0 {
    def "int_cramp_" # NAME : CrampBinaryWithV0;
  }
  multiclass CrampBinaryMaskOutWithV0 {
    def "int_cramp_" # NAME : CrampBinaryMOutWithV0;
  }
  multiclass CrampBinaryMaskOut {
    def "int_cramp_" # NAME : CrampBinaryMOut;
  }
  multiclass CrampSaturatingBinaryAAX {
    def "int_cramp_" # NAME : CrampSaturatingBinaryAAXUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampSaturatingBinaryAAXMasked;
  }
  multiclass CrampSaturatingBinaryAAShift {
    def "int_cramp_" # NAME : CrampSaturatingBinaryAAShiftUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampSaturatingBinaryAAShiftMasked;
  }
  multiclass CrampSaturatingBinaryABShift {
    def "int_cramp_" # NAME : CrampSaturatingBinaryABShiftUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampSaturatingBinaryABShiftMasked;
  }
  multiclass CrampRVVSlide {
    def "int_cramp_" # NAME : CrampRVVSlideUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampRVVSlideMasked;
  }
  multiclass CrampTernaryAAXA {
    def "int_cramp_" # NAME : CrampTernaryAAXAUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampTernaryAAXAMasked;
  }
  multiclass CrampCompare {
    def "int_cramp_" # NAME : CrampCompareUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampCompareMasked;
  }
  multiclass CrampClassify {
    def "int_cramp_" # NAME : CrampClassifyUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampClassifyMasked;
  }
  multiclass CrampTernaryWide {
    def "int_cramp_" # NAME : CrampTernaryWideUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampTernaryWideMasked;
  }
  multiclass CrampReduction {
    def "int_cramp_" # NAME : CrampReductionUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampReductionMasked;
  }
  multiclass CrampMaskedUnarySOut {
    def "int_cramp_" # NAME : CrampMaskedUnarySOutUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampMaskedUnarySOutMasked;
  }
  multiclass CrampMaskedUnaryMOut {
    def "int_cramp_" # NAME : CrampUnaryUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampMaskedUnaryMOutMasked;
  }
  multiclass CrampConversion {
    def "int_cramp_" #NAME :CrampConversionUnMasked;
    def "int_cramp_" # NAME # "_mask" : CrampConversionMasked;
  }
  multiclass CrampUSSegLoad<int nf> {
    def "int_cramp_" # NAME : CrampUSSegLoad<nf>;
    def "int_cramp_" # NAME # "_mask" : CrampUSSegLoadMasked<nf>;
  }
  multiclass CrampUSSegLoadFF<int nf> {
    def "int_cramp_" # NAME : CrampUSSegLoadFF<nf>;
    def "int_cramp_" # NAME # "_mask" : CrampUSSegLoadFFMasked<nf>;
  }
  multiclass CrampSSegLoad<int nf> {
    def "int_cramp_" # NAME : CrampSSegLoad<nf>;
    def "int_cramp_" # NAME # "_mask" : CrampSSegLoadMasked<nf>;
  }
  multiclass CrampISegLoad<int nf> {
    def "int_cramp_" # NAME : CrampISegLoad<nf>;
    def "int_cramp_" # NAME # "_mask" : CrampISegLoadMasked<nf>;
  }
  multiclass CrampUSSegStore<int nf> {
    def "int_cramp_" # NAME : CrampUSSegStore<nf>;
    def "int_cramp_" # NAME # "_mask" : CrampUSSegStoreMasked<nf>;
  }
  multiclass CrampSSegStore<int nf> {
    def "int_cramp_" # NAME : CrampSSegStore<nf>;
    def "int_cramp_" # NAME # "_mask" : CrampSSegStoreMasked<nf>;
  }
  multiclass CrampISegStore<int nf> {
    def "int_cramp_" # NAME : CrampISegStore<nf>;
    def "int_cramp_" # NAME # "_mask" : CrampISegStoreMasked<nf>;
  }

  defm vle : CrampUSLoad;
  defm vleff : CrampUSLoadFF;
  defm vse : CrampUSStore;
  defm vlse: CrampSLoad;
  defm vsse: CrampSStore;
  defm vluxei : CrampILoad;
  defm vloxei : CrampILoad;
  defm vsoxei : CrampIStore;
  defm vsuxei : CrampIStore;

  def int_cramp_vlm : CrampUSMLoad;
  def int_cramp_vsm : CrampUSStore;

  defm vadd : CrampBinaryAAX;
  defm vsub : CrampBinaryAAX;
  defm vrsub : CrampBinaryAAX;

  defm vwaddu : CrampBinaryABX;
  defm vwadd : CrampBinaryABX;
  defm vwaddu_w : CrampBinaryAAX;
  defm vwadd_w : CrampBinaryAAX;
  defm vwsubu : CrampBinaryABX;
  defm vwsub : CrampBinaryABX;
  defm vwsubu_w : CrampBinaryAAX;
  defm vwsub_w : CrampBinaryAAX;

  defm vzext : CrampUnaryAB;
  defm vsext : CrampUnaryAB;

  defm vadc : CrampBinaryWithV0;
  defm vmadc_carry_in : CrampBinaryMaskOutWithV0;
  defm vmadc : CrampBinaryMaskOut;

  defm vsbc : CrampBinaryWithV0;
  defm vmsbc_borrow_in : CrampBinaryMaskOutWithV0;
  defm vmsbc : CrampBinaryMaskOut;

  defm vand : CrampBinaryAAX;
  defm vor : CrampBinaryAAX;
  defm vxor : CrampBinaryAAX;

  defm vsll : CrampBinaryAAShift;
  defm vsrl : CrampBinaryAAShift;
  defm vsra : CrampBinaryAAShift;

  defm vnsrl : CrampBinaryABShift;
  defm vnsra : CrampBinaryABShift;

  defm vmseq : CrampCompare;
  defm vmsne : CrampCompare;
  defm vmsltu : CrampCompare;
  defm vmslt : CrampCompare;
  defm vmsleu : CrampCompare;
  defm vmsle : CrampCompare;
  defm vmsgtu : CrampCompare;
  defm vmsgt : CrampCompare;
  defm vmsgeu : CrampCompare;
  defm vmsge : CrampCompare;

  defm vminu : CrampBinaryAAX;
  defm vmin : CrampBinaryAAX;
  defm vmaxu : CrampBinaryAAX;
  defm vmax : CrampBinaryAAX;

  defm vmul : CrampBinaryAAX;
  defm vmulh : CrampBinaryAAX;
  defm vmulhu : CrampBinaryAAX;
  defm vmulhsu : CrampBinaryAAX;

  defm vdivu : CrampBinaryAAX;
  defm vdiv : CrampBinaryAAX;
  defm vremu : CrampBinaryAAX;
  defm vrem : CrampBinaryAAX;

  defm vwmul : CrampBinaryABX;
  defm vwmulu : CrampBinaryABX;
  defm vwmulsu : CrampBinaryABX;

  defm vmacc : CrampTernaryAAXA;
  defm vnmsac : CrampTernaryAAXA;
  defm vmadd : CrampTernaryAAXA;
  defm vnmsub : CrampTernaryAAXA;

  defm vwmaccu  : CrampTernaryWide;
  defm vwmacc   : CrampTernaryWide;
  defm vwmaccus : CrampTernaryWide;
  defm vwmaccsu : CrampTernaryWide;

  defm vfadd : CrampBinaryAAX;
  defm vfsub : CrampBinaryAAX;
  defm vfrsub : CrampBinaryAAX;

  defm vfwadd : CrampBinaryABX;
  defm vfwsub : CrampBinaryABX;
  defm vfwadd_w : CrampBinaryAAX;
  defm vfwsub_w : CrampBinaryAAX;

  defm vsaddu : CrampSaturatingBinaryAAX;
  defm vsadd : CrampSaturatingBinaryAAX;
  defm vssubu : CrampSaturatingBinaryAAX;
  defm vssub : CrampSaturatingBinaryAAX;

  defm vmerge : CrampBinaryWithV0;

  // Output: (vector)
  // Input: (passthru, vector_in, vl)
  def int_cramp_vmv_v_v : Intrinsic<[llvm_anyvector_ty],
                                    [LLVMMatchType<0>, LLVMMatchType<0>,
                                     llvm_anyint_ty],
                                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }
  // Output: (vector)
  // Input: (passthru, scalar, vl)
  def int_cramp_vmv_v_x : Intrinsic<[llvm_anyint_ty],
                                    [LLVMMatchType<0>, LLVMVectorElementType<0>,
                                     llvm_anyint_ty],
                                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }
  // Output: (vector)
  // Input: (passthru, scalar, vl)
  def int_cramp_vfmv_v_f : Intrinsic<[llvm_anyfloat_ty],
                                     [LLVMMatchType<0>, LLVMVectorElementType<0>,
                                      llvm_anyint_ty],
                                     [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }

  def int_cramp_vmv_x_s : Intrinsic<[LLVMVectorElementType<0>],
                                    [llvm_anyint_ty],
                                    [IntrNoMem]>, CrampVIntrinsic;
  def int_cramp_vmv_s_x : Intrinsic<[llvm_anyint_ty],
                                    [LLVMMatchType<0>, LLVMVectorElementType<0>,
                                     llvm_anyint_ty],
                                    [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }

  def int_cramp_vfmv_f_s : Intrinsic<[LLVMVectorElementType<0>],
                                     [llvm_anyfloat_ty],
                                     [IntrNoMem]>, CrampVIntrinsic;
  def int_cramp_vfmv_s_f : Intrinsic<[llvm_anyfloat_ty],
                                     [LLVMMatchType<0>, LLVMVectorElementType<0>,
                                      llvm_anyint_ty],
                                     [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }

  defm vfmul : CrampBinaryAAX;
  defm vfdiv : CrampBinaryAAX;
  defm vfrdiv : CrampBinaryAAX;

  defm vfwmul : CrampBinaryABX;

  defm vfmacc : CrampTernaryAAXA;
  defm vfnmacc : CrampTernaryAAXA;
  defm vfmsac : CrampTernaryAAXA;
  defm vfnmsac : CrampTernaryAAXA;
  defm vfmadd : CrampTernaryAAXA;
  defm vfnmadd : CrampTernaryAAXA;
  defm vfmsub : CrampTernaryAAXA;
  defm vfnmsub : CrampTernaryAAXA;

  defm vfwmacc : CrampTernaryWide;
  defm vfwnmacc : CrampTernaryWide;
  defm vfwmsac : CrampTernaryWide;
  defm vfwnmsac : CrampTernaryWide;

  defm vfsqrt : CrampUnaryAA;
  defm vfrsqrt7 : CrampUnaryAA;
  defm vfrec7 : CrampUnaryAA;

  defm vfmin : CrampBinaryAAX;
  defm vfmax : CrampBinaryAAX;

  defm vfsgnj : CrampBinaryAAX;
  defm vfsgnjn : CrampBinaryAAX;
  defm vfsgnjx : CrampBinaryAAX;

  defm vfclass : CrampClassify;

  defm vfmerge : CrampBinaryWithV0;

  defm vslideup : CrampRVVSlide;
  defm vslidedown : CrampRVVSlide;

  defm vslide1up : CrampBinaryAAX;
  defm vslide1down : CrampBinaryAAX;
  defm vfslide1up : CrampBinaryAAX;
  defm vfslide1down : CrampBinaryAAX;

  defm vrgather_vv : CrampRGatherVV;
  defm vrgather_vx : CrampRGatherVX;
  defm vrgatherei16_vv : CrampRGatherEI16VV;

  def "int_cramp_vcompress" : CrampCompress;

  defm vaaddu : CrampSaturatingBinaryAAX;
  defm vaadd : CrampSaturatingBinaryAAX;
  defm vasubu : CrampSaturatingBinaryAAX;
  defm vasub : CrampSaturatingBinaryAAX;

  defm vsmul : CrampSaturatingBinaryAAX;

  defm vssrl : CrampSaturatingBinaryAAShift;
  defm vssra : CrampSaturatingBinaryAAShift;

  defm vnclipu : CrampSaturatingBinaryABShift;
  defm vnclip : CrampSaturatingBinaryABShift;

  defm vmfeq : CrampCompare;
  defm vmfne : CrampCompare;
  defm vmflt : CrampCompare;
  defm vmfle : CrampCompare;
  defm vmfgt : CrampCompare;
  defm vmfge : CrampCompare;

  defm vredsum : CrampReduction;
  defm vredand : CrampReduction;
  defm vredor : CrampReduction;
  defm vredxor : CrampReduction;
  defm vredminu : CrampReduction;
  defm vredmin : CrampReduction;
  defm vredmaxu : CrampReduction;
  defm vredmax : CrampReduction;

  defm vwredsumu : CrampReduction;
  defm vwredsum : CrampReduction;

  defm vfredosum : CrampReduction;
  defm vfredusum : CrampReduction;
  defm vfredmin : CrampReduction;
  defm vfredmax : CrampReduction;

  defm vfwredusum : CrampReduction;
  defm vfwredosum : CrampReduction;

  def int_cramp_vmand: CrampBinaryAAAUnMasked;
  def int_cramp_vmnand: CrampBinaryAAAUnMasked;
  def int_cramp_vmandn: CrampBinaryAAAUnMasked;
  def int_cramp_vmxor: CrampBinaryAAAUnMasked;
  def int_cramp_vmor: CrampBinaryAAAUnMasked;
  def int_cramp_vmnor: CrampBinaryAAAUnMasked;
  def int_cramp_vmorn: CrampBinaryAAAUnMasked;
  def int_cramp_vmxnor: CrampBinaryAAAUnMasked;
  def int_cramp_vmclr : CrampNullaryIntrinsic;
  def int_cramp_vmset : CrampNullaryIntrinsic;

  defm vcpop : CrampMaskedUnarySOut;
  defm vfirst : CrampMaskedUnarySOut;
  defm vmsbf : CrampMaskedUnaryMOut;
  defm vmsof : CrampMaskedUnaryMOut;
  defm vmsif : CrampMaskedUnaryMOut;

  defm vfcvt_xu_f_v : CrampConversion;
  defm vfcvt_x_f_v : CrampConversion;
  defm vfcvt_rtz_xu_f_v : CrampConversion;
  defm vfcvt_rtz_x_f_v : CrampConversion;
  defm vfcvt_f_xu_v : CrampConversion;
  defm vfcvt_f_x_v : CrampConversion;

  defm vfwcvt_f_xu_v : CrampConversion;
  defm vfwcvt_f_x_v : CrampConversion;
  defm vfwcvt_xu_f_v : CrampConversion;
  defm vfwcvt_x_f_v : CrampConversion;
  defm vfwcvt_rtz_xu_f_v : CrampConversion;
  defm vfwcvt_rtz_x_f_v : CrampConversion;
  defm vfwcvt_f_f_v : CrampConversion;

  defm vfncvt_f_xu_w : CrampConversion;
  defm vfncvt_f_x_w : CrampConversion;
  defm vfncvt_xu_f_w : CrampConversion;
  defm vfncvt_x_f_w : CrampConversion;
  defm vfncvt_rtz_xu_f_w : CrampConversion;
  defm vfncvt_rtz_x_f_w : CrampConversion;
  defm vfncvt_f_f_w : CrampConversion;
  defm vfncvt_rod_f_f_w : CrampConversion;

  // Output: (vector)
  // Input: (passthru, mask type input, vl)
  def int_cramp_viota : Intrinsic<[llvm_anyvector_ty],
                                  [LLVMMatchType<0>,
                                   LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                                   llvm_anyint_ty],
                                  [IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }
  // Output: (vector)
  // Input: (maskedoff, mask type vector_in, mask, vl, policy)
  def int_cramp_viota_mask : Intrinsic<[llvm_anyvector_ty],
                                       [LLVMMatchType<0>,
                                        LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                                        LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                                        llvm_anyint_ty, LLVMMatchType<1>],
                                       [ImmArg<ArgIndex<4>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 3;
  }
  // Output: (vector)
  // Input: (passthru, vl)
  def int_cramp_vid : CrampID;

  // Output: (vector)
  // Input: (maskedoff, mask, vl, policy)
  def int_cramp_vid_mask : Intrinsic<[llvm_anyvector_ty],
                                     [LLVMMatchType<0>,
                                      LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>,
                                      llvm_anyint_ty, LLVMMatchType<1>],
                                     [ImmArg<ArgIndex<3>>, IntrNoMem]>, CrampVIntrinsic {
    let VLOperand = 2;
  }

  foreach nf = [2, 3, 4, 5, 6, 7, 8] in {
    defm vlseg # nf : CrampUSSegLoad<nf>;
    defm vlseg # nf # ff : CrampUSSegLoadFF<nf>;
    defm vlsseg # nf : CrampSSegLoad<nf>;
    defm vloxseg # nf : CrampISegLoad<nf>;
    defm vluxseg # nf : CrampISegLoad<nf>;
    defm vsseg # nf : CrampUSSegStore<nf>;
    defm vssseg # nf : CrampSSegStore<nf>;
    defm vsoxseg # nf : CrampISegStore<nf>;
    defm vsuxseg # nf : CrampISegStore<nf>;
  }

  // Strided loads/stores for fixed vectors.
  def int_cramp_masked_strided_load
        : Intrinsic<[llvm_anyvector_ty],
                    [LLVMMatchType<0>, llvm_anyptr_ty,
                     llvm_anyint_ty, LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>],
                    [NoCapture<ArgIndex<1>>, IntrReadMem]>;
  def int_cramp_masked_strided_store
        : Intrinsic<[],
                    [llvm_anyvector_ty, llvm_anyptr_ty,
                     llvm_anyint_ty, LLVMScalarOrSameVectorWidth<0, llvm_i1_ty>],
                    [NoCapture<ArgIndex<1>>, IntrWriteMem]>;

  // Segment loads for fixed vectors.
  foreach nf = [2, 3, 4, 5, 6, 7, 8] in {
    def int_cramp_seg # nf # _load
          : Intrinsic<!listconcat([llvm_anyvector_ty], !listsplat(LLVMMatchType<0>,
                                !add(nf, -1))),
                      [llvm_anyptr_ty, llvm_anyint_ty],
                      [NoCapture<ArgIndex<0>>, IntrReadMem]>;
  }

} // TargetPrefix = "cramp"

//===----------------------------------------------------------------------===//
// Scalar Cryptography
//
// These intrinsics will lower directly into the corresponding instructions
// added by the scalar cyptography extension, if the extension is present.

let TargetPrefix = "cramp" in {

class CrampScalarCryptoGprIntrinsicAny
    : Intrinsic<[llvm_anyint_ty],
                [LLVMMatchType<0>],
                [IntrNoMem, IntrSpeculatable]>;

class CrampScalarCryptoByteSelect32
    : Intrinsic<[llvm_i32_ty],
                [llvm_i32_ty, llvm_i32_ty, llvm_i8_ty],
                [IntrNoMem, IntrWillReturn, IntrSpeculatable,
                 ImmArg<ArgIndex<2>>]>;

class CrampScalarCryptoGprGprIntrinsic32
    : Intrinsic<[llvm_i32_ty],
                [llvm_i32_ty, llvm_i32_ty],
                [IntrNoMem, IntrWillReturn, IntrSpeculatable]>;

class CrampScalarCryptoGprGprIntrinsic64
    : Intrinsic<[llvm_i64_ty],
                [llvm_i64_ty, llvm_i64_ty],
                [IntrNoMem, IntrWillReturn, IntrSpeculatable]>;

class CrampScalarCryptoGprIntrinsic64
    : Intrinsic<[llvm_i64_ty],
                [llvm_i64_ty],
                [IntrNoMem, IntrWillReturn, IntrSpeculatable]>;

class CrampScalarCryptoByteSelectAny
    : Intrinsic<[llvm_anyint_ty],
                [LLVMMatchType<0>, LLVMMatchType<0>, llvm_i8_ty],
                [IntrNoMem, IntrSpeculatable, IntrWillReturn,
                 ImmArg<ArgIndex<2>>]>;

// Zknd
def int_cramp_aes32dsi  : CrampScalarCryptoByteSelect32;
def int_cramp_aes32dsmi : CrampScalarCryptoByteSelect32;

def int_cramp_aes64ds   : CrampScalarCryptoGprGprIntrinsic64;
def int_cramp_aes64dsm  : CrampScalarCryptoGprGprIntrinsic64;

def int_cramp_aes64im   : CrampScalarCryptoGprIntrinsic64;

// Zkne
def int_cramp_aes32esi  : CrampScalarCryptoByteSelect32;
def int_cramp_aes32esmi : CrampScalarCryptoByteSelect32;

def int_cramp_aes64es   : CrampScalarCryptoGprGprIntrinsic64;
def int_cramp_aes64esm  : CrampScalarCryptoGprGprIntrinsic64;

// Zknd & Zkne
def int_cramp_aes64ks2  : CrampScalarCryptoGprGprIntrinsic64;
def int_cramp_aes64ks1i : Intrinsic<[llvm_i64_ty], [llvm_i64_ty, llvm_i32_ty],
                                    [IntrNoMem, IntrSpeculatable,
                                     IntrWillReturn, ImmArg<ArgIndex<1>>]>;

// Zknh
def int_cramp_sha256sig0 : CrampScalarCryptoGprIntrinsicAny;
def int_cramp_sha256sig1 : CrampScalarCryptoGprIntrinsicAny;
def int_cramp_sha256sum0 : CrampScalarCryptoGprIntrinsicAny;
def int_cramp_sha256sum1 : CrampScalarCryptoGprIntrinsicAny;

def int_cramp_sha512sig0l : CrampScalarCryptoGprGprIntrinsic32;
def int_cramp_sha512sig0h : CrampScalarCryptoGprGprIntrinsic32;
def int_cramp_sha512sig1l : CrampScalarCryptoGprGprIntrinsic32;
def int_cramp_sha512sig1h : CrampScalarCryptoGprGprIntrinsic32;
def int_cramp_sha512sum0r : CrampScalarCryptoGprGprIntrinsic32;
def int_cramp_sha512sum1r : CrampScalarCryptoGprGprIntrinsic32;

def int_cramp_sha512sig0 : CrampScalarCryptoGprIntrinsic64;
def int_cramp_sha512sig1 : CrampScalarCryptoGprIntrinsic64;
def int_cramp_sha512sum0 : CrampScalarCryptoGprIntrinsic64;
def int_cramp_sha512sum1 : CrampScalarCryptoGprIntrinsic64;

// Zksed
def int_cramp_sm4ks      : CrampScalarCryptoByteSelectAny;
def int_cramp_sm4ed      : CrampScalarCryptoByteSelectAny;

// Zksh
def int_cramp_sm3p0      : CrampScalarCryptoGprIntrinsicAny;
def int_cramp_sm3p1      : CrampScalarCryptoGprIntrinsicAny;
} // TargetPrefix = "cramp"
