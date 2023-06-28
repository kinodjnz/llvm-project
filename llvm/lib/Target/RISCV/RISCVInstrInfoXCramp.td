//===-- RISCVInstrInfoXCramp.td ----------------------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// This file describes the vendor extensions of cramp instructions.
///
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Operand and SDNode transformation definitions.
//===----------------------------------------------------------------------===//

def uimm4_lsb0 : Operand<XLenVT>,
                 ImmLeaf<XLenVT, [{return isShiftedUInt<3, 1>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<4, "Lsb0">;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeUImmOperand<4>";
  let OperandType = "OPERAND_UIMM4_LSB0";
  let OperandNamespace = "RISCVOp";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (!MCOp.evaluateAsConstantImm(Imm))
      return false;
    return isShiftedUInt<3, 1>(Imm);
  }];
}

def uimm5_lsb0 : Operand<XLenVT>,
                 ImmLeaf<XLenVT, [{return isShiftedUInt<4, 1>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<5, "Lsb0">;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeUImmOperand<5>";
  let OperandType = "OPERAND_UIMM5_LSB0";
  let OperandNamespace = "RISCVOp";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (!MCOp.evaluateAsConstantImm(Imm))
      return false;
    return isShiftedUInt<4, 1>(Imm);
  }];
}

//===----------------------------------------------------------------------===//
// Instruction Class Templates
//===----------------------------------------------------------------------===//

let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
class RVZcArith_r<bits<5> funct5, string OpcodeStr> :
  RVInst16CSZN<0b100111, funct5, 0b01, (outs GPRC:$rd_wb), (ins GPRC:$rd),
               OpcodeStr, "$rd"> {
  let Constraints = "$rd = $rd_wb";
}

//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//

let Predicates = [HasVendorXCramp, HasStdExtZbb] in {
def C_ZEXT_H  : RVZcArith_r<0b11010 , "c.zext.h">,
                Sched<[WriteIALU, ReadIALU]>;
def C_SEXT_B  : RVZcArith_r<0b11001 , "c.sext.b">,
                Sched<[WriteIALU, ReadIALU]>;
def C_SEXT_H  : RVZcArith_r<0b11011 , "c.sext.h">,
                Sched<[WriteIALU, ReadIALU]>;
}

let Predicates = [HasVendorXCramp] in
def C_ZEXT_B  : RVZcArith_r<0b11000 , "c.zext.b">,
                Sched<[WriteIALU, ReadIALU]>;

let Predicates = [HasVendorXCramp, HasStdExtMOrZmmul] in
def C_MUL     : CS_ALU<0b100111, 0b10, "c.mul", GPRC>,
                Sched<[WriteIMul, ReadIMul, ReadIMul]>;

let Predicates = [HasVendorXCramp] in {
def C_NOT : RVZcArith_r<0b11101 , "c.not">,
            Sched<[WriteIALU, ReadIALU]>;
def C_NEG : RVZcArith_r<0b11110 , "c.neg">,
            Sched<[WriteIALU, ReadIALU]>;
}

let DecoderNamespace = "Cramp", Predicates = [HasVendorXCramp] in {
def C_LBU : CLoadByte_ri<0b011, "c.lbu", GPRC, uimm5>,
            Sched<[WriteLDB, ReadMemBase]> {
  bits<5> imm;
  let Inst{12} = imm{0};
  let Inst{11-10} = imm{4,3};
  let Inst{6-5} = imm{2,1};
}

def C_LB : CLoadByte_ri<0b001, "c.lb", GPRC, uimm5>,
           Sched<[WriteLDB, ReadMemBase]> {
  bits<5> imm;
  let Inst{12} = imm{0};
  let Inst{11-10} = imm{4,3};
  let Inst{6-5} = imm{2,1};
}

def C_LHU : CLoadHalf_ri<0b001, "c.lhu", GPRC, uimm4_lsb0>,
            Sched<[WriteLDH, ReadMemBase]> {
  bits<4> imm;
  let Inst{12-11} = 0b01;
  let Inst{10} = imm{3};
  let Inst{6-5} = imm{2,1};
}

def C_LH : CLoadHalf_ri<0b001, "c.lh", GPRC, uimm4_lsb0>,
           Sched<[WriteLDH, ReadMemBase]> {
  bits<4> imm;
  let Inst{12-11} = 0b00;
  let Inst{10} = imm{3};
  let Inst{6-5} = imm{2,1};
}

def C_SB : CStoreBH_rri<0b011, "c.sb", GPRC, uimm5>,
           Sched<[WriteSTB, ReadStoreData, ReadMemBase]> {
  bits<5> imm;
  let Inst{12} = imm{0};
  let Inst{11-10} = imm{4,3};
  let Inst{6-5} = imm{2,1};
}

def C_SH : CStoreBH_rri<0b001, "c.sh", GPRC, uimm4_lsb0>, 
           Sched<[WriteSTH, ReadStoreData, ReadMemBase]> {
  bits<4> imm;
  let Inst{12-11} = 0b10;
  let Inst{10} = imm{3};
  let Inst{6-5} = imm{2,1};
}

def C_SB0 : CStoreZero_ri<0b001, "c.sb0", uimm4>, 
           Sched<[WriteSTB, ReadStoreData, ReadMemBase]> {
  bits<4> imm;
  let Inst{12-11} = 0b11;
  let Inst{10} = imm{3};
  let Inst{6-5} = imm{2,1};
  let Inst{4} = imm{0};
  let Inst{3-2} = 0b01;
}

def C_SH0 : CStoreZero_ri<0b001, "c.sh0", uimm5_lsb0>, 
           Sched<[WriteSTH, ReadStoreData, ReadMemBase]> {
  bits<5> imm;
  let Inst{12-11} = 0b11;
  let Inst{10} = imm{3};
  let Inst{6-5} = imm{2,1};
  let Inst{4} = imm{4};
  let Inst{3-2} = 0b11;
}

def C_SW0 : CStoreZero_ri<0b001, "c.sw0", uimm7_lsb00>, 
           Sched<[WriteSTW, ReadStoreData, ReadMemBase]> {
  bits<7> imm;
  let Inst{12-11} = 0b11;
  let Inst{10} = imm{3};
  let Inst{6} = imm{2};
  let Inst{5} = imm{6};
  let Inst{4-3} = imm{5,4};
  let Inst{2} = 0b0;
}
}

let Predicates = [HasVendorXCramp, HasStdExtMOrZmmul] in{
def : CompressPat<(MUL GPRC:$rs1, GPRC:$rs1, GPRC:$rs2),
                  (C_MUL GPRC:$rs1, GPRC:$rs2)>;
let isCompressOnly = true in
def : CompressPat<(MUL GPRC:$rs1, GPRC:$rs2, GPRC:$rs1),
                  (C_MUL GPRC:$rs1, GPRC:$rs2)>;
} // Predicates = [HasVendorXCramp, HasStdExtMOrZmmul]

let Predicates = [HasVendorXCramp, HasStdExtZbb] in{
def : CompressPat<(SEXT_B GPRC:$rs1, GPRC:$rs1),
                  (C_SEXT_B GPRC:$rs1, GPRC:$rs1)>;
def : CompressPat<(SEXT_H GPRC:$rs1, GPRC:$rs1),
                  (C_SEXT_H GPRC:$rs1, GPRC:$rs1)>;
} // Predicates = [HasVendorXCramp, HasStdExtZbb]

let Predicates = [HasVendorXCramp, HasStdExtZbb] in{
def : CompressPat<(ZEXT_H_RV32 GPRC:$rs1, GPRC:$rs1),
                  (C_ZEXT_H GPRC:$rs1, GPRC:$rs1)>;
def : CompressPat<(ZEXT_H_RV64 GPRC:$rs1, GPRC:$rs1),
                  (C_ZEXT_H GPRC:$rs1, GPRC:$rs1)>;
} // Predicates = [HasVendorXCramp, HasStdExtZbb]

let Predicates = [HasVendorXCramp] in{
def : CompressPat<(ANDI GPRC:$rs1, GPRC:$rs1, 255),
                  (C_ZEXT_B GPRC:$rs1, GPRC:$rs1)>;
} // Predicates = [HasVendorXCramp]

let Predicates = [HasVendorXCramp] in{
def : CompressPat<(XORI GPRC:$rs1, GPRC:$rs1, -1),
                  (C_NOT GPRC:$rs1, GPRC:$rs1)>;
}

let Predicates = [HasVendorXCramp] in{
def : CompressPat<(SUB GPRC:$rs1, X0, GPRC:$rs1),
                  (C_NEG GPRC:$rs1, GPRC:$rs1)>;
}

let Predicates = [HasVendorXCramp] in{
def : CompressPat<(LBU GPRC:$rd, GPRCMem:$rs1, uimm5:$imm),
                  (C_LBU GPRC:$rd, GPRCMem:$rs1, uimm5:$imm)>;
def : CompressPat<(LB GPRC:$rd, GPRCMem:$rs1, uimm5:$imm),
                  (C_LB GPRC:$rd, GPRCMem:$rs1, uimm5:$imm)>;
def : CompressPat<(LHU GPRC:$rd, GPRCMem:$rs1, uimm4_lsb0:$imm),
                  (C_LHU GPRC:$rd, GPRCMem:$rs1, uimm4_lsb0:$imm)>;
def : CompressPat<(LH GPRC:$rd, GPRCMem:$rs1, uimm4_lsb0:$imm),
                  (C_LH GPRC:$rd, GPRCMem:$rs1, uimm4_lsb0:$imm)>;
def : CompressPat<(SB GPRC:$rs2, GPRCMem:$rs1, uimm5:$imm),
                  (C_SB GPRC:$rs2, GPRCMem:$rs1, uimm5:$imm)>;
def : CompressPat<(SH GPRC:$rs2, GPRCMem:$rs1, uimm4_lsb0:$imm),
                  (C_SH GPRC:$rs2, GPRCMem:$rs1, uimm4_lsb0:$imm)>;
def : CompressPat<(SB X0, GPRCMem:$rs1, uimm4:$imm),
                  (C_SB0 GPRCMem:$rs1, uimm4:$imm)>;
def : CompressPat<(SH X0, GPRCMem:$rs1, uimm5_lsb0:$imm),
                  (C_SH0 GPRCMem:$rs1, uimm5_lsb0:$imm)>;
def : CompressPat<(SW X0, GPRCMem:$rs1, uimm7_lsb00:$imm),
                  (C_SW0 GPRCMem:$rs1, uimm7_lsb00:$imm)>;
}// Predicates = [HasVendorXCramp]


//===----------------------------------------------------------------------===//
// Pseudo Instructions
//===----------------------------------------------------------------------===//

let Predicates = [HasVendorXCramp] in {
def : InstAlias<"c.lbu $rd, (${rs1})",(C_LBU GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"c.lb $rd, (${rs1})", (C_LB GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"c.lhu $rd, (${rs1})",(C_LHU GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"c.lh $rd, (${rs1})", (C_LH GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"c.sb $rd, (${rs1})", (C_SB GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"c.sh $rd, (${rs1})", (C_SH GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"c.sb0 (${rs1})",(C_SB0 GPRC:$rs1, 0)>;
def : InstAlias<"c.sh0 (${rs1})",(C_SH0 GPRC:$rs1, 0)>;
def : InstAlias<"c.sw0 (${rs1})",(C_SW0 GPRC:$rs1, 0)>;
}
