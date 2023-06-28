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

def simm7_lsb00nonzero : Operand<XLenVT>,
                         ImmLeaf<XLenVT,
                         [{return isShiftedInt<5, 2>(Imm) && (Imm != 0);}]> {
  let ParserMatchClass = SImmAsmOperand<7, "Lsb00NonZero">;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeSImmNonZeroOperand<7>";
  let OperandType = "OPERAND_SIMM7_LSB00_NONZERO";
  let OperandNamespace = "RISCVOp";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (!MCOp.evaluateAsConstantImm(Imm))
      return false;
    return isShiftedInt<5, 2>(Imm) && (Imm != 0);
  }];
}

def simm3_not0nor4 : Operand<XLenVT>,
                     ImmLeaf<XLenVT,
                     [{return isInt<3>(Imm) && (Imm != 0) && (Imm != 4);}]> {
  let ParserMatchClass = SImmAsmOperand<3, "Not0Nor4">;
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeSImmNot0Nor4Operand<3>";
  let OperandType = "OPERAND_SIMM3_NOT0NOR4";
  let OperandNamespace = "RISCVOp";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (!MCOp.evaluateAsConstantImm(Imm))
      return false;
    return isInt<3>(Imm) && (Imm != 0) && (Imm != 4);
  }];
}

def simm6_lsb0 : Operand<OtherVT>,
                 ImmLeaf<XLenVT, [{return isShiftedInt<5, 1>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<6, "Lsb0">;
  let PrintMethod = "printBranchOperand";
  let EncoderMethod = "getImmOpValueAsr1";
  let DecoderMethod = "decodeSImmOperandAndLsl1<6>";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (MCOp.evaluateAsConstantImm(Imm))
      return isShiftedInt<5, 1>(Imm);
    return MCOp.isBareSymbolRef();

  }];
  let OperandType = "OPERAND_PCREL";
}

class UImm8Operand : Operand<XLenVT>,
                     ImmLeaf<XLenVT, [{return isUInt<8>(Imm);}]> {
  let EncoderMethod = "getImmOpValue";
  let DecoderMethod = "decodeUImmOperand<8>";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (MCOp.evaluateAsConstantImm(Imm))
      return isUInt<8>(Imm);
    return MCOp.isBareSymbolRef();
  }];
  let OperandType = "OPERAND_UIMM8";
  let OperandNamespace = "RISCVOp";
}

def uimm8_auipc : UImm8Operand {
  let ParserMatchClass = UImmAsmOperand<8, "C_AUIPC">;
}

def uimm_bfi_len : Operand<XLenVT>,
                   ImmLeaf<XLenVT,
                   [{return (isUInt<3>(Imm) && (Imm != 7)) || (Imm == 8);}]> {
  let ParserMatchClass = UImmAsmOperand<5, "BFILen">;
  let EncoderMethod = "encodeUImmBFILen";
  let DecoderMethod = "decodeUImmBFILen<3>";
  let OperandType = "OPERAND_UIMM3_BFI_LEN";
  let OperandNamespace = "RISCVOp";
  let MCOperandPredicate = [{
    int64_t Imm;
    if (!MCOp.evaluateAsConstantImm(Imm))
      return false;
    return (isUInt<3>(Imm) && (Imm != 7)) || (Imm == 8);
  }];
}

//===----------------------------------------------------------------------===//
// Instruction Class Templates
//===----------------------------------------------------------------------===//

let hasSideEffects = 0, mayLoad = 1, mayStore = 0 in
class CLoadByte_ri<bits<3> funct3, string OpcodeStr,
                   RegisterClass cls, DAGOperand opnd>
    : RVInst16CL<funct3, 0b00, (outs cls:$rd), (ins GPRCMem:$rs1, opnd:$imm),
                 OpcodeStr, "$rd, ${imm}(${rs1})">;

let hasSideEffects = 0, mayLoad = 1, mayStore = 0 in
class CLoadHalf_ri<bits<3> funct3, string OpcodeStr,
                   RegisterClass cls, DAGOperand opnd>
    : RVInst16CL<funct3, 0b10, (outs cls:$rd), (ins GPRCMem:$rs1, opnd:$imm),
                 OpcodeStr, "$rd, ${imm}(${rs1})">;

let hasSideEffects = 0, mayLoad = 0, mayStore = 1 in
class CStoreBH_rri<bits<3> funct3, string OpcodeStr,
                 RegisterClass cls, DAGOperand opnd>
    : RVInst16CS<funct3, 0b10, (outs), (ins cls:$rs2,GPRCMem:$rs1, opnd:$imm),
                 OpcodeStr, "$rs2, ${imm}(${rs1})">;

let hasSideEffects = 0, mayLoad = 0, mayStore = 1 in
class CStoreZero_ri<bits<3> funct3, string OpcodeStr, DAGOperand opnd>
    : RVInst16CCS0<funct3, 0b10, (outs), (ins GPRCMem:$rs1, opnd:$imm),
                   OpcodeStr, "${imm}(${rs1})">;

let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
class CZcArith_r<bits<5> funct5, string OpcodeStr> :
  RVInst16CU<0b100111, funct5, 0b01, (outs GPRC:$rd_wb), (ins GPRC:$rd),
             OpcodeStr, "$rd"> {
  let Constraints = "$rd = $rd_wb";
}

let hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
class CBranch2<bits<3> funct3, string OpcodeStr>
    : RVInst16CCB<funct3, 0b00, (outs), (ins GPRC:$rs1, GPRC:$rs2, simm6_lsb0:$imm),
                  OpcodeStr, "$rs1, $rs2, $imm"> {
  bits<6> imm;
  let isBranch = 1;
  let isTerminator = 1;
  let Inst{12-10} = imm{4-2};
  let Inst{6-5} = imm{1-0};
}

//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//

let DecoderNamespace = "Cramp", Predicates = [HasVendorXCramp, HasStdExtZbb] in {
def CC_ZEXT_H  : CZcArith_r<0b11010 , "cc.zext.h">,
                 Sched<[WriteIALU, ReadIALU]>;
def CC_SEXT_B  : CZcArith_r<0b11001 , "cc.sext.b">,
                 Sched<[WriteIALU, ReadIALU]>;
def CC_SEXT_H  : CZcArith_r<0b11011 , "cc.sext.h">,
                 Sched<[WriteIALU, ReadIALU]>;
}

let DecoderNamespace = "Cramp", Predicates = [HasVendorXCramp] in
def CC_ZEXT_B  : CZcArith_r<0b11000 , "cc.zext.b">,
                 Sched<[WriteIALU, ReadIALU]>;

let DecoderNamespace = "Cramp", Predicates = [HasVendorXCramp, HasStdExtZmmul] in
def CC_MUL     : CA_ALU<0b100111, 0b10, "cc.mul">,
                 Sched<[WriteIMul, ReadIMul, ReadIMul]>;

let DecoderNamespace = "Cramp", Predicates = [HasVendorXCramp] in {
def CC_NOT : CZcArith_r<0b11101 , "cc.not">,
             Sched<[WriteIALU, ReadIALU]>;
def CC_NEG : CZcArith_r<0b11110 , "cc.neg">,
             Sched<[WriteIALU, ReadIALU]>;
}

let DecoderNamespace = "Cramp", Predicates = [HasVendorXCramp] in {
def CC_LBU : CLoadByte_ri<0b011, "cc.lbu", GPRC, uimm5>,
             Sched<[WriteLDB, ReadMemBase]> {
  bits<5> imm;
  let Inst{12} = imm{0};
  let Inst{11-10} = imm{4,3};
  let Inst{6-5} = imm{2,1};
}

def CC_LB : CLoadByte_ri<0b001, "cc.lb", GPRC, uimm5>,
            Sched<[WriteLDB, ReadMemBase]> {
  bits<5> imm;
  let Inst{12} = imm{0};
  let Inst{11-10} = imm{4,3};
  let Inst{6-5} = imm{2,1};
}

def CC_LHU : CLoadHalf_ri<0b001, "cc.lhu", GPRC, uimm4_lsb0>,
             Sched<[WriteLDH, ReadMemBase]> {
  bits<4> imm;
  let Inst{12-11} = 0b01;
  let Inst{10} = imm{3};
  let Inst{6-5} = imm{2,1};
}

def CC_LH : CLoadHalf_ri<0b001, "cc.lh", GPRC, uimm4_lsb0>,
            Sched<[WriteLDH, ReadMemBase]> {
  bits<4> imm;
  let Inst{12-11} = 0b00;
  let Inst{10} = imm{3};
  let Inst{6-5} = imm{2,1};
}

def CC_SB : CStoreBH_rri<0b011, "cc.sb", GPRC, uimm5>,
            Sched<[WriteSTB, ReadStoreData, ReadMemBase]> {
  bits<5> imm;
  let Inst{12} = imm{0};
  let Inst{11-10} = imm{4,3};
  let Inst{6-5} = imm{2,1};
}

def CC_SH : CStoreBH_rri<0b001, "cc.sh", GPRC, uimm4_lsb0>, 
            Sched<[WriteSTH, ReadStoreData, ReadMemBase]> {
  bits<4> imm;
  let Inst{12-11} = 0b10;
  let Inst{10} = imm{3};
  let Inst{6-5} = imm{2,1};
}

def CC_SB0 : CStoreZero_ri<0b001, "cc.sb0", uimm4>, 
             Sched<[WriteSTB, ReadStoreData, ReadMemBase]> {
  bits<4> imm;
  let Inst{12-11} = 0b11;
  let Inst{10} = imm{3};
  let Inst{6-5} = imm{2,1};
  let Inst{4} = imm{0};
  let Inst{3-2} = 0b01;
}

def CC_SH0 : CStoreZero_ri<0b001, "cc.sh0", uimm5_lsb0>, 
             Sched<[WriteSTH, ReadStoreData, ReadMemBase]> {
  bits<5> imm;
  let Inst{12-11} = 0b11;
  let Inst{10} = imm{3};
  let Inst{6-5} = imm{2,1};
  let Inst{4} = imm{4};
  let Inst{3-2} = 0b11;
}

def CC_SW0 : CStoreZero_ri<0b001, "cc.sw0", uimm7_lsb00>, 
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

let DecoderNamespace = "Cramp", Predicates = [HasVendorXCramp],
    hasSideEffects = 0, mayLoad = 0, mayStore = 0 in {
def C_ADDI2W : RVInst16CCR2<0b101, 0b10, (outs GPRC:$rd),
            (ins GPRC:$rs, simm7_lsb00nonzero:$imm),
            "c.addi2w", "$rd, $rs, $imm">,
             Sched<[WriteIALU, ReadIALU]> {
  bits<7> imm;
  let Inst{12} = imm{6};
  let Inst{11-10} = imm{4,3};
  let Inst{6} = imm{2};
  let Inst{5} = imm{5};
}

def C_ADDI2B : RVInst16CCR2<0b111, 0b10, (outs GPRC:$rd),
            (ins GPRC:$rs, simm3_not0nor4:$imm),
            "c.addi2b", "$rd, $rs, $imm">,
             Sched<[WriteIALU, ReadIALU]> {
  bits<3> imm;
  let Inst{12-10} = imm{2-0};
  let Inst{6-5} = 0b01;
}

def C_SEQZ : RVInst16CCR2<0b111, 0b10, (outs GPRC:$rd), (ins GPRC:$rs),
            "c.seqz", "$rd, $rs">,
             Sched<[WriteIALU, ReadIALU]> {
  let Inst{12-10} = 0b000;
  let Inst{6-5} = 0b01;
}

def C_SNEZ : RVInst16CCR2<0b111, 0b10, (outs GPRC:$rd), (ins GPRC:$rs),
            "c.snez", "$rd, $rs">,
             Sched<[WriteIALU, ReadIALU]> {
  let Inst{12-10} = 0b100;
  let Inst{6-5} = 0b01;
}

def C_ADD2 : RVInst16CCR3<0b111, 0b00, 0b10, (outs GPRC:$rd), (ins GPRC:$rs1, GPRC:$rs2),
            "c.add2", "$rd, $rs1, $rs2">,
             Sched<[WriteIALU, ReadIALU]>;
def C_SLT : RVInst16CCR3<0b111, 0b10, 0b10, (outs GPRC:$rd), (ins GPRC:$rs1, GPRC:$rs2),
            "c.slt", "$rd, $rs1, $rs2">,
             Sched<[WriteIALU, ReadIALU]>;
def C_SLTU : RVInst16CCR3<0b111, 0b11, 0b10, (outs GPRC:$rd), (ins GPRC:$rs1, GPRC:$rs2),
            "c.sltu", "$rd, $rs1, $rs2">,
             Sched<[WriteIALU, ReadIALU]>;
}

let DecoderNamespace = "Cramp", Predicates = [HasVendorXCramp] in {
def C_BEQ : CBranch2<0b100, "c.beq">, Sched<[WriteJmp]>;
def C_BNE : CBranch2<0b101, "c.bne">, Sched<[WriteJmp]>;
}

let DecoderNamespace = "Cramp", Predicates = [HasVendorXCramp],
    hasSideEffects = 0, mayLoad = 0, mayStore = 0 in
def C_AUIPC : RVInst16CIW<0b111, 0b00, (outs GPRC:$rd), (ins uimm8_auipc:$imm),
                          "c.auipc", "$rd, $imm">, Sched<[WriteIALU]> {
  let Inst{12-5} = imm{7-0};
}

let Predicates = [HasVendorXCramp] in {
def CMOV : RVBTernaryR<0b11, 0b101, OPC_OP, "cmov", "$rd, $rs2, $rs1, $rs3">,
           Sched<[WriteCMov, ReadCMov, ReadCMov, ReadCMov]>;
def FSL  : RVBTernaryR<0b10, 0b001, OPC_OP, "fsl", "$rd, $rs1, $rs3, $rs2">,
           Sched<[WriteFSReg, ReadFSReg, ReadFSReg, ReadFSReg]>;
def FSR  : RVBTernaryR<0b10, 0b101, OPC_OP, "fsr", "$rd, $rs1, $rs3, $rs2">,
           Sched<[WriteFSReg, ReadFSReg, ReadFSReg, ReadFSReg]>;
def FSRI : RVBTernaryImm5<0b10, 0b101, OPC_OP_IMM, "fsri",
                          "$rd, $rs1, $rs3, $shamt">,
           Sched<[WriteFSRImm, ReadFSRImm, ReadFSRImm]>;
def BSCTH : ALU_rr<0b0100101, 0b001, "bsct.h">,
            Sched<[WriteBscthReg, ReadBscthReg, ReadBscthReg]>;
def BFMP : RVBTernaryR<0b10, 0b000, OPC_OP_P, "bfmp", "$rd, $rs1, $rs3, $rs2">,
           Sched<[WriteBFReg, ReadBFReg, ReadBFReg, ReadBFReg]>;
def BFM  : RVBitFieldRLen<0b10, OPC_OP_P, "bfm",
                         "$rd, $rs1, $rs3, $rs2, $len">,
           Sched<[WriteBFRLen, ReadBFRLen, ReadBFRLen, ReadBFRLen]>;
def BFPP : RVBTernaryR<0b10, 0b001, OPC_OP_P, "bfpp", "$rd, $rs1, $rs3, $rs2">,
           Sched<[WriteBFReg, ReadBFReg, ReadBFReg, ReadBFReg]>;
def BFP  : RVBitFieldRLen<0b11, OPC_OP_P, "bfp",
                          "$rd, $rs1, $rs3, $rs2, $len">,
           Sched<[WriteBFRLen, ReadBFRLen, ReadBFRLen, ReadBFRLen]>;
} // Predicates = [HasVendorXCramp]

let DecoderNamespace = "Cramp", Predicates = [HasVendorXCramp] in {
def BFMI : RVBitFieldImm5Len<0b10, OPC_OP_V, "bfmi",
                             "$rd, $rs1, $rs3, $shamt, $len">,
           Sched<[WriteBFImm, ReadBFImm, ReadBFImm]>;
def BFPI : RVBitFieldImm5Len<0b11, OPC_OP_V, "bfpi",
                             "$rd, $rs1, $rs3, $shamt, $len">,
           Sched<[WriteBFImm, ReadBFImm, ReadBFImm]>;
} // Predicates = [HasVendorXCramp]

let Predicates = [HasVendorXCramp] in {
def ORC    : RVBUnary<0b0010100, 0b11111, 0b101, OPC_OP_IMM, "orc">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC2   : RVBUnary<0b0010100, 0b11110, 0b101, OPC_OP_IMM, "orc2">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC4   : RVBUnary<0b0010100, 0b11100, 0b101, OPC_OP_IMM, "orc4">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC8   : RVBUnary<0b0010100, 0b11000, 0b101, OPC_OP_IMM, "orc8">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC16  : RVBUnary<0b0010100, 0b10000, 0b101, OPC_OP_IMM, "orc16">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC_H  : RVBUnary<0b0010100, 0b01111, 0b101, OPC_OP_IMM, "orc.h">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC2_H : RVBUnary<0b0010100, 0b01110, 0b101, OPC_OP_IMM, "orc2.h">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC4_H : RVBUnary<0b0010100, 0b01100, 0b101, OPC_OP_IMM, "orc4.h">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC8_H : RVBUnary<0b0010100, 0b01000, 0b101, OPC_OP_IMM, "orc8.h">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC2_B : RVBUnary<0b0010100, 0b00110, 0b101, OPC_OP_IMM, "orc2.b">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC4_B : RVBUnary<0b0010100, 0b00100, 0b101, OPC_OP_IMM, "orc4.b">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC_N  : RVBUnary<0b0010100, 0b00011, 0b101, OPC_OP_IMM, "orc.n">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC2_N : RVBUnary<0b0010100, 0b00010, 0b101, OPC_OP_IMM, "orc2.n">,
             Sched<[WriteORCB, ReadORCB]>;
def ORC_P  : RVBUnary<0b0010100, 0b00001, 0b101, OPC_OP_IMM, "orc.p">,
             Sched<[WriteORCB, ReadORCB]>;
} // Predicates = [HasVendorXCramp]

let Predicates = [HasVendorXCramp] in {
def : Pat<(select (XLenVT (setne (XLenVT GPR:$rs2), 0)), (XLenVT GPR:$rs1), (XLenVT GPR:$rs3)),
          (CMOV GPR:$rs1, GPR:$rs2, GPR:$rs3)>;
def : Pat<(select (XLenVT (seteq (XLenVT GPR:$rs2), 0)), (XLenVT GPR:$rs3), (XLenVT GPR:$rs1)),
          (CMOV GPR:$rs1, GPR:$rs2, GPR:$rs3)>;
def : Pat<(select (XLenVT (setne (XLenVT GPR:$x), simm12_plus1:$y)), (XLenVT GPR:$rs1), (XLenVT GPR:$rs3)),
          (CMOV GPR:$rs1, (ADDI GPR:$x, (NegImm simm12_plus1:$y)), GPR:$rs3)>;
def : Pat<(select (XLenVT (seteq (XLenVT GPR:$x), simm12_plus1:$y)), (XLenVT GPR:$rs3), (XLenVT GPR:$rs1)),
          (CMOV GPR:$rs1, (ADDI GPR:$x, (NegImm simm12_plus1:$y)), GPR:$rs3)>;
def : Pat<(select (XLenVT (setne (XLenVT GPR:$x), GPR:$y)), (XLenVT GPR:$rs1), (XLenVT GPR:$rs3)),
          (CMOV GPR:$rs1, (XOR GPR:$x, GPR:$y), GPR:$rs3)>;
def : Pat<(select (XLenVT (seteq (XLenVT GPR:$x), GPR:$y)), (XLenVT GPR:$rs3), (XLenVT GPR:$rs1)),
          (CMOV GPR:$rs1, (XOR GPR:$x, GPR:$y), GPR:$rs3)>;

def : Pat<(select (XLenVT GPR:$rs2), (XLenVT GPR:$rs1), (XLenVT GPR:$rs3)),
          (CMOV GPR:$rs1, GPR:$rs2, GPR:$rs3)>;
} // Predicates = [HasVendorXCramp]

let Predicates = [HasVendorXCramp] in {
def : Pat<(riscv_fsl (XLenVT GPR:$rs1), (XLenVT GPR:$rs3), (XLenVT GPR:$rs2)),
          (FSL GPR:$rs1, GPR:$rs2, GPR:$rs3)>;
def : Pat<(riscv_fsr (XLenVT GPR:$rs1), (XLenVT GPR:$rs3), (XLenVT GPR:$rs2)),
          (FSR GPR:$rs1, GPR:$rs2, GPR:$rs3)>;
def : Pat<(riscv_fsr (XLenVT GPR:$rs1), (XLenVT GPR:$rs3), uimm5:$shamt),
          (FSRI GPR:$rs1, GPR:$rs3, uimm5:$shamt)>;
// We can use FSRI for FSL by immediate if we subtract the immediate from
// XLen and swap the operands.
def : Pat<(riscv_fsl (XLenVT GPR:$rs3), (XLenVT GPR:$rs1), uimm5:$shamt),
          (FSRI GPR:$rs1, GPR:$rs3, (ImmSubFromXLen uimm5:$shamt))>;
} // Predicates = [HasVendorXCramp]

let Predicates = [HasVendorXCramp, HasStdExtZmmul] in{
def : CompressPat<(MUL GPRC:$rs1, GPRC:$rs1, GPRC:$rs2),
                  (CC_MUL GPRC:$rs1, GPRC:$rs2)>;
let isCompressOnly = true in
def : CompressPat<(MUL GPRC:$rs1, GPRC:$rs2, GPRC:$rs1),
                  (CC_MUL GPRC:$rs1, GPRC:$rs2)>;
} // Predicates = [HasVendorXCramp, HasStdExtZmmul]

let Predicates = [HasVendorXCramp, HasStdExtZbb] in{
def : CompressPat<(SEXT_B GPRC:$rs1, GPRC:$rs1),
                  (CC_SEXT_B GPRC:$rs1, GPRC:$rs1)>;
def : CompressPat<(SEXT_H GPRC:$rs1, GPRC:$rs1),
                  (CC_SEXT_H GPRC:$rs1, GPRC:$rs1)>;
} // Predicates = [HasVendorXCramp, HasStdExtZbb]

let Predicates = [HasVendorXCramp, HasStdExtZbb] in{
def : CompressPat<(ZEXT_H_RV32 GPRC:$rs1, GPRC:$rs1),
                  (CC_ZEXT_H GPRC:$rs1, GPRC:$rs1)>;
def : CompressPat<(ZEXT_H_RV64 GPRC:$rs1, GPRC:$rs1),
                  (CC_ZEXT_H GPRC:$rs1, GPRC:$rs1)>;
} // Predicates = [HasVendorXCramp, HasStdExtZbb]

let Predicates = [HasVendorXCramp] in{
def : CompressPat<(ANDI GPRC:$rs1, GPRC:$rs1, 255),
                  (CC_ZEXT_B GPRC:$rs1, GPRC:$rs1)>;
} // Predicates = [HasVendorXCramp]

let Predicates = [HasVendorXCramp] in{
def : CompressPat<(XORI GPRC:$rs1, GPRC:$rs1, -1),
                  (CC_NOT GPRC:$rs1, GPRC:$rs1)>;
}

let Predicates = [HasVendorXCramp] in{
def : CompressPat<(SUB GPRC:$rs1, X0, GPRC:$rs1),
                  (CC_NEG GPRC:$rs1, GPRC:$rs1)>;
}

let Predicates = [HasVendorXCramp] in{
def : CompressPat<(LBU GPRC:$rd, GPRCMem:$rs1, uimm5:$imm),
                  (CC_LBU GPRC:$rd, GPRCMem:$rs1, uimm5:$imm)>;
def : CompressPat<(LB GPRC:$rd, GPRCMem:$rs1, uimm5:$imm),
                  (CC_LB GPRC:$rd, GPRCMem:$rs1, uimm5:$imm)>;
def : CompressPat<(LHU GPRC:$rd, GPRCMem:$rs1, uimm4_lsb0:$imm),
                  (CC_LHU GPRC:$rd, GPRCMem:$rs1, uimm4_lsb0:$imm)>;
def : CompressPat<(LH GPRC:$rd, GPRCMem:$rs1, uimm4_lsb0:$imm),
                  (CC_LH GPRC:$rd, GPRCMem:$rs1, uimm4_lsb0:$imm)>;
def : CompressPat<(SB GPRC:$rs2, GPRCMem:$rs1, uimm5:$imm),
                  (CC_SB GPRC:$rs2, GPRCMem:$rs1, uimm5:$imm)>;
def : CompressPat<(SH GPRC:$rs2, GPRCMem:$rs1, uimm4_lsb0:$imm),
                  (CC_SH GPRC:$rs2, GPRCMem:$rs1, uimm4_lsb0:$imm)>;
def : CompressPat<(SB X0, GPRCMem:$rs1, uimm4:$imm),
                  (CC_SB0 GPRCMem:$rs1, uimm4:$imm)>;
def : CompressPat<(SH X0, GPRCMem:$rs1, uimm5_lsb0:$imm),
                  (CC_SH0 GPRCMem:$rs1, uimm5_lsb0:$imm)>;
def : CompressPat<(SW X0, GPRCMem:$rs1, uimm7_lsb00:$imm),
                  (CC_SW0 GPRCMem:$rs1, uimm7_lsb00:$imm)>;
}// Predicates = [HasVendorXCramp]

let Predicates = [HasVendorXCramp] in{
def : CompressPat<(ADDI GPRC:$rd, GPRC:$rs, simm7_lsb00nonzero:$imm),
                  (C_ADDI2W GPRC:$rd, GPRC:$rs, simm7_lsb00nonzero:$imm)>;
def : CompressPat<(ADDI GPRC:$rd, GPRC:$rs, simm3_not0nor4:$imm),
                  (C_ADDI2B GPRC:$rd, GPRC:$rs, simm3_not0nor4:$imm)>;
def : CompressPat<(SLTIU GPRC:$rd, GPRC:$rs, 1),
                  (C_SEQZ GPRC:$rd, GPRC:$rs)>;
def : CompressPat<(SLTU GPRC:$rd, X0, GPRC:$rs),
                  (C_SNEZ GPRC:$rd, GPRC:$rs)>;
def : CompressPat<(ADD GPRC:$rd, GPRC:$rs1, GPRC:$rs2),
                  (C_ADD2 GPRC:$rd, GPRC:$rs1, GPRC:$rs2)>;
def : CompressPat<(SLT GPRC:$rd, GPRC:$rs1, GPRC:$rs2),
                  (C_SLT GPRC:$rd, GPRC:$rs1, GPRC:$rs2)>;
def : CompressPat<(SLTU GPRC:$rd, GPRC:$rs1, GPRC:$rs2),
                  (C_SLTU GPRC:$rd, GPRC:$rs1, GPRC:$rs2)>;
}// Predicates = [HasVendorXCramp]

let Predicates = [HasVendorXCramp] in{
def : CompressPat<(BEQ GPRC:$rs1, GPRC:$rs2, simm6_lsb0:$imm),
                  (C_BEQ GPRC:$rs1, GPRC:$rs2, simm6_lsb0:$imm)>;
def : CompressPat<(BNE GPRC:$rs1, GPRC:$rs2, simm6_lsb0:$imm),
                  (C_BNE GPRC:$rs1, GPRC:$rs2, simm6_lsb0:$imm)>;
}// Predicates = [HasVendorXCramp]

//===----------------------------------------------------------------------===//
// Pseudo Instructions
//===----------------------------------------------------------------------===//

let Predicates = [HasVendorXCramp] in {
def : InstAlias<"cc.lbu $rd, (${rs1})",(CC_LBU GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"cc.lb $rd, (${rs1})", (CC_LB GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"cc.lhu $rd, (${rs1})",(CC_LHU GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"cc.lh $rd, (${rs1})", (CC_LH GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"cc.sb $rd, (${rs1})", (CC_SB GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"cc.sh $rd, (${rs1})", (CC_SH GPRC:$rd, GPRC:$rs1, 0)>;
def : InstAlias<"cc.sb0 (${rs1})",(CC_SB0 GPRC:$rs1, 0)>;
def : InstAlias<"cc.sh0 (${rs1})",(CC_SH0 GPRC:$rs1, 0)>;
def : InstAlias<"cc.sw0 (${rs1})",(CC_SW0 GPRC:$rs1, 0)>;
}
