//===-- RISCVScheduleXCramp.td - Scheduling Definitions XCramp ---*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

def WriteCMov       : SchedWrite;    // cmov
def WriteFSReg      : SchedWrite;    // fsl, fsr
def WriteFSRImm     : SchedWrite;    // fsri
def WriteBscthReg   : SchedWrite;    // bsct.h
def WriteBFReg      : SchedWrite;    // bfmp, bfpp
def WriteBFImm      : SchedWrite;    // bfmi, bfpi
def WriteBFRLen     : SchedWrite;    // bfm, bfp

def ReadCMov             : SchedRead; // cmov
def ReadFSReg            : SchedRead; // fsl, fsr
def ReadFSRImm           : SchedRead; // fsri
def ReadBscthReg         : SchedRead; // bsct.h
def ReadBFReg            : SchedRead; // bfmp, bfpp
def ReadBFImm            : SchedRead; // bfmi, bfpi
def ReadBFRLen           : SchedRead; // bfm, bfp

multiclass UnsupportedSchedXCramp {
let Unsupported = true in {
def : WriteRes<WriteCMov, []>;
def : WriteRes<WriteFSReg, []>;
def : WriteRes<WriteFSRImm, []>;
def : WriteRes<WriteBscthReg, []>;
def : WriteRes<WriteBFReg, []>;
def : WriteRes<WriteBFImm, []>;
def : WriteRes<WriteBFRLen, []>;

def : ReadAdvance<ReadCMov, 0>;
def : ReadAdvance<ReadFSReg, 0>;
def : ReadAdvance<ReadFSRImm, 0>;
def : ReadAdvance<ReadBscthReg, 0>;
def : ReadAdvance<ReadBFReg, 0>;
def : ReadAdvance<ReadBFImm, 0>;
def : ReadAdvance<ReadBFRLen, 0>;
}
}
