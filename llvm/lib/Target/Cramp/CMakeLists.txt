add_llvm_component_group(Cramp)

set(LLVM_TARGET_DEFINITIONS Cramp.td)

tablegen(LLVM CrampGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM CrampGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM CrampGenCompressInstEmitter.inc -gen-compress-inst-emitter)
tablegen(LLVM CrampGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM CrampGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM CrampGenGlobalISel.inc -gen-global-isel)
tablegen(LLVM CrampGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM CrampGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM CrampGenMCPseudoLowering.inc -gen-pseudo-lowering)
tablegen(LLVM CrampGenRegisterBank.inc -gen-register-bank)
tablegen(LLVM CrampGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM CrampGenSearchableTables.inc -gen-searchable-tables)
tablegen(LLVM CrampGenSubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(CrampCommonTableGen)

add_llvm_target(CrampCodeGen
  CrampAsmPrinter.cpp
  CrampCallLowering.cpp
  CrampCodeGenPrepare.cpp
  CrampMakeCompressible.cpp
  CrampExpandAtomicPseudoInsts.cpp
  CrampExpandPseudoInsts.cpp
  CrampFrameLowering.cpp
  CrampGatherScatterLowering.cpp
  CrampInsertVSETVLI.cpp
  CrampInstrInfo.cpp
  CrampInstructionSelector.cpp
  CrampISelDAGToDAG.cpp
  CrampISelLowering.cpp
  CrampLegalizerInfo.cpp
  CrampMachineFunctionInfo.cpp
  CrampMacroFusion.cpp
  CrampMCInstLower.cpp
  CrampMergeBaseOffset.cpp
  CrampRedundantCopyElimination.cpp
  CrampRegisterBankInfo.cpp
  CrampRegisterInfo.cpp
  CrampSelectionDAGInfo.cpp
  CrampSExtWRemoval.cpp
  CrampSubtarget.cpp
  CrampTargetMachine.cpp
  CrampTargetObjectFile.cpp
  CrampTargetTransformInfo.cpp

  LINK_COMPONENTS
  Analysis
  AsmPrinter
  Core
  IPO
  CodeGen
  MC
  CrampDesc
  CrampInfo
  SelectionDAG
  Support
  Target
  TransformUtils
  GlobalISel

  ADD_TO_COMPONENT
  Cramp
  )

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
