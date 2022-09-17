add_llvm_component_library(LLVMCrampDesc
  CrampAsmBackend.cpp
  CrampBaseInfo.cpp
  CrampELFObjectWriter.cpp
  CrampInstPrinter.cpp
  CrampMCAsmInfo.cpp
  CrampMCCodeEmitter.cpp
  CrampMCExpr.cpp
  CrampMCObjectFileInfo.cpp
  CrampMCTargetDesc.cpp
  CrampMatInt.cpp
  CrampTargetStreamer.cpp
  CrampELFStreamer.cpp

  LINK_COMPONENTS
  MC
  CrampInfo
  Support

  ADD_TO_COMPONENT
  Cramp
)
