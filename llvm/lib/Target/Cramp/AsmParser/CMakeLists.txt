add_llvm_component_library(LLVMCrampAsmParser
  CrampAsmParser.cpp

  LINK_COMPONENTS
  MC
  MCParser
  CrampDesc
  CrampInfo
  Support

  ADD_TO_COMPONENT
  Cramp
  )
