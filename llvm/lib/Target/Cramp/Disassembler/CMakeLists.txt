add_llvm_component_library(LLVMCrampDisassembler
  CrampDisassembler.cpp

  LINK_COMPONENTS
  MC
  MCDisassembler
  CrampDesc
  CrampInfo
  Support

  ADD_TO_COMPONENT
  Cramp
  )
