add_llvm_component_library(LLVMCrampInfo
  CrampTargetInfo.cpp

  LINK_COMPONENTS
  MC
  Support

  ADD_TO_COMPONENT
  Cramp
  )
