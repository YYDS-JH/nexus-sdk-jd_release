#ifndef NEXUS_LOG_HPP_
#define NEXUS_LOG_HPP_

#include <iostream>

#ifdef NEXUS_ENABLE_CONSOLE_LOG
  #define NEXUS_COUT std::cout
  #define NEXUS_CERR std::cerr
#else
  #define NEXUS_COUT while (false) std::cout
  #define NEXUS_CERR while (false) std::cerr
#endif

#endif  // NEXUS_LOG_HPP_
