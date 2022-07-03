// Executables must have the following defined if the library contains
// doctest definitions. For builds with this disabled, e.g. code shipped to
// users, this can be left out.
#ifdef ENABLE_DOCTEST_IN_LIBRARY
#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"
#endif

#include <iostream>
#include <stdlib.h>
#include <wiringPiSPI.h>

#define SPI_CHANNEL 0
#define SPI_CLOCK_SPEED 1000000

/*
 * Simple main program that demontrates how access
 * CMake definitions (here the version number) from source code.
 */
int main() {
  std::cout << "GNC System v"
            << PROJECT_VERSION_MAJOR
            << "."
            << PROJECT_VERSION_MINOR
            << "."
            << PROJECT_VERSION_PATCH
            << "."
            << PROJECT_VERSION_TWEAK
            << std::endl;

  int fd = wiringPiSPISetupMode(SPI_CHANNEL, SPI_CLOCK_SPEED, 0);
  if (fd == -1)
  {
    std::cout << "Failed to init SPI communication.\n";
    return -1;
  }
  std::cout << "SPI communication successfully setup.\n";

  unsigned char buf[2] = {23, 0};
  wiringPiSPIDataRW(SPI_CHANNEL, buf, 2);
  std::cout << "Data returned: " << +buf[1] << "\n";
  
  return 0;

}


