if(PCB STREQUAL X9D OR PCB STREQUAL X9D+ OR PCB STREQUAL SKY9X OR PCB STREQUAL AR9X)
  set(EEPROM_CONVERSIONS 216)
endif()

if(PCB STREQUAL SKY9X OR PCB STREQUAL AR9X OR PCB STREQUAL 9XRPRO)
  set(EEPROM_CONVERSIONS 216)
endif()

if(PCB STREQUAL X7 OR PCB STREQUAL XLITE OR PCB STREQUAL X9E)
  set(EEPROM_CONVERSIONS 218)
endif()

if(PCB STREQUAL X12S OR PCB STREQUAL X10)
  set(EEPROM_CONVERSIONS 218)
endif()

if (PCB STREQUAL TANGO)
  if (VERSION_MAJOR STREQUAL "1" AND VERSION_MINOR STREQUAL "1")
    set(SRC
      ${SRC}
      storage/conversions/conversions_101_110.cpp
      )
    add_definitions(-DTANGO_CONVERT_VERSION_101)
  endif()
  set(EEPROM_CONVERSIONS 219)
endif()

if(EEPROM_CONVERSIONS)
  add_definitions(-DEEPROM_CONVERSIONS=${EEPROM_CONVERSIONS})
  set(SRC
    ${SRC}
    storage/conversions/conversions.cpp
    )
endif()

if(EEPROM_CONVERSIONS LESS 218)
  set(SRC
    ${SRC}
    storage/conversions/conversions_216_218.cpp
    )
endif()

if(EEPROM_CONVERSIONS LESS 219)
  set(SRC
    ${SRC}
    storage/conversions/conversions_218_219.cpp
  )
endif()
