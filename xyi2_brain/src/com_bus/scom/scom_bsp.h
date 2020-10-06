#ifndef __scom_bsp_H
#define __scom_bsp_H

  #include <sstream>
  #include <boost/asio.hpp>
  #include <boost/bind.hpp>
  #include <iostream>
  #include <stdio.h>
  #include <stdlib.h>
  #include <string.h>

  #include "scom_config.h"

  // bsp
  bool Scom_Init();

  void Scom_Flush_In();
  void Scom_Flush_Out();
  void Scom_Flush();

  void Scom_Close();

  void SCom_WriteChar(char *dta);

  void SCom_ReadChar(char *dta);

#endif




