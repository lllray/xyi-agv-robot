#include "scom_bsp.h"

using namespace boost::asio;

// variable
boost::asio::io_service m_ios;
boost::system::error_code m_ec;
static serial_port *pSerialPort;

// bsp
bool Scom_Init() 
{
  pSerialPort = new serial_port(m_ios);

  pSerialPort->open(RS_DEVICE,m_ec);

  if(pSerialPort)
  {
    pSerialPort->set_option(serial_port::baud_rate(BAUDRATE),m_ec);
    pSerialPort->set_option(serial_port::flow_control(serial_port::flow_control::none),m_ec);
    pSerialPort->set_option(PARITY,m_ec);
    pSerialPort->set_option(serial_port::stop_bits(STOPBITS),m_ec);
    pSerialPort->set_option(serial_port::character_size(CharSize),m_ec);
    
    Scom_Flush();

    return true;
  }else{
    return false;
  }
}

void Scom_Flush_In()
{
  int FlushFd = open(RS_DEVICE,O_RDWR);
  tcflush(FlushFd,TCIFLUSH);
  close(FlushFd);
}

void Scom_Flush_Out()
{
  int FlushFd = open(RS_DEVICE,O_RDWR);
  tcflush(FlushFd,TCOFLUSH);
  close(FlushFd);
}

void Scom_Flush()
{
  int FlushFd = open(RS_DEVICE,O_RDWR);
  tcflush(FlushFd,TCIOFLUSH);
  close(FlushFd);
}

void Scom_Close()
{
  Scom_Flush();
  pSerialPort->close();

  delete pSerialPort;
}

void SCom_WriteChar(char *dta)
{
    char dta_deal[CommandLenth];

    memcpy(dta_deal, dta, CommandLenth);

    Scom_Flush_Out();
    write(*pSerialPort, buffer(dta_deal),m_ec);

}

void SCom_ReadChar(char *dta)
{
    char dta_deal[CommandLenth];

    read(*pSerialPort, buffer(dta_deal),m_ec);
    Scom_Flush_In();

    memcpy(dta, dta_deal, CommandLenth);

}













