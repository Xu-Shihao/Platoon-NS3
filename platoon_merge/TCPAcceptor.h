/*
 * TCPAcceptor.h
 *
 *  Created on: Jun 29, 2015
 *      Author: apratim
 */

#ifndef TCPACCEPTOR_H_
#define TCPACCEPTOR_H_


#endif /* TCPACCEPTOR_H_ */

#include <string>
#include <netinet/in.h>
#include "TCPStream.h"

using namespace std;

class TCPAcceptor
{
    int    m_lsd;
    string m_address;
    int    m_port;
    bool   m_listening;

  public:
    TCPAcceptor(int port, const char* address="");
    ~TCPAcceptor();

    int        start();
    TCPStream* accept();

  private:
    TCPAcceptor() {}
};

