/*
 * TCPStream.cpp
 *
 *  Created on: Jun 29, 2015
 *      Author: apratim
 */

#include <arpa/inet.h>
#include "TCPStream.h"

TCPStream::TCPStream(int sd, struct sockaddr_in* address) : m_sd(sd)
{
    char ip[50];
    inet_ntop(PF_INET, (struct in_addr*)&(address->sin_addr.s_addr),
              ip, sizeof(ip)-1);
    m_peerIP = ip;
    m_peerPort = ntohs(address->sin_port);
}

TCPStream::~TCPStream()
{
    close(m_sd);
}

ssize_t TCPStream::send(char* buffer, size_t len)
{

	ssize_t num_bytes;
	num_bytes = write(m_sd, buffer, len);
	return num_bytes;

}

ssize_t TCPStream::receive(char* buffer, size_t len)
{

	ssize_t num_bytes;
	num_bytes = read(m_sd, buffer, len);
	return num_bytes;

}

string TCPStream::getPeerIP()
{

	return m_peerIP;

}

int TCPStream::getPeerPort()
{

	return m_peerPort;

}

