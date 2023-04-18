#include <string>
#include <vector>
using namespace std;
#include <windows.h>
#include "robot.h"
#include <conio.h>
using namespace openutils;

#pragma warning (disable : 4290)

void CWinSock::Initialize()
{
   WORD ver = MAKEWORD(1, 1);
   WSADATA wsadata;
   WSAStartup(ver, &wsadata);
}


void CWinSock::Finalize()
{
   WSACleanup();
}

CServerSocket::CServerSocket()
{
   m_nPort = 80;
   m_nQueue = 10;
   Init();
}

CServerSocket::CServerSocket(int port)
{
   m_nPort = port;
   m_nQueue = 10;
   Init();
}

CServerSocket::CServerSocket(int port, int queue)
{
   m_nPort = port;
   m_nQueue = queue;
   Init();
}

/**
* Binds the server to the given address.
*/
void CServerSocket::Bind(CSocketAddress *sock_addr)
{
   m_bBound = false;
   Close();
   m_sockAddr = sock_addr;
   Accept();
}

/**
* Listens and accepts a client.Returns the accepted connection.
*/
CRobot *CServerSocket::Accept() throw (CSocketException)
{
   if(m_sockAddr != NULL)
      m_sockAddrIn = m_sockAddr->GetSockAddrIn();
   if(!m_bBound)
   {
      m_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
      int nret = bind(m_socket, (LPSOCKADDR)&m_sockAddrIn, sizeof(struct sockaddr));
      if(nret == SOCKET_ERROR)
      {
         nret = WSAGetLastError();
         throw CSocketException(nret, "Failed to bind: Accept()");
      }
      m_bBound = true;
   }
   int nret = listen(m_socket, m_nQueue);
   if(nret == SOCKET_ERROR)
   {
      nret = WSAGetLastError();
      throw CSocketException(nret, "Failed to listen: Accept()");
   }
   SOCKET theClient;
   SOCKADDR_IN clientAddr;
   int ssz = sizeof(struct sockaddr);
   theClient = accept(m_socket, (LPSOCKADDR)&clientAddr, &ssz);
   //theClient = accept(m_socket,NULL,NULL);
   if(theClient == INVALID_SOCKET)
   {
      int nret2 = WSAGetLastError();
      throw CSocketException(nret2, "Invalid client socket: Accept()");
   }
   CRobot *sockClient = new CRobot();
   sockClient->SetSocket(theClient);
   sockClient->SetClientAddr(clientAddr);
   return sockClient;
}

void CServerSocket::Close()
{
   closesocket(m_socket);
   m_sockAddr = NULL;
   m_bBound = false;
   m_bListening = false;
}

void CServerSocket::Init()
{
   m_sockAddrIn.sin_family = AF_INET;
   m_sockAddrIn.sin_addr.s_addr = INADDR_ANY;
   m_sockAddrIn.sin_port = htons((u_short)m_nPort);

   m_sockAddr = NULL; // bind the same machine
   m_bBound = false;
   m_bListening = true;
}

CServerSocket::~CServerSocket()
{
   Close();
}

void CServerSocket::SetPort(int port)
{
   m_nPort = port;
   Init();
}

/**
* Sets the queue size
* @param port Value of port
*/
void CServerSocket::SetQueue(int q)
{
   m_nQueue = q;
}

int CServerSocket::GetPort()
{
   return m_nPort;
}

int CServerSocket::GetQueue()
{
   return m_nQueue;
}

CSocketAddress *CServerSocket::GetSocketAddress()
{
   return m_sockAddr;
}

bool CServerSocket::IsListening()
{
   return m_bListening;
}

// class CRobot

CRobot::CRobot()
{
   m_clientAddr = NULL;
}

void CRobot::SetSocket(SOCKET sock)
{
   m_socket = sock;
}

/**
* Sets address details
* @param addr SOCKADDR_IN
*/
void CRobot::SetClientAddr(SOCKADDR_IN addr)
{
   if(m_clientAddr != NULL) delete m_clientAddr;
   m_clientAddr = new CSocketAddress(addr);
}

int CRobot::Connect()
{
   if(m_clientAddr == NULL)
   {
      printf("Cannot connect to NULL host");
   }
   Connect(m_clientAddr->GetName(), m_clientAddr->GetPort());
   return 1;
}

/**
* Connects to a server
* @param host_name Server name
* @param port Port to connect
*/
int CRobot::Connect(const char *host_name, int port)
{
   int nret;
   LPHOSTENT hostEntry;
   hostEntry = gethostbyname(host_name);
   if(!hostEntry)
   {
      nret = WSAGetLastError();
      printf("Failed to resolve host");
      return 0;
   }

   m_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
   if(m_socket == INVALID_SOCKET)
   {
      nret = WSAGetLastError();
      printf("Failed to create client socket");
      return 0;
   }

   SOCKADDR_IN serverInfo;
   serverInfo.sin_family = AF_INET;
   serverInfo.sin_addr = *((LPIN_ADDR)*hostEntry->h_addr_list);
   serverInfo.sin_port = htons((u_short)port);
   nret = connect(m_socket, (LPSOCKADDR)&serverInfo, sizeof(struct sockaddr));
   if(nret == SOCKET_ERROR)
   {
      nret = WSAGetLastError();
      printf("Connect failed.");
      return 0;
   }

   return 1;
}

/**
* Writes data to the socket. Returns number of bytes written
* @param data data to write
*/
int CRobot::Send(const char *data) throw (CSocketException)
{
   int len, nret = 0, nSent, nTotalSent = 0;

   len = (int)strlen(data);

   while(nTotalSent < len)
   {
      nSent = send(m_socket, data + nTotalSent, len - nTotalSent, 0);
      if(nSent == SOCKET_ERROR)
      {
         nret = WSAGetLastError();
         throw CSocketException(nret, "Network failure: Send()");
      }
      else
      {
         nTotalSent += nSent;
      }
   }
   Sleep(200);
   return nret;
}

/*
* Reads data from the socket.Returns number of bytes actually read.
* @param buffer Data buffer
* @param len Number of bytes to read
*/
int CRobot::Read(char *buffer, int len) throw (CSocketException)
{
   int nret = 0;
   nret = recv(m_socket, buffer, len, 0);
   if(nret == SOCKET_ERROR)
   {
      nret = WSAGetLastError();
      throw CSocketException(nret, "Network failure: Read()");
   }
   buffer[nret] = '\0';
   return nret;
}

void CRobot::Close()
{
   closesocket(m_socket);
   if(m_clientAddr != NULL) delete m_clientAddr;
   CWinSock::Finalize();
}

int CRobot::Initialize()
{
   int nret;                  // for integer return values
   system("cls");
   printf("Connecting to %s through port %d...\n", IPV4_STRING, PORT);

   // initializes winsock
   CWinSock::Initialize();
   nret = Connect(IPV4_STRING, PORT);
   if(nret == 0)
   {
      printf("\n\nSimulator must be started and placed in\n");
      printf("remote mode before running this program.\n\n");
      printf("Press any key to close program...");
      getch();
      return FALSE;
   }
   return TRUE;
}

CRobot::~CRobot()
{
   Close();
}

// CSocketAddress

CSocketAddress::CSocketAddress(const char *host, int port)
{
   m_sockAddrIn.sin_family = AF_INET;
   m_sockAddrIn.sin_addr.s_addr = INADDR_ANY; // initialized only in GetSockAddrIn()
   m_sockAddrIn.sin_port = htons((u_short)port);
   m_strHostName = host;
   m_nPort = port;
}

CSocketAddress::CSocketAddress(SOCKADDR_IN sockAddr)
{
   m_sockAddrIn.sin_family = sockAddr.sin_family;
   m_sockAddrIn.sin_addr.s_addr = sockAddr.sin_addr.s_addr;
   m_sockAddrIn.sin_port = sockAddr.sin_port;
   m_strHostName = inet_ntoa(m_sockAddrIn.sin_addr);
   m_nPort = sockAddr.sin_port;;
}

const char *CSocketAddress::GetIP()
{
   return (const char *)inet_ntoa(m_sockAddrIn.sin_addr);
}

const char *CSocketAddress::GetName()
{
   HOSTENT *lpHost = gethostbyname(GetIP());
   if(lpHost == NULL) return NULL;
   return lpHost->h_name;
}

void CSocketAddress::GetAliases(vector<string> *ret)
{
   HOSTENT *lpHost = gethostbyname(GetIP());
   if(lpHost == NULL) return;
   char **tmp = (char **)lpHost->h_aliases;
   if(tmp == NULL)
      return;
   else
   {
      int i = 0;
      while(true)
      {
         if(tmp[i] == NULL) break;
         else ret->push_back(tmp[i]);
      }
   }
}

/**
* Returns the sockaddr_in. tries to bind with the server.
* throws CSocketException on failure.
*/
SOCKADDR_IN CSocketAddress::GetSockAddrIn() throw (CSocketException)
{
   m_lpHostEnt = gethostbyname(m_strHostName.c_str());
   if(!m_lpHostEnt)
   {
      int nret = WSAGetLastError();
      throw CSocketException(nret, "Failed to resolve host:gethostbyname()");
   }
   m_sockAddrIn.sin_addr = *((LPIN_ADDR)*m_lpHostEnt->h_addr_list);
   return m_sockAddrIn;
}

void CSocketAddress::operator = (CSocketAddress addr)
{
   m_sockAddrIn = addr.m_sockAddrIn;
   m_strHostName = addr.m_strHostName;
   m_nPort = addr.m_nPort;
   m_lpHostEnt = addr.m_lpHostEnt;
}

CSocketAddress::~CSocketAddress()
{
}
