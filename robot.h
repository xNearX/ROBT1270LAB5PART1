#ifndef _SOCK_H_
#define _SOCK_H_

#include <cstdio>
#include <string>
using namespace std;
#include <vector>
#include <windows.h>

#define PORT         1270
#define IPV4_STRING  "127.0.0.1"

#pragma warning (disable : 4996)
#pragma warning (disable : 4290)
#pragma comment(lib,"wsock32")

namespace openutils
{

   class CRobot;
   class CSocketException;
   class CSocketAddress;

   class CWinSock
   {
   public:
      static void Initialize();/// WSAStartup
      static void Finalize();/// WSACleanup
   };

   class CServerSocket
   {
   private:
      SOCKET m_socket; /// listening socket
      SOCKADDR_IN m_sockAddrIn; /// default socket settings
      int m_nPort; /// server port
      int m_nQueue; /// number of clients that can be in queue waiting for acceptance.
      CSocketAddress *m_sockAddr; /// Address to which this server is attached.
      bool m_bBound; /// true if bound to port.
      bool m_bListening; /// true if listening
   public:
      CServerSocket();  /// default constructor
      CServerSocket(int port); /// overloaded constructor
      CServerSocket(int port, int queue); /// overloaded constructor
      ~CServerSocket(); /// default destructor
      void Bind(CSocketAddress *scok_addr);/// Binds the server to the given address.
      CRobot *Accept() throw (CSocketException);/// Accepts a client connection.
      void Close(); /// Closes the Socket.	
      bool IsListening(); /// returns the listening flag

      void SetPort(int port); /// Sets the port
      void SetQueue(int q); /// Sets the queue size
      int GetPort(); /// returns the port
      int GetQueue(); /// returns the queue size
      CSocketAddress *GetSocketAddress(); /// Returns the socket address
   private:
      void Init(); /// default initialization
   };

   class CRobot
   {
   private:
      SOCKET m_socket; /// SOCKET for communication
      CSocketAddress *m_clientAddr; /// Address details of this socket.
   public:
      CRobot(); /// Default constructor
      void SetSocket(SOCKET sock); /// Sets the SOCKET
      void SetClientAddr(SOCKADDR_IN addr); /// Sets address details
      int Connect(); /// Connects to a server
      int Connect(const char *host_name, int port); /// Connects to host
      CSocketAddress *GetAddress() { return m_clientAddr; } /// Returns the client address
      int Send(const char *data) throw (CSocketException); /// Writes data to the socket
      int Read(char *buffer, int len) throw (CSocketException); /// Reads data from the socket
      void Close(); /// Closes the socket
      int Initialize();
      ~CRobot(); /// Destructor
   };

   class CSocketAddress
   {
   private:
      SOCKADDR_IN m_sockAddrIn; /// server info
      LPHOSTENT m_lpHostEnt; /// used to obtain address by name
      string m_strHostName; /// host name
      int m_nPort; /// port 
   public:
      CSocketAddress(const char *host, int port); /// default constructor		
      CSocketAddress(SOCKADDR_IN sockAddr);/// Constructor initialized by a SOCKADDR_IN
      const char *GetIP(); /// Returns the IP address
      const char *GetName(); /// Returns the official address
      int GetPort() { return m_nPort; } /// Returns the port
      void GetAliases(vector<string> *ret); /// Returns aliases
      SOCKADDR_IN GetSockAddrIn() throw (CSocketException); /// returns the sockaddr_in
      void operator = (CSocketAddress addr); /// Assignment operation
      ~CSocketAddress(); /// Destructor
   };

   class CSocketException
   {
   private:
      string m_strError; /// error message
      int m_nCode; /// Error code
   public:
      CSocketException(int code, const char *msg)
      {
         m_nCode = code;
         m_strError = msg;
      }

      inline int GetCode() { return m_nCode; }
      inline const char *GetMessage() { return m_strError.c_str(); }
   };
}

using namespace openutils;

#endif