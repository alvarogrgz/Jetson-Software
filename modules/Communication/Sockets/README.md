# ISocket implementation
The ISocket and ISocketServer implementation allows the developer to implement generic network or IPC communication seamleslly without caring about the underlying transport protocol.

In order to guarantee that every ISocket and ISocketServer respect the same behaviour, a unique test suite has been created. The same test suite is executed for every implementation of the ISocket and ISocketServer interfaces.
The test suite can be found in ```tests/Communication/SocketsTests```

## Currently implemented transports
### Network
- TCP

### IPC
- Unix Domain Sockets

## Interfaces behaviour
In this section, the behaviour of the ISocket and ISocketServer are listed. More details about their implementation are available in the test module.

#### [Server] Open and close sequence
It must be possible to open and close the server multiple times

#### [Server] Not possible to open two servers on same resource
A server must fail to open if the resource (e.g. port) is already in use by another server.

#### [Server] Allows multiple client to connect

#### [Server] Accept operation is interrupted if server gets closed

#### [Client] Open and close sequence
A client can be reused multiple times as long as the server is open.

#### [Socket] Write operation fails if partner socket is closed

#### [Socket] Blocking read and timeout read return immediately on socket close

#### [Socket] Blocking read and timeout read return immediately on partner socket close

#### [Socket] Timeout read returns amount of bytes received before timeout
If the socket requests to read 4 bytes, but only 2 are received before the timeout, those 2 bytes are returned.