#ifndef SRC_GCNETWORKUTIL_HH_
#define SRC_GCNETWORKUTIL_HH_

#include <gazebo/math/Pose.hh>
#include <string>
#include <vector>

#define SERVER_PORT "6720"

struct gcPacket {
	double poses[5][7];
};

struct initPkg {
	int id, numClients;
};

namespace gc {

/*
 * Receive a packet of certain size from the given sock file descriptor.
 * The function is blocking, and will return the number of bytes received.
 * Failure message will be printed out if fails.
 */
int Recv(int sockfd, char *buf, int size);

/*
 * Send a packet of certain size to the given sock file descriptor.
 * The function is blocking, and will return the number of bytes sent.
 * Failure message will be printed out if fails.
 */
int Send(int sockfd, char *buf, int size);

/*
 * Connect the client program to the server, and return the network id and number
 * of clients through pointers. The socket file descriptor for this connection
 * is returned.
 *
 * server:
 * 		address of server
 * id:
 * 		pointer to return the network id of the caller
 * numClients:
 * 		pointer to return the number of clients
 */
int ConnectGcServer(const std::string &server, int *id, int *numClients);

/* Open a socket on the server to listen to a given number of clients. */
int ListenGcClients(int numClients);

/* Construct a gcPacket given a vector of Gazebo Pose. */
struct gcPacket GetPacket(const std::vector<gazebo::math::Pose> &poses);

}

#endif /* SRC_GCNETWORKUTIL_HH_ */
