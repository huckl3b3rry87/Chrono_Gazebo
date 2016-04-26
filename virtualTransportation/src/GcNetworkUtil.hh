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

int Recv(int sockfd, char *buf, int size);

int Send(int sockfd, char *buf, int size);

int ConnectGcServer(const std::string &server, int *id, int *numClients);

int ListenGcClients(int numClients);

struct gcPacket GetPacket(const std::vector<gazebo::math::Pose> &poses);

}

#endif /* SRC_GCNETWORKUTIL_HH_ */
