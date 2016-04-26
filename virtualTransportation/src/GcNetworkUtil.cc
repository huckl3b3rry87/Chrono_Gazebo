#include "GcNetworkUtil.hh"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <assert.h>

namespace gc {

int Recv(int sockfd, char *buf, int size) {
	int remain, received;
	char *ptr;

	remain = size;
	ptr = buf;
	while (remain) {
		received = recv(sockfd, ptr, remain, 0);
		if (received <= 0) {
			fprintf(stderr, "receive error: %d\n", received);
			return received;
		}
		remain -= received;
		ptr += received;
	}

//	printf("received:\n");
//	for (int i = 0; i < size; i++) {
//		printf("%02x ", buf[i] & 0xff);
//	}
//	printf("\n");

	return 1;
}

int Send(int sockfd, char *buf, int size) {
	int remain, sent;
	char *ptr;

	remain = size;
	ptr = buf;
	while (remain) {
		sent = send(sockfd, ptr, remain, 0);
		if (send < 0) {
			fprintf(stderr, "send error: %d\n", sent);
			return sent;
		}
		remain -= sent;
		ptr += sent;
	}

//	printf("sent: \n");
//	for (int i = 0; i < size; i++) {
//		printf("%02x ", buf[i] & 0xff);
//	}
//	printf("\n");

	return 1;
}

int ConnectGcServer(const std::string &server, int *id, int *numClients) {
	struct addrinfo hints, *res;
	int sockfd;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;  // use IPv4 or IPv6, whichever
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;     // fill in my IP for me

	getaddrinfo(server.c_str(), SERVER_PORT, &hints, &res);

	// make a socket, bind it, and listen on it:

	assert(
			(sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol))
					!= -1);

	int timeout = 60;
	while (timeout && connect(sockfd, res->ai_addr, res->ai_addrlen)) {
		sleep(1);
		timeout--;
	}
	if (!timeout) {
		printf("connection timeout!\n");
		close(sockfd);
		return -1;
	}

	struct initPkg pkg;
	assert(recv(sockfd, &pkg, sizeof pkg, 0) != -1);
	*id = pkg.id;
	*numClients = pkg.numClients;

	return sockfd;
}

int ListenGcClients(int numClients) {
	struct addrinfo hints, *res;
	int sockfd;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;  // use IPv4 or IPv6, whichever
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;     // fill in my IP for me

	getaddrinfo(NULL, SERVER_PORT, &hints, &res);

	struct sockaddr_in *ipv4 = (struct sockaddr_in *) res->ai_addr;
	void *addr = &(ipv4->sin_addr);
	char ipstr[INET6_ADDRSTRLEN];
	inet_ntop(res->ai_family, addr, ipstr, sizeof(ipstr));
	printf("Created socket at %s:%s\n", ipstr, SERVER_PORT);

	// make a socket, bind it, and listen on it:

	assert(
			(sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol))
					!= -1);
	assert(bind(sockfd, res->ai_addr, res->ai_addrlen) == 0);
	assert(listen(sockfd, numClients) == 0);

	return sockfd;
}

struct gcPacket GetPacket(const std::vector<gazebo::math::Pose> &poses) {
	struct gcPacket ret;
	for (int i = 0; i < 5; i++) {
		ret.poses[i][0] = poses[i].pos.x;
		ret.poses[i][1] = poses[i].pos.y;
		ret.poses[i][2] = poses[i].pos.z;
		ret.poses[i][3] = poses[i].rot.w;
		ret.poses[i][4] = poses[i].rot.x;
		ret.poses[i][5] = poses[i].rot.y;
		ret.poses[i][6] = poses[i].rot.z;
	}
	return ret;
}

}
