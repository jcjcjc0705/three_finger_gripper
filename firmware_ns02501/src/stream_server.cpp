#include "stream_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace stream
{
namespace
{

constexpr uint8_t kTypeJpeg = 1;
constexpr std::size_t kHeaderSize = 24;

void put_u16(uint8_t * p, uint16_t v) { std::memcpy(p, &v, 2); }
void put_u32(uint8_t * p, uint32_t v) { std::memcpy(p, &v, 4); }
void put_u64(uint8_t * p, uint64_t v) { std::memcpy(p, &v, 8); }

void make_header(uint8_t * h, uint8_t type, uint32_t seq, uint64_t stamp_ns, uint32_t length)
{
  std::memcpy(h, "PSS1", 4);
  h[4] = type;
  h[5] = 0;
  put_u16(h + 6, 0);
  put_u32(h + 8, seq);
  put_u64(h + 12, stamp_ns);
  put_u32(h + 20, length);
}

}  // namespace

Server::~Server()
{
  close();
}

bool Server::listen(int port)
{
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) { return false; }
  const int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 ||
      ::listen(fd, 1) != 0)
  {
    const int saved = errno;
    ::close(fd);
    errno = saved;
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  listener_ = fd;
  return true;
}

void Server::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  drop_locked();
  if (listener_ >= 0) {
    ::close(listener_);
    listener_ = -1;
  }
}

void Server::drop_locked()
{
  if (client_ >= 0) {
    ::close(client_);
    client_ = -1;
  }
}

void Server::poll_accept(int timeout_ms)
{
  int listener;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    listener = listener_;
  }
  if (listener < 0) { return; }

  pollfd pfd{listener, POLLIN, 0};
  if (::poll(&pfd, 1, timeout_ms) <= 0) { return; }

  sockaddr_in peer{};
  socklen_t len = sizeof(peer);
  const int fd = ::accept(listener, reinterpret_cast<sockaddr *>(&peer), &len);
  if (fd < 0) { return; }

  const int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  timeval tv{1, 0};
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  char ip[INET_ADDRSTRLEN] = "?";
  inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
  std::printf("stream client %s:%u connected\n", ip, ntohs(peer.sin_port));
  std::fflush(stdout);

  std::lock_guard<std::mutex> lock(mutex_);
  drop_locked();
  client_ = fd;
}

bool Server::connected()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return client_ >= 0;
}

bool Server::send(iovec * iov, int count)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (client_ < 0) { return false; }
  std::size_t total = 0;
  for (int i = 0; i < count; ++i) { total += iov[i].iov_len; }

  while (total > 0) {
    msghdr msg{};
    msg.msg_iov = iov;
    msg.msg_iovlen = count;
    const ssize_t n = ::sendmsg(client_, &msg, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR) { continue; }
      std::fprintf(stderr, "stream client dropped: %s\n", std::strerror(errno));
      drop_locked();
      return false;
    }
    total -= static_cast<std::size_t>(n);
    std::size_t left = static_cast<std::size_t>(n);
    while (count > 0 && left >= iov->iov_len) {
      left -= iov->iov_len;
      ++iov;
      --count;
    }
    if (count > 0) {
      iov->iov_base = static_cast<uint8_t *>(iov->iov_base) + left;
      iov->iov_len -= left;
    }
  }
  return true;
}

bool Server::send_jpeg(uint32_t seq, uint64_t stamp_ns, uint16_t width, uint16_t height,
                       const uint8_t * data, std::size_t size)
{
  uint8_t header[kHeaderSize + 4];
  make_header(header, kTypeJpeg, seq, stamp_ns, static_cast<uint32_t>(4 + size));
  put_u16(header + kHeaderSize, width);
  put_u16(header + kHeaderSize + 2, height);
  iovec iov[2] = {
    {header, sizeof(header)},
    {const_cast<uint8_t *>(data), size},
  };
  return send(iov, 2);
}

}  // namespace stream
