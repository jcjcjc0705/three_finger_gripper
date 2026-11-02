// Sends JPEG frames to one TCP client: gripper_sensors/palm_stream.
//
// Every message is a 24 byte little-endian header followed by its payload:
//   char[4] magic "PSS1" | u8 type | u8 0 | u16 0 | u32 seq | u64 stamp_ns | u32 length
//   type 1  jpeg: u16 width | u16 height | jpeg bytes
// stamp_ns is the board's CLOCK_MONOTONIC at capture.
#ifndef STREAM_SERVER_HPP_
#define STREAM_SERVER_HPP_

#include <sys/uio.h>

#include <cstddef>
#include <cstdint>
#include <mutex>

namespace stream
{

class Server
{
public:
  Server() = default;
  ~Server();

  Server(const Server &) = delete;
  Server & operator=(const Server &) = delete;

  bool listen(int port);
  void close();

  /// Waits up to timeout_ms for a client. A new client replaces the old one.
  void poll_accept(int timeout_ms);
  bool connected();

  /// A client that stops reading is dropped after a second, so a send blocks its
  /// caller for that long at most. Returns false when nothing was sent.
  bool send_jpeg(uint32_t seq, uint64_t stamp_ns, uint16_t width, uint16_t height,
                 const uint8_t * data, std::size_t size);

private:
  bool send(iovec * iov, int count);
  void drop_locked();

  std::mutex mutex_;
  int listener_{-1};
  int client_{-1};
};

}  // namespace stream

#endif  // STREAM_SERVER_HPP_
