// Minimal V4L2 capture for a single UVC camera, mmap buffers only.
//
// Shares nothing with the motor side: no DynamixelSDK, no gripper_servo. The
// palm camera reaches the board as a plain USB Video Class device, so this is
// stock Linux userspace.
//
// Deliberately narrow: one device, one format, MMAP streaming. No userptr, no
// DMABUF, no controls. Widen it when something actually needs more.
#ifndef CAMERA__V4L2_CAPTURE_HPP_
#define CAMERA__V4L2_CAPTURE_HPP_

#include <linux/videodev2.h>

#include <cstdint>
#include <string>
#include <vector>

namespace camera
{

/// Turn a V4L2 fourcc into "MJPG" for printing.
std::string fourcc_to_string(uint32_t fourcc);

/// Parse "MJPG" back into a fourcc. Returns 0 on anything not 4 characters.
uint32_t string_to_fourcc(const std::string & text);

struct FrameSize
{
  uint32_t width{0};
  uint32_t height{0};
  std::vector<float> fps;  ///< discrete frame rates, empty when stepwise
};

struct FormatInfo
{
  uint32_t pixelformat{0};
  std::string description;
  bool emulated{false};          ///< libv4l would convert this one, the kernel does not
  std::vector<FrameSize> sizes;  ///< empty when the device reports stepwise sizes
};

/// One dequeued frame. `data` points into the driver's mmap'd buffer and stays
/// valid only until the next capture() call, which requeues it.
struct Frame
{
  const uint8_t * data{nullptr};
  std::size_t size{0};
  std::uint32_t sequence{0};
  struct timeval timestamp{};
};

class V4L2Capture
{
public:
  explicit V4L2Capture(std::string device);
  ~V4L2Capture();

  V4L2Capture(const V4L2Capture &) = delete;
  V4L2Capture & operator=(const V4L2Capture &) = delete;

  bool open();
  void close();

  /// Everything the device says it can do. Works after open(), before start().
  bool list_formats(std::vector<FormatInfo> & out);

  /// Negotiate a format and map `buffer_count` buffers. The driver may hand
  /// back something other than what was asked for -- negotiated_*() report
  /// what was actually granted, and the caller is expected to check.
  ///
  /// Keep buffer_count low: this board has roughly 7 MB of free RAM, so four
  /// 640x480 YUYV buffers (2.4 MB) is already a meaningful bite, while the
  /// same in MJPEG is a few hundred KB.
  ///
  /// `fps` of 0 leaves the driver's default frame rate alone.
  bool start(uint32_t pixelformat, uint32_t width, uint32_t height, uint32_t buffer_count,
             float fps = 0.0f);
  void stop();

  /// Wait for one frame. Returns false on error or timeout; a timeout is not
  /// distinguished from an error because neither is recoverable here.
  bool capture(Frame & frame, int timeout_ms);

  uint32_t negotiated_format() const { return format_; }
  uint32_t negotiated_width() const { return width_; }
  uint32_t negotiated_height() const { return height_; }
  /// Frame rate the driver reports after start(), 0 when it does not say.
  float negotiated_fps() const { return fps_; }
  uint32_t buffer_count() const { return buffers_.size(); }

  /// Bytes actually mapped. For MJPEG the driver sizes buffers for the worst
  /// case, not the typical compressed frame, so this is far larger than the
  /// files that come out -- and it is what has to fit in the board's memory.
  std::size_t mapped_bytes() const;

  const std::string & last_error() const { return last_error_; }

private:
  struct Buffer
  {
    void * start{nullptr};
    std::size_t length{0};
  };

  bool unmap_buffers();

  std::string device_;
  int fd_{-1};
  bool streaming_{false};

  std::vector<Buffer> buffers_;
  /// Index of the buffer handed out by the last capture(), or -1. It has to be
  /// requeued before the next dequeue or the driver runs out of buffers.
  int queued_back_{-1};

  uint32_t format_{0};
  uint32_t width_{0};
  uint32_t height_{0};
  float fps_{0.0f};

  std::string last_error_;
};

}  // namespace camera

#endif  // CAMERA__V4L2_CAPTURE_HPP_
