#include "camera/v4l2_capture.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <utility>

namespace camera
{
namespace
{

/// Every ioctl here can be interrupted by a signal, and none of them are
/// meaningfully retryable for any other reason.
int xioctl(int fd, unsigned long request, void * arg)
{
  int result;
  do {
    result = ::ioctl(fd, request, arg);
  } while (result == -1 && errno == EINTR);
  return result;
}

std::string errno_text(const std::string & what)
{
  return what + ": " + std::strerror(errno);
}

/// MemAvailable in bytes, or 0 when it cannot be read.
std::size_t available_memory()
{
  std::FILE * f = std::fopen("/proc/meminfo", "r");
  if (f == nullptr) { return 0; }
  char line[128];
  unsigned long kb = 0;
  while (std::fgets(line, sizeof(line), f) != nullptr) {
    if (std::sscanf(line, "MemAvailable: %lu kB", &kb) == 1) { break; }
  }
  std::fclose(f);
  return static_cast<std::size_t>(kb) * 1024;
}

}  // namespace

std::string fourcc_to_string(uint32_t fourcc)
{
  std::string out(4, ' ');
  for (int i = 0; i < 4; ++i) {
    const char c = static_cast<char>((fourcc >> (8 * i)) & 0xFF);
    out[i] = (c >= 32 && c < 127) ? c : '?';
  }
  return out;
}

uint32_t string_to_fourcc(const std::string & text)
{
  if (text.size() != 4) { return 0; }
  return v4l2_fourcc(text[0], text[1], text[2], text[3]);
}

V4L2Capture::V4L2Capture(std::string device) : device_(std::move(device)) {}

V4L2Capture::~V4L2Capture() { close(); }

bool V4L2Capture::open()
{
  fd_ = ::open(device_.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    last_error_ = errno_text("cannot open " + device_);
    return false;
  }

  v4l2_capability cap{};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) != 0) {
    last_error_ = errno_text("VIDIOC_QUERYCAP (not a V4L2 device?)");
    close();
    return false;
  }

  // A UVC camera exposes several nodes; the metadata one answers QUERYCAP but
  // cannot stream video, so checking both capabilities is what keeps us from
  // opening /dev/video1 and hanging.
  const uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
                          ? cap.device_caps
                          : cap.capabilities;
  if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) {
    last_error_ = device_ + " is not a video capture device";
    close();
    return false;
  }
  if (!(caps & V4L2_CAP_STREAMING)) {
    last_error_ = device_ + " does not support streaming I/O";
    close();
    return false;
  }
  return true;
}

void V4L2Capture::close()
{
  stop();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool V4L2Capture::list_formats(std::vector<FormatInfo> & out)
{
  out.clear();
  if (fd_ < 0) {
    last_error_ = "device not open";
    return false;
  }

  v4l2_fmtdesc desc{};
  desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  for (desc.index = 0; xioctl(fd_, VIDIOC_ENUM_FMT, &desc) == 0; ++desc.index) {
    FormatInfo info;
    info.pixelformat = desc.pixelformat;
    info.description = reinterpret_cast<const char *>(desc.description);
    info.emulated = (desc.flags & V4L2_FMT_FLAG_EMULATED) != 0;

    v4l2_frmsizeenum size{};
    size.pixel_format = desc.pixelformat;
    for (size.index = 0; xioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &size) == 0; ++size.index) {
      // Stepwise/continuous devices describe a range rather than a list; the
      // caller gets an empty list and has to name a size itself.
      if (size.type != V4L2_FRMSIZE_TYPE_DISCRETE) { break; }
      FrameSize entry{size.discrete.width, size.discrete.height, {}};

      v4l2_frmivalenum ival{};
      ival.pixel_format = desc.pixelformat;
      ival.width = entry.width;
      ival.height = entry.height;
      for (ival.index = 0; xioctl(fd_, VIDIOC_ENUM_FRAMEINTERVALS, &ival) == 0; ++ival.index) {
        if (ival.type != V4L2_FRMIVAL_TYPE_DISCRETE) { break; }
        if (ival.discrete.numerator == 0) { continue; }
        entry.fps.push_back(static_cast<float>(ival.discrete.denominator) /
                            static_cast<float>(ival.discrete.numerator));
      }
      info.sizes.push_back(std::move(entry));
    }
    out.push_back(std::move(info));
  }

  if (out.empty()) {
    last_error_ = "device reported no capture formats";
    return false;
  }
  return true;
}

bool V4L2Capture::start(
  uint32_t pixelformat, uint32_t width, uint32_t height, uint32_t buffer_count, float fps)
{
  if (fd_ < 0) {
    last_error_ = "device not open";
    return false;
  }
  if (streaming_) {
    last_error_ = "already streaming";
    return false;
  }

  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = pixelformat;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) != 0) {
    last_error_ = errno_text("VIDIOC_S_FMT");
    return false;
  }

  // S_FMT succeeds even when the driver substitutes something it prefers, so
  // what came back is the truth, not what was asked for.
  format_ = fmt.fmt.pix.pixelformat;
  width_ = fmt.fmt.pix.width;
  height_ = fmt.fmt.pix.height;

  v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (fps > 0.0f) {
    parm.parm.capture.timeperframe.numerator = 1000;
    parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps * 1000.0f + 0.5f);
    if (xioctl(fd_, VIDIOC_S_PARM, &parm) != 0) {
      last_error_ = errno_text("VIDIOC_S_PARM");
      return false;
    }
  } else if (xioctl(fd_, VIDIOC_G_PARM, &parm) != 0) {
    parm.parm.capture.timeperframe.numerator = 0;
  }
  const v4l2_fract & tpf = parm.parm.capture.timeperframe;
  fps_ = tpf.numerator != 0 ? static_cast<float>(tpf.denominator) / tpf.numerator : 0.0f;

  // REQBUFS allocates in the kernel, so running out there invokes the OOM
  // killer instead of failing the call.
  const std::size_t needed = static_cast<std::size_t>(fmt.fmt.pix.sizeimage) * buffer_count;
  const std::size_t available = available_memory();
  if (available > 0 && needed > available * 3 / 4) {
    last_error_ = std::to_string(buffer_count) + " buffers of " +
                  std::to_string(fmt.fmt.pix.sizeimage / 1024) + " KB need " +
                  std::to_string(needed / 1024) + " KB, only " +
                  std::to_string(available / 1024) + " KB available";
    return false;
  }

  v4l2_requestbuffers req{};
  req.count = buffer_count;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) != 0) {
    last_error_ = errno_text("VIDIOC_REQBUFS");
    return false;
  }
  if (req.count < 2) {
    last_error_ = "driver granted only " + std::to_string(req.count) +
                  " buffers, need at least 2";
    return false;
  }

  buffers_.resize(req.count);
  for (uint32_t i = 0; i < req.count; ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      last_error_ = errno_text("VIDIOC_QUERYBUF");
      unmap_buffers();
      return false;
    }
    void * mapped = ::mmap(
      nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
    if (mapped == MAP_FAILED) {
      // The likely cause on this board is simply running out of memory.
      last_error_ = errno_text("mmap of buffer " + std::to_string(i));
      unmap_buffers();
      return false;
    }
    buffers_[i].start = mapped;
    buffers_[i].length = buf.length;
  }

  for (uint32_t i = 0; i < buffers_.size(); ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QBUF, &buf) != 0) {
      last_error_ = errno_text("VIDIOC_QBUF");
      unmap_buffers();
      return false;
    }
  }

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) != 0) {
    last_error_ = errno_text("VIDIOC_STREAMON");
    unmap_buffers();
    return false;
  }
  streaming_ = true;
  queued_back_ = -1;
  return true;
}

void V4L2Capture::stop()
{
  if (streaming_) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;
  }
  unmap_buffers();
}

std::size_t V4L2Capture::mapped_bytes() const
{
  std::size_t total = 0;
  for (const auto & buffer : buffers_) { total += buffer.length; }
  return total;
}

bool V4L2Capture::unmap_buffers()
{
  for (auto & buffer : buffers_) {
    if (buffer.start != nullptr) {
      ::munmap(buffer.start, buffer.length);
    }
  }
  buffers_.clear();
  queued_back_ = -1;
  return true;
}

bool V4L2Capture::capture(Frame & frame, int timeout_ms)
{
  if (!streaming_) {
    last_error_ = "not streaming";
    return false;
  }

  // The frame handed out last time points into a buffer the driver still owns
  // as far as it knows; give it back before asking for another.
  if (queued_back_ >= 0) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = static_cast<uint32_t>(queued_back_);
    if (xioctl(fd_, VIDIOC_QBUF, &buf) != 0) {
      last_error_ = errno_text("VIDIOC_QBUF (requeue)");
      return false;
    }
    queued_back_ = -1;
  }

  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  int ready;
  do {
    ready = ::poll(&pfd, 1, timeout_ms);
  } while (ready == -1 && errno == EINTR);

  if (ready < 0) {
    last_error_ = errno_text("poll");
    return false;
  }
  if (ready == 0) {
    last_error_ = "timed out waiting for a frame after " +
                  std::to_string(timeout_ms) + " ms";
    return false;
  }

  v4l2_buffer buf{};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_DQBUF, &buf) != 0) {
    last_error_ = errno_text("VIDIOC_DQBUF");
    return false;
  }
  if (buf.index >= buffers_.size()) {
    last_error_ = "driver returned buffer index out of range";
    return false;
  }

  frame.data = static_cast<const uint8_t *>(buffers_[buf.index].start);
  // MJPEG frames vary in length, so the mapped size is an upper bound only --
  // writing buffers_[i].length bytes would append garbage to every file.
  frame.size = buf.bytesused;
  frame.sequence = buf.sequence;
  frame.timestamp = buf.timestamp;
  queued_back_ = static_cast<int>(buf.index);
  return true;
}

}  // namespace camera
