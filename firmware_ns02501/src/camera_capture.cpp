// Camera bring-up tool for the ns02501/NT98532 board.
//
// The palm camera is a plain USB Video Class device, so this needs a kernel
// with MEDIA_SUPPORT + USB_VIDEO_CLASS -- disabled in every stock ns02501
// defconfig, see HARDWARE.md.
//
//   ./camera_capture --list                       what can this camera do?
//   ./camera_capture --frames 5 --output /tmp     grab five frames
//
// Start with --list. The board has roughly 7 MB of free RAM, so MJPEG at a
// modest size is the realistic target; 640x480 YUYV is 614 KB per buffer and
// 1280x720 YUYV does not fit at all.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <string>
#include <vector>

#include "camera/v4l2_capture.hpp"

namespace
{

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

void usage()
{
  std::printf(
    "usage: camera_capture [--device PATH] [--list]\n"
    "                      [--format FOURCC] [--size WxH] [--buffers N]\n"
    "                      [--frames N] [--output DIR] [--timeout MS]\n"
    "\n"
    "  --device   default /dev/video0\n"
    "  --list     print the formats and sizes the camera reports, then exit\n"
    "  --format   fourcc, default MJPG. YUYV works but eats far more memory\n"
    "  --size     default 640x480\n"
    "  --buffers  mmap buffers, default 3. Raising it costs one frame of RAM each\n"
    "  --frames   frames to capture, default 1. 0 streams until Ctrl-C\n"
    "  --output   directory to write frames into; omit to capture and discard\n"
    "  --timeout  per-frame wait in ms, default 2000\n");
}

/// MJPEG frames are already JPEG files; anything else gets written raw, since
/// converting would mean pulling in a colour-space conversion this tool has no
/// business owning.
const char * extension_for(uint32_t fourcc)
{
  if (fourcc == V4L2_PIX_FMT_MJPEG || fourcc == V4L2_PIX_FMT_JPEG) { return "jpg"; }
  return "raw";
}

int do_list(camera::V4L2Capture & cam)
{
  std::vector<camera::FormatInfo> formats;
  if (!cam.list_formats(formats)) {
    std::fprintf(stderr, "%s\n", cam.last_error().c_str());
    return 1;
  }

  for (const auto & fmt : formats) {
    std::printf("%s  %s%s\n",
                camera::fourcc_to_string(fmt.pixelformat).c_str(),
                fmt.description.c_str(),
                fmt.emulated ? "  (emulated)" : "");
    if (fmt.sizes.empty()) {
      std::printf("      (stepwise sizes -- pass --size explicitly)\n");
      continue;
    }
    for (const auto & size : fmt.sizes) {
      std::printf("      %4ux%-4u  fps:", size.width, size.height);
      for (float fps : size.fps) { std::printf(" %g", fps); }
      std::printf("\n");
    }
  }
  return 0;
}

bool write_frame(const std::string & dir, int index, uint32_t fourcc,
                 const camera::Frame & frame)
{
  char path[512];
  std::snprintf(path, sizeof(path), "%s/frame_%03d.%s",
                dir.c_str(), index, extension_for(fourcc));

  std::FILE * file = std::fopen(path, "wb");
  if (file == nullptr) {
    std::fprintf(stderr, "cannot open %s: %s\n", path, std::strerror(errno));
    return false;
  }
  const std::size_t written = std::fwrite(frame.data, 1, frame.size, file);
  // A short write means the disk is full, and the file is a lie, so say so
  // rather than reporting success.
  const bool ok = (written == frame.size);
  if (!ok) {
    std::fprintf(stderr, "short write to %s (%zu of %zu bytes)\n",
                 path, written, frame.size);
  }
  if (std::fclose(file) != 0 && ok) {
    std::fprintf(stderr, "cannot flush %s: %s\n", path, std::strerror(errno));
    return false;
  }
  if (ok) { std::printf("  wrote %s (%zu bytes)\n", path, frame.size); }
  return ok;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string device = "/dev/video0";
  std::string format_text = "MJPG";
  std::string output_dir;
  uint32_t width = 640;
  uint32_t height = 480;
  uint32_t buffers = 3;
  int frames = 1;
  int timeout_ms = 2000;
  bool list_only = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--device" && i + 1 < argc) { device = argv[++i]; }
    else if (arg == "--list") { list_only = true; }
    else if (arg == "--format" && i + 1 < argc) { format_text = argv[++i]; }
    else if (arg == "--output" && i + 1 < argc) { output_dir = argv[++i]; }
    else if (arg == "--buffers" && i + 1 < argc) {
      buffers = static_cast<uint32_t>(std::atoi(argv[++i]));
    }
    else if (arg == "--frames" && i + 1 < argc) { frames = std::atoi(argv[++i]); }
    else if (arg == "--timeout" && i + 1 < argc) { timeout_ms = std::atoi(argv[++i]); }
    else if (arg == "--size" && i + 1 < argc) {
      if (std::sscanf(argv[++i], "%ux%u", &width, &height) != 2) {
        std::fprintf(stderr, "--size wants WxH, e.g. 640x480\n");
        return 1;
      }
    }
    else {
      usage();
      return (arg == "-h" || arg == "--help") ? 0 : 1;
    }
  }

  const uint32_t fourcc = camera::string_to_fourcc(format_text);
  if (fourcc == 0) {
    std::fprintf(stderr, "--format wants a 4-character fourcc, e.g. MJPG\n");
    return 1;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  camera::V4L2Capture cam(device);
  if (!cam.open()) {
    std::fprintf(stderr, "%s\n", cam.last_error().c_str());
    std::fprintf(stderr,
                 "(is the camera plugged in, and does this kernel have "
                 "USB_VIDEO_CLASS? check dmesg for uvcvideo)\n");
    return 1;
  }

  if (list_only) { return do_list(cam); }

  if (!cam.start(fourcc, width, height, buffers)) {
    std::fprintf(stderr, "%s\n", cam.last_error().c_str());
    return 1;
  }

  const uint32_t got = cam.negotiated_format();
  // Memory is the binding constraint on this board, and the driver sizes MJPEG
  // buffers for the worst case rather than the typical compressed frame, so
  // say plainly how much was actually mapped.
  std::printf("streaming %s %ux%u, %u buffers, %.1f MB mapped (%.0f KB each)\n",
              camera::fourcc_to_string(got).c_str(),
              cam.negotiated_width(), cam.negotiated_height(), cam.buffer_count(),
              static_cast<double>(cam.mapped_bytes()) / (1024.0 * 1024.0),
              static_cast<double>(cam.mapped_bytes()) / cam.buffer_count() / 1024.0);
  // The driver is free to substitute a format it prefers, and silently getting
  // YUYV when MJPEG was asked for would show up much later as files four times
  // the expected size.
  if (got != fourcc || cam.negotiated_width() != width || cam.negotiated_height() != height) {
    std::printf("NOTE: asked for %s %ux%u -- the driver substituted the above. "
                "Run --list to see what it really supports.\n",
                format_text.c_str(), width, height);
  }

  int captured = 0;
  int rc = 0;
  while (!g_stop && (frames == 0 || captured < frames)) {
    camera::Frame frame;
    if (!cam.capture(frame, timeout_ms)) {
      std::fprintf(stderr, "%s\n", cam.last_error().c_str());
      rc = 1;
      break;
    }
    std::printf("frame %d: seq=%u %zu bytes\n", captured, frame.sequence, frame.size);
    if (!output_dir.empty() && !write_frame(output_dir, captured, got, frame)) {
      rc = 1;
      break;
    }
    ++captured;
  }

  cam.stop();
  cam.close();
  std::printf("captured %d frame(s)\n", captured);
  return rc;
}
