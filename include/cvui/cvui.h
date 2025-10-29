#pragma once

#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace cvui {
namespace detail {
static std::string g_window;
static cv::Point g_lastClick(-10000, -10000);
static cv::Point g_mouse(0, 0);

static void onMouse(int event, int x, int y, int /*flags*/, void* /*userdata*/) {
  g_mouse = {x, y};
  if (event == cv::EVENT_LBUTTONUP) {
    g_lastClick = {x, y};
  }
}
} // namespace detail

inline void init(const std::string& window) {
  detail::g_window = window;
  // Window must exist before setting callback; user may also create it.
  try { cv::namedWindow(window); } catch(...) {}
  cv::setMouseCallback(window, detail::onMouse, nullptr);
}

inline void context(const std::string& /*window*/) {
  // Kept for compatibility. No-op in this minimal version.
}

inline void update() {
  // No-op: we use OpenCV callbacks directly.
}

inline void window(cv::Mat& where, int x, int y, int w, int h, const std::string& title) {
  cv::Rect r(x, y, w, h);
  cv::rectangle(where, r, cv::Scalar(80, 80, 80), 1);
  // Title bar
  cv::rectangle(where, cv::Rect(x, y, w, 24), cv::Scalar(50, 50, 50), cv::FILLED);
  cv::putText(where, title, {x + 6, y + 17}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
}

inline void text(cv::Mat& where, int x, int y, const std::string& content, double size = 0.5, cv::Scalar color = cv::Scalar(220, 220, 220)) {
  cv::putText(where, content, {x, y}, cv::FONT_HERSHEY_SIMPLEX, size, color, 1, cv::LINE_AA);
}

inline bool button(cv::Mat& where, int x, int y, int w, int h, const std::string& label) {
  cv::Rect r(x, y, w, h);
  bool hover = r.contains(detail::g_mouse);
  cv::Scalar fill = hover ? cv::Scalar(90, 90, 90) : cv::Scalar(70, 70, 70);
  cv::rectangle(where, r, fill, cv::FILLED);
  cv::rectangle(where, r, cv::Scalar(120, 120, 120), 1);
  int baseline = 0;
  cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);
  cv::Point org(x + (w - ts.width) / 2, y + (h + ts.height) / 2);
  cv::putText(where, label, org, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

  // Click detection: return true when last mouse-up happened within this rect.
  if (r.contains(detail::g_lastClick)) {
    // Consume the click so only one widget triggers per frame.
    detail::g_lastClick = {-10000, -10000};
    return true;
  }
  return false;
}
}

