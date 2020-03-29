#ifndef _WIN32
#include <unistd.h>
#include <sys/time.h>
#if __APPLE__
#include <mach/mach_time.h>
#endif
#else
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <cstdio>
#include <cstdarg>
#include <ctime>

#include "CBasics.hpp"

#if __APPLE__
static u64 MachToDolphinNum;
static u64 MachToDolphinDenom;
#elif _WIN32
static LARGE_INTEGER PerfFrequency;
#endif

namespace urde {

void CBasics::Initialize() {
#if __APPLE__
  mach_timebase_info_data_t timebase;
  mach_timebase_info(&timebase);
  MachToDolphinNum = GetGCTicksPerSec() * timebase.numer;
  MachToDolphinDenom = 1000000000ull * timebase.denom;
#elif _WIN32
  QueryPerformanceFrequency(&PerfFrequency);
#endif
}

u64 CBasics::GetGCTicks() {
#if __APPLE__
  return mach_absolute_time() * MachToDolphinNum / MachToDolphinDenom;
#elif __linux__ || __FreeBSD__
  struct timespec tp;
  clock_gettime(CLOCK_MONOTONIC, &tp);

  return u64((tp.tv_sec * 1000000000ull) + tp.tv_nsec) * GetGCTicksPerSec() / 1000000000ull;
#elif _WIN32
  LARGE_INTEGER perf;
  QueryPerformanceCounter(&perf);
  perf.QuadPart *= GetGCTicksPerSec();
  perf.QuadPart /= PerfFrequency.QuadPart;
  return perf.QuadPart;
#else
  return 0;
#endif
}

const u64 CBasics::SECONDS_TO_2000 = 946684800LL;
const u64 CBasics::TICKS_PER_SECOND = 60750000LL;

static struct tm* localtime_r(const time_t& time, struct tm& timeSt, long& gmtOff) {
#ifndef _WIN32
  auto ret = ::localtime_r(&time, &timeSt);
  if (!ret)
    return nullptr;
  gmtOff = ret->tm_gmtoff;
  return ret;
#else
  struct tm _gmSt;
  auto reta = localtime_s(&timeSt, &time);
  auto retb = gmtime_s(&_gmSt, &time);
  if (reta || retb)
    return nullptr;
  gmtOff = mktime(&timeSt) - mktime(&_gmSt);
  return &timeSt;
#endif
}

OSTime CBasics::ToWiiTime(std::chrono::system_clock::time_point time) {
  auto sec = std::chrono::time_point_cast<std::chrono::seconds>(time);
  auto us = std::chrono::duration_cast<std::chrono::microseconds>((time - sec)).count();
  time_t sysTime = std::chrono::system_clock::to_time_t(sec);

  struct tm _timeSt;
  long gmtOff;
  struct tm* timeSt = localtime_r(sysTime, _timeSt, gmtOff);
  if (!timeSt)
    return 0;

  /* Returning local */
  return OSTime(TICKS_PER_SECOND * ((sysTime + gmtOff) - SECONDS_TO_2000) + us * TICKS_PER_SECOND / 1000000);
}

std::chrono::system_clock::time_point CBasics::FromWiiTime(OSTime wiiTime) {
  auto div = urde::div(SECONDS_TO_2000 + wiiTime, TICKS_PER_SECOND);
  time_t time = time_t(div.quot);

  time_t sysTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  struct tm _timeSt;
  long gmtOff;
  struct tm* timeSt = localtime_r(sysTime, _timeSt, gmtOff);
  if (!timeSt)
    return std::chrono::system_clock::from_time_t(0);

  /* Returning GMT */
  return std::chrono::system_clock::from_time_t(time - gmtOff) +
         std::chrono::microseconds(div.rem * 1000000 / TICKS_PER_SECOND);
}

OSCalendarTime CBasics::ToCalendarTime(std::chrono::system_clock::time_point time) {
  OSCalendarTime ret;

  auto sec = std::chrono::time_point_cast<std::chrono::seconds>(time);
  auto us = std::chrono::duration_cast<std::chrono::microseconds>((time - sec)).count();
  time_t sysTime = std::chrono::system_clock::to_time_t(sec);
  struct tm _timeSt;
  long gmtOff;
  struct tm* timeSt = localtime_r(sysTime, _timeSt, gmtOff);
  if (!timeSt)
    return {};

  ret.x0_sec = timeSt->tm_sec;
  ret.x4_min = timeSt->tm_min;
  ret.x8_hour = timeSt->tm_hour;
  ret.xc_mday = timeSt->tm_mday;
  ret.x10_mon = timeSt->tm_mon;
  ret.x14_year = timeSt->tm_year + 1900;
  ret.x18_wday = timeSt->tm_wday;
  ret.x1c_yday = timeSt->tm_yday;

  auto div = urde::div(us, decltype(us)(1000));
  ret.x20_msec = div.quot;
  ret.x24_usec = div.rem;

  return ret;
}

} // namespace urde
