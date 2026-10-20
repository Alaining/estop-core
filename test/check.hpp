#pragma once

// Plain asserts for the host tests: every failure is printed, main() returns the verdict.

#include <cstdio>

namespace estop::test {

inline int failures = 0;

inline void check(bool ok, const char *what, const char *file, int line) {
  if (!ok) {
    std::printf("FAIL %s:%d: %s\n", file, line, what);
    failures++;
  }
}

inline int verdict(const char *name) {
  std::printf("%s: %s (%d failures)\n", name, failures == 0 ? "PASS" : "FAIL", failures);
  return failures == 0 ? 0 : 1;
}

} // namespace estop::test

#define CHECK(x) ::estop::test::check((x), #x, __FILE__, __LINE__)
