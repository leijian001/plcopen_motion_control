#include <stdio.h>
#include <map>
#include <string>

#ifdef _WIN32
#include <intrin.h>
#else
#include <x86intrin.h>
#endif

int main()
{
  std::map<std::string, double> time_stamps;
  uint64_t i;
  double d;
  i = __rdtsc();
  __rdtsc();
  time_stamps.insert(std::make_pair("test", (__rdtsc() - i) / 1.8));
  d = (__rdtsc() - i) / 1.8;
  printf("%f ns\n", d);
}