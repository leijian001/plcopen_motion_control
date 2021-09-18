#include <stdio.h>
#include <map>
#include <string>
#include <time.h>

#define NSEC_PER_SEC (1000000000L)
#define DIFF_NS(A, B)                                                          \
  (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + ((B).tv_nsec) - (A).tv_nsec)

int main()
{
  std::map<std::string, int> time_stamps;
  struct timespec next_period, start_time, end_time;
  clock_gettime(CLOCK_MONOTONIC, &start_time);
  clock_gettime(CLOCK_MONOTONIC, &next_period);
  clock_gettime(CLOCK_MONOTONIC, &end_time);
  time_stamps.insert(std::make_pair("test", DIFF_NS(start_time, end_time)));
  clock_gettime(CLOCK_MONOTONIC, &end_time);
  ;
  printf("%d ns\n", DIFF_NS(start_time, end_time));
}