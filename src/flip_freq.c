#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>


void change_freq(char *freq, FILE *h)
{
if ( write(fileno(h), freq, strlen(freq)) != strlen(freq) )
{
        perror("Could not append line to file");
        exit(1);
}
}


/* Run a lot of computations */
void stress()
{
  double value = 1.0;
  for(int i=0; i<10000000; i++)
  {
    value += sqrt(4.5) * sqrt(value) + 0.01;
  }
}



int main() {

FILE* h = fopen("/sys/devices/system/cpu/cpufreq/policy4/scaling_setspeed", "w");

int freq;
char freq_str[30];

  struct timespec tstart={0,0}, tend={0,0};


for (freq = 200000; freq <= 2000000; freq += 100000)
{

clock_gettime(CLOCK_MONOTONIC, &tstart);
sprintf(freq_str, "%i", freq);
change_freq(freq_str, h);
stress();

/*
for(int i=0; i<1000; i++)
{
if(i%2 == 0) freq = low;
else freq = high;
}
*/

  clock_gettime(CLOCK_MONOTONIC, &tend);
    printf("%8i %.5f\n", freq,
            ( ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) -
           ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec)));
}

fclose(h);

  return 0;
}

