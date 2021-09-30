//These are libraries which contain useful functions
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>

#define MAP_SIZE 262144UL
#define MEM_LOC  0x40000000
#define FIFO_LOC 0x41000000
#define FIFO_STATUS_OFFSET 0x00000004
#define FIFO_DATA_OFFSET 0x00000008
 
int main(int argc, char **argv)
{
  int fd;		                  //File identifier
  int numSamples = 4096;	    //Number of samples to collect
  void *cfg;		              //A pointer to a memory location.  The * indicates that it is a pointer - it points to a location in memory
  char *name = "/dev/mem";	  //Name of the memory resource

  uint32_t i;
  uint32_t tmp, fifoStatus;
  uint32_t *data;
  uint8_t  resetFlag = 0;
  uint8_t saveType = 2;
  FILE *ptr;

  clock_t start, stop;

  /*
   * Parse the input arguments
   */
  int c;
  while ((c = getopt(argc,argv,"n:rt:")) != -1) {
    switch (c) {
      case 'n':
        numSamples = atoi(optarg);
        break;
      case 'r':
        resetFlag = 1;
        break;
      case 't':
        saveType = atoi(optarg);
        break;

      case '?':
        if (isprint (optopt))
            fprintf (stderr, "Unknown option `-%c'.\n", optopt);
        else
            fprintf (stderr,
                    "Unknown option character `\\x%x'.\n",
                    optopt);
        return 1;

      default:
        abort();
        break;
    }
  }
  /*
   * Allocate memory
   */
  if (saveType != 0) {
    //Clear file and open for writing if saving to file directly
    ptr = fopen("saved-scan-data.bin","wb");
  } else {
    data = (uint32_t *) malloc(numSamples * sizeof(uint32_t));
    if (!data) {
      printf("Error allocating memory");
      return -1;
    }
  }
  /*
   * This returns a file identifier corresponding to the memory, and allows for reading and writing.  O_RDWR is just a constant
   */
  if((fd = open(name, O_RDWR)) < 0) {
    perror("open");
    return 1;
  }
  /*
   * mmap maps the memory location FIFO_LOC to the pointer cfg, which "points" to that location in memory.
   */
  cfg = mmap(0,MAP_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,fd,FIFO_LOC);
  /*
   * Optionally reset the FIFO before collecting data
   */
  if (resetFlag == 1) {
    *((uint32_t *)(cfg + FIFO_LOC)) = 1;
    usleep(10);
  }
  /*
   * Read FIFO status and wait until FPGA indicates that data can be read
   */
  while ((fifoStatus = *((uint32_t *)(cfg + FIFO_STATUS_OFFSET))) != 3) {
    usleep(10);
  }
  /*
   * Read data up to numSamples
   */
  if (saveType != 2) {
    for (i = 0;i < numSamples;i++) {
      *(data + i) = *((uint32_t *)(cfg  + FIFO_DATA_OFFSET));
    }
  } else {
    for (i = 0;i < numSamples;i++) {
        tmp = *((uint32_t *)(cfg + FIFO_DATA_OFFSET));
        fwrite(&tmp,4,1,ptr);
    }
  }
  
  /*
   * Print data to command line - this is used to pass data to Python server
   */
  if (saveType == 0) {
    for (i = 0;i < numSamples;i++) {
      printf("%08x\n",*(data + i));
    }
    free(data); //Frees up the memory allocated to data
  } else if (saveType == 1) {
    fwrite(data,4,(size_t)(numSamples),ptr);
    fclose(ptr);
    free(data);
  } else {
    fclose(ptr);
  }
  
  //Resets the FIFO
  *((uint32_t *)(cfg + 0)) = 1;
  //Unmap cfg from pointing to the previous location in memory
  munmap(cfg, MAP_SIZE);
  return 0;	//C functions should have a return value - 0 is the usual "no error" return value
}
