# General description or introduction of the problem and your solution

This homework requires us to implement mulit-core gaussian blur in systemC as modules in tiny32-mc with DMA. Our main porgram has to utilize mutex lock for limited resources like DMA, memory bandwidth to share between two gaussian modules.

# Implementation details (data structure, flows and algorithms)

tiny32-mc/mc_main.cpp
```
#include "dma.h"
...
#include "GaussianFilter.h"
#include "fe310_plic.h"

struct TinyOptions : public Options {
public:
    ...
    addr_t dma_start_addr = 0x40000000;
    addr_t dma_end_addr = 0x40001000;
    addr_t gaussianFilter_start_addr = 0x43000000;
    addr_t gaussianFilter_size = 0x01000000;
    addr_t gaussianFilter_end_addr = gaussianFilter_start_addr + gaussianFilter_size - 1;
    addr_t gaussianFilter_1_start_addr = 0x44000000;
    addr_t gaussianFilter_1_size = 0x01000000;
    addr_t gaussianFilter_1_end_addr = gaussianFilter_1_start_addr + gaussianFilter_1_size - 1;
    ...
}

int sc_main(int argc, char **argv) {
	...
  SimpleBus<4, 7> bus("SimpleBus");
  FE310_PLIC<2, 64, 96, 32> plic("PLIC");
	SimpleDMA dma("SimpleDMA", 4);
	GaussianFilter gaussian_filter("gaussian_filter");
	GaussianFilter gaussian_filter_1("gaussian_filter_1");
  ...
  bus.ports[3] = new PortMapping(opt.dma_start_addr, opt.dma_end_addr);
	bus.ports[4] = new PortMapping(opt.gaussianFilter_start_addr, opt.gaussianFilter_end_addr);
	bus.ports[5] = new PortMapping(opt.gaussianFilter_1_start_addr, opt.gaussianFilter_1_end_addr);
	bus.ports[6] = new PortMapping(opt.plic_start_addr, opt.plic_end_addr);
  ...
  PeripheralWriteConnector dma_connector("SimpleDMA-Connector");  // to respect ISS bus locking
	dma_connector.isock.bind(bus.tsocks[3]);
	dma.isock.bind(dma_connector.tsock);
	dma_connector.bus_lock = bus_lock;
  ...
  bus.isocks[3].bind(dma.tsock);
	bus.isocks[4].bind(gaussian_filter.tsock);
	bus.isocks[5].bind(gaussian_filter_1.tsock);
	bus.isocks[6].bind(plic.tsock);
  ...
  plic.target_harts[0] = &core0;
	plic.target_harts[1] = &core1;
	dma.plic = &plic;

```
The code above only inclued changed codes from previous mc_main.cpp. At first, we have to include three new header files dma.h for direct memory access, GaussianFilter.h for gaussian module and fe310_plic for interrecpt control. In class TinyOptions, new address about dma and two gaussian modules have to add.
Finally in sc_main, we change SimpleBus from <3,3> to <4,7> and add new ports and bind new tsock.

muti_gaussian/main.c

```

#include <string.h>
#include "stdio.h"
#include "math.h"
#include "lena_std_short.h"

// Gaussian Filter ACC
static char* const GAUSSIANFILTER_START_ADDR = reinterpret_cast<char* const>(0x43000000);
static char* const GAUSSIANFILTER_READ_ADDR  = reinterpret_cast<char* const>(0x43000004);

// Gaussian Filter ACC 1
static char* const GAUSSIANFILTER_START_1_ADDR = reinterpret_cast<char* const>(0x44000000);
static char* const GAUSSIANFILTER_READ_1_ADDR  = reinterpret_cast<char* const>(0x44000004);

// DMA 
static volatile uint32_t * const DMA_SRC_ADDR  = (uint32_t * const)0x40000000;
static volatile uint32_t * const DMA_DST_ADDR  = (uint32_t * const)0x40000004;
static volatile uint32_t * const DMA_LEN_ADDR  = (uint32_t * const)0x40000008;
static volatile uint32_t * const DMA_OP_ADDR   = (uint32_t * const)0x4000000C;
static volatile uint32_t * const DMA_STAT_ADDR = (uint32_t * const)0x40000010;
static const uint32_t DMA_OP_MEMCPY = 1;

bool _is_using_dma = true;

unsigned int ReadfromByteArray(unsigned char* array, unsigned int offset) {
	unsigned int output = (array[offset] << 0) | (array[offset + 1] << 8) | (array[offset + 2] << 16) | (array[offset + 3] << 24);
	return output;
}

void write_data_to_ACC(char* ADDR, unsigned char* buffer, int len){
  if(_is_using_dma){  
    // Using DMA 
    *DMA_SRC_ADDR = (uint32_t)(buffer);
    *DMA_DST_ADDR = (uint32_t)(ADDR);
    *DMA_LEN_ADDR = len;
    *DMA_OP_ADDR  = DMA_OP_MEMCPY;
  }else{
    // Directly Send
    memcpy(ADDR, buffer, sizeof(unsigned char)*len);
  }
}
void read_data_from_ACC(char* ADDR, unsigned char* buffer, int len){
  if(_is_using_dma){
    // Using DMA 
    *DMA_SRC_ADDR = (uint32_t)(ADDR);
    *DMA_DST_ADDR = (uint32_t)(buffer);
    *DMA_LEN_ADDR = len;
    *DMA_OP_ADDR  = DMA_OP_MEMCPY;
  }else{
    // Directly Read
    memcpy(buffer, ADDR, sizeof(unsigned char)*len);
  }
}

int sem_init (uint32_t *__sem, uint32_t count) __THROW
{
  *__sem=count;
  return 0;
}

int sem_wait (uint32_t *__sem) __THROW
{
  uint32_t value, success; //RV32A
  __asm__ __volatile__("\
L%=:\n\t\
     lr.w %[value],(%[__sem])            # load reserved\n\t\
     beqz %[value],L%=                   # if zero, try again\n\t\
     addi %[value],%[value],-1           # value --\n\t\
     sc.w %[success],%[value],(%[__sem]) # store conditionally\n\t\
     bnez %[success], L%=                # if the store failed, try again\n\t\
"
    : [value] "=r"(value), [success]"=r"(success)
    : [__sem] "r"(__sem)
    : "memory");
  return 0;
}

int sem_post (uint32_t *__sem) __THROW
{
  uint32_t value, success; //RV32A
  __asm__ __volatile__("\
L%=:\n\t\
     lr.w %[value],(%[__sem])            # load reserved\n\t\
     addi %[value],%[value], 1           # value ++\n\t\
     sc.w %[success],%[value],(%[__sem]) # store conditionally\n\t\
     bnez %[success], L%=                # if the store failed, try again\n\t\
"
    : [value] "=r"(value), [success]"=r"(success)
    : [__sem] "r"(__sem)
    : "memory");
  return 0;
}

int barrier(uint32_t *__sem, uint32_t *__lock, uint32_t *counter, uint32_t thread_count) {
	sem_wait(__lock);
	if (*counter == thread_count - 1) { //all finished
		*counter = 0;
		sem_post(__lock);
		for (int j = 0; j < thread_count - 1; ++j) sem_post(__sem);
	} else {
		(*counter)++;
		sem_post(__lock);
		sem_wait(__sem);
	}
	return 0;
}

void sprintfloat(char* buf, float num) {
	const char* tmpSign = (num < 0) ? "-" : "+";
	float tmpVal = (num < 0) ? -num : num;

	int tmpInt1 = (int) tmpVal;                  // Get the integer (678).
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
	int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

	// Print as parts, note that you need 0-padding for fractional bit.
	sprintf(buf, "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);
}

//Total number of cores
//static const int PROCESSORS = 2;
#define PROCESSORS 2
//the barrier synchronization objects
uint32_t barrier_counter=0; 
uint32_t barrier_lock; 
uint32_t barrier_sem; 
//the mutex object to control global summation
uint32_t lock;  
//print synchronication semaphore (print in core order)
uint32_t print_sem[PROCESSORS]; 
float pi_over_4 = 0;
unsigned char result_saver[260][260][4];

int main(unsigned hart_id) {

	/////////////////////////////
	// thread and barrier init //
	/////////////////////////////
	if (hart_id == 0) {
		// create a barrier object with a count of PROCESSORS
		sem_init(&barrier_lock, 1);
		sem_init(&barrier_sem, 0); //lock all cores initially
		for(int i=0; i< PROCESSORS; ++i){
			sem_init(&print_sem[i], 0); //lock printing initially
		}
		// Create mutex lock
		sem_init(&lock, 1);
	}

	/////////////////////////////////////////
	// accumulate local sum to shared data //
	/////////////////////////////////////////
  unsigned char* source_array= lena_std_short_bmp;

  unsigned int input_rgb_raw_data_offset = ReadfromByteArray(source_array, 10);
  unsigned int width = ReadfromByteArray(source_array, 18);
  unsigned int length = ReadfromByteArray(source_array, 22);
  unsigned int bytes_per_pixel = ReadfromByteArray(source_array, 28) / 8;
  unsigned char* source_bitmap = &source_array[input_rgb_raw_data_offset];
  sem_wait(&lock);
  printf ("hart_id = %d\n",hart_id);
  printf("======================================\n");
  printf("\t  Reading from array\n");
  printf("======================================\n");
  printf(" input_rgb_raw_data_offset\t= %d\n", input_rgb_raw_data_offset);
  printf(" width\t\t\t\t= %d\n", width);
  printf(" length\t\t\t\t= %d\n", length);
  printf(" bytes_per_pixel\t\t= %d\n",bytes_per_pixel);
  printf("======================================\n");
  sem_post(&lock);

  int start_width = width / PROCESSORS * hart_id, end_width = width / PROCESSORS * hart_id + width / PROCESSORS;
  unsigned char  buffer[4] = {0};
  for(int i = start_width; i < end_width; i++){
    for(int j = 0; j < length; j++){
      for(int v = -1; v <= 1; v ++){
        for(int u = -1; u <= 1; u++){
          if((v + i) >= 0  &&  (v + i ) < width && (u + j) >= 0 && (u + j) < length ){
            buffer[0] = *(source_bitmap + bytes_per_pixel * ((j + u) * width + (i + v)) + 2);
            buffer[1] = *(source_bitmap + bytes_per_pixel * ((j + u) * width + (i + v)) + 1);
            buffer[2] = *(source_bitmap + bytes_per_pixel * ((j + u) * width + (i + v)) + 0);
            buffer[3] = 0;
          }else{
            buffer[0] = 0;
            buffer[1] = 0;
            buffer[2] = 0;
            buffer[3] = 0;
          }
          //printf ("i = %d j = %d v = %d u = %d\n",i,j,v,u);
          sem_wait(&lock);
          //printf ("i = %d j = %d v = %d u = %d\n",i,j,v,u);
          if (hart_id == 0) write_data_to_ACC(GAUSSIANFILTER_START_ADDR, buffer, 4);
          else write_data_to_ACC(GAUSSIANFILTER_START_1_ADDR, buffer, 4);
          sem_post(&lock);
          //printf ("Finish write data\n");
        }
      }
      //printf ("Start read data\n");
      sem_wait(&lock);
      if (hart_id == 0) read_data_from_ACC(GAUSSIANFILTER_READ_ADDR, buffer, 4);
      else read_data_from_ACC(GAUSSIANFILTER_READ_1_ADDR, buffer, 4);
      result_saver[i][j][0] = buffer[0];
      result_saver[i][j][1] = buffer[1];
      result_saver[i][j][2] = buffer[2];
      result_saver[i][j][3] = buffer[3];
      sem_post(&lock);
      //printf ("buffer = %d %d %d %d\n",buffer[0],buffer[1],buffer[2],buffer[3]);
      //printf ("Finish read data\n");
    }
  }

	////////////////////////////
	// barrier to synchronize //
	////////////////////////////
	//Wait for all threads to finish
	barrier(&barrier_sem, &barrier_lock, &barrier_counter, PROCESSORS);

	if (hart_id == 0) {  // Core 0 print first and then others
    sem_wait(&lock);
		for(int i = start_width; i < start_width+10; i++){
      for(int j = 0; j < 10; j++){
        printf ("[%d %d] %d %d %d %d\n",i,j,result_saver[i][j][0],result_saver[i][j][1],result_saver[i][j][2],result_saver[i][j][3]);
      }
    }
    sem_post(&lock);
	} else {
		for (int i = 1; i < PROCESSORS; ++i) {
      sem_wait(&lock);
      for(int i = start_width; i < start_width+10; i++){
        for(int j = 0; j < 10; j++){
          //sem_wait(&print_sem[i]); 
          printf ("[%d %d] %d %d %d %d\n",i,j,result_saver[i][j][0],result_saver[i][j][1],result_saver[i][j][2],result_saver[i][j][3]);
          //sem_post(&print_sem[i + 1]);
        }
      }
      sem_post(&lock);
		}
	}

	return 0;
}

```

The overall logic in main.cpp is very similiar to single-core version. First, get data from lena_std_short_bmp. We have to add mutex to avoid racing printf function for printing result from parsing lena_std_short_bmp data. Then, calculate start index based on hart_id. In calculation process every dma asscess have to ge through mutex to avoid reacing. Finally, we print out the result for verification.


# Result

* single-core

```

[0 0] 162 220 191 0
[0 1] 69 250 133 0
[0 2] 93 32 179 0
[0 3] 114 91 213 0
[0 4] 189 251 91 0
[0 5] 24 152 244 0
[0 6] 46 188 15 0
[0 7] 27 143 251 0
[0 8] 255 59 217 0
[0 9] 213 208 101 0
[1 0] 110 34 212 0
[1 1] 124 125 12 0
[1 2] 116 157 249 0
[1 3] 175 38 92 0
[1 4] 46 4 54 0
[1 5] 152 186 215 0
[1 6] 167 219 9 0
[1 7] 109 87 204 0
[1 8] 30 171 54 0
[1 9] 235 54 147 0
[2 0] 78 229 182 0
[2 1] 64 43 197 0
[2 2] 98 168 250 0
[2 3] 213 116 197 0
[2 4] 71 29 113 0
[2 5] 151 176 222 0
[2 6] 156 189 34 0
[2 7] 88 39 197 0
[2 8] 19 170 29 0
[2 9] 237 88 165 0
[3 0] 43 237 182 0
[3 1] 75 84 232 0
[3 2] 153 221 76 0
[3 3] 231 95 236 0
[3 4] 13 145 37 0
[3 5] 56 230 82 0
[3 6] 70 4 129 0
[3 7] 55 210 91 0
[3 8] 38 183 27 0
[3 9] 226 87 171 0
[4 0] 113 130 70 0
[4 1] 202 29 184 0
[4 2] 254 84 222 0
[4 3] 235 76 247 0
[4 4] 177 1 191 0
[4 5] 174 21 162 0
[4 6] 224 113 204 0
[4 7] 18 156 252 0
[4 8] 11 114 233 0
[4 9] 160 222 75 0
[5 0] 120 105 29 0
[5 1] 248 94 219 0
[5 2] 59 200 63 0
[5 3] 6 150 47 0
[5 4] 169 39 192 0
[5 5] 159 45 158 0
[5 6] 222 135 213 0
[5 7] 255 146 223 0
[5 8] 212 35 147 0
[5 9] 108 118 254 0
[6 0] 85 47 180 0
[6 1] 251 138 211 0
[6 2] 94 69 147 0
[6 3] 57 43 140 0
[6 4] 4 225 52 0
[6 5] 7 187 11 0
[6 6] 19 158 242 0
[6 7] 239 82 163 0
[6 8] 162 198 63 0
[6 9] 58 25 198 0
[7 0] 153 200 16 0
[7 1] 58 57 95 0
[7 2] 125 186 237 0
[7 3] 106 166 207 0
[7 4] 82 119 166 0
[7 5] 58 4 85 0
[7 6] 10 108 208 0
[7 7] 185 218 83 0
[7 8] 74 43 217 0
[7 9] 208 119 63 0
[8 0] 17 197 230 0
[8 1] 135 249 27 0
[8 2] 130 222 29 0
[8 3] 110 161 212 0
[8 4] 82 120 198 0
[8 5] 9 222 102 0
[8 6] 162 244 171 0
[8 7] 36 29 234 0
[8 8] 150 87 42 0
[8 9] 42 189 138 0
[9 0] 80 90 120 0
[9 1] 150 32 94 0
[9 2] 73 118 229 0
[9 3] 33 13 128 0
[9 4] 4 217 107 0
[9 5] 158 50 7 0
[9 6] 0 27 58 0
[9 7] 90 31 65 0
[9 8] 222 143 132 0
[9 9] 171 79 51 0
[128 0] 36 211 88 0
[128 1] 127 85 244 0
[128 2] 83 242 103 0
[128 3] 206 66 159 0
[128 4] 238 111 233 0
[128 5] 4 141 38 0
[128 6] 36 148 35 0
[128 7] 68 189 33 0
[128 8] 104 250 63 0
[128 9] 119 250 66 0
[129 0] 77 238 106 0
[129 1] 202 123 20 0
[129 2] 133 4 143 0
[129 3] 226 75 195 0
[129 4] 8 145 242 0
[129 5] 39 179 20 0
[129 6] 65 163 32 0
[129 7] 81 197 59 0
[129 8] 102 244 63 0
[129 9] 114 239 30 0
[130 0] 159 46 124 0
[130 1] 30 174 23 0
[130 2] 168 21 133 0
[130 3] 242 99 202 0
[130 4] 31 179 247 0
[130 5] 56 209 7 0
[130 6] 80 196 21 0
[130 7] 104 216 52 0
[130 8] 117 248 59 0
[130 9] 110 2 39 0
[131 0] 243 136 164 0
[131 1] 88 10 74 0
[131 2] 187 81 169 0
[131 3] 1 144 242 0
[131 4] 43 207 42 0
[131 5] 55 243 70 0
[131 6] 71 252 97 0
[131 7] 102 250 103 0
[131 8] 119 8 84 0
[131 9] 108 32 62 0
[132 0] 34 213 249 0
[132 1] 114 101 190 0
[132 2] 201 156 252 0
[132 3] 11 192 34 0
[132 4] 40 235 95 0
[132 5] 47 21 139 0
[132 6] 67 34 152 0
[132 7] 94 13 124 0
[132 8] 104 13 96 0
[132 9] 101 55 89 0
[133 0] 60 237 39 0
[133 1] 144 131 238 0
[133 2] 233 185 26 0
[133 3] 39 224 53 0
[133 4] 59 16 124 0
[133 5] 63 48 165 0
[133 6] 83 46 150 0
[133 7] 106 21 109 0
[133 8] 116 23 96 0
[133 9] 117 75 123 0
[134 0] 68 229 45 0
[134 1] 171 132 254 0
[134 2] 15 204 73 0
[134 3] 83 2 107 0
[134 4] 109 45 149 0
[134 5] 109 59 166 0
[134 6] 116 57 159 0
[134 7] 131 49 143 0
[134 8] 141 60 138 0
[134 9] 142 93 159 0
[135 0] 79 2 103 0
[135 1] 194 176 66 0
[135 2] 43 249 128 0
[135 3] 111 27 134 0
[135 4] 136 48 146 0
[135 5] 134 56 148 0
[135 6] 136 68 151 0
[135 7] 144 96 173 0
[135 8] 152 113 183 0
[135 9] 156 107 177 0
[136 0] 80 47 160 0
[136 1] 201 224 105 0
[136 2] 56 23 122 0
[136 3] 128 45 135 0
[136 4] 146 68 175 0
[136 5] 135 73 158 0
[136 6] 135 75 140 0
[136 7] 143 108 180 0
[136 8] 159 123 194 0
[136 9] 168 108 177 0
[137 0] 46 38 168 0
[137 1] 181 212 117 0
[137 2] 55 19 143 0
[137 3] 143 60 184 0
[137 4] 169 90 237 0
[137 5] 148 97 216 0
[137 6] 143 102 197 0
[137 7] 153 110 213 0
[137 8] 165 106 196 0
[137 9] 167 103 168 0

```

* multi-core

```
[0 0] 162 220 191 0
[0 1] 69 250 133 0
[0 2] 93 32 179 0
[0 3] 114 91 213 0
[0 4] 189 251 91 0
[0 5] 24 152 244 0
[0 6] 46 188 15 0
[0 7] 27 143 251 0
[0 8] 255 59 217 0
[0 9] 213 208 101 0
[1 0] 110 34 212 0
[1 1] 124 125 12 0
[1 2] 116 157 249 0
[1 3] 175 38 92 0
[1 4] 46 4 54 0
[1 5] 152 186 215 0
[1 6] 167 219 9 0
[1 7] 109 87 204 0
[1 8] 30 171 54 0
[1 9] 235 54 147 0
[2 0] 78 229 182 0
[2 1] 64 43 197 0
[2 2] 98 168 250 0
[2 3] 213 116 197 0
[2 4] 71 29 113 0
[2 5] 151 176 222 0
[2 6] 156 189 34 0
[2 7] 88 39 197 0
[2 8] 19 170 29 0
[2 9] 237 88 165 0
[3 0] 43 237 182 0
[3 1] 75 84 232 0
[3 2] 153 221 76 0
[3 3] 231 95 236 0
[3 4] 13 145 37 0
[3 5] 56 230 82 0
[3 6] 70 4 129 0
[3 7] 55 210 91 0
[3 8] 38 183 27 0
[3 9] 226 87 171 0
[4 0] 113 130 70 0
[4 1] 202 29 184 0
[4 2] 254 84 222 0
[4 3] 235 76 247 0
[4 4] 177 1 191 0
[4 5] 174 21 162 0
[4 6] 224 113 204 0
[4 7] 18 156 252 0
[4 8] 11 114 233 0
[4 9] 160 222 75 0
[5 0] 120 105 29 0
[5 1] 248 94 219 0
[5 2] 59 200 63 0
[5 3] 6 150 47 0
[5 4] 169 39 192 0
[5 5] 159 45 158 0
[5 6] 222 135 213 0
[5 7] 255 146 223 0
[5 8] 212 35 147 0
[5 9] 108 118 254 0
[6 0] 85 47 180 0
[6 1] 251 138 211 0
[6 2] 94 69 147 0
[6 3] 57 43 140 0
[6 4] 4 225 52 0
[6 5] 7 187 11 0
[6 6] 19 158 242 0
[6 7] 239 82 163 0
[6 8] 162 198 63 0
[6 9] 58 25 198 0
[7 0] 153 200 16 0
[7 1] 58 57 95 0
[7 2] 125 186 237 0
[7 3] 106 166 207 0
[7 4] 82 119 166 0
[7 5] 58 4 85 0
[7 6] 10 108 208 0
[7 7] 185 218 83 0
[7 8] 74 43 217 0
[7 9] 208 119 63 0
[8 0] 17 197 230 0
[8 1] 135 249 27 0
[8 2] 130 222 29 0
[8 3] 110 161 212 0
[8 4] 82 120 198 0
[8 5] 9 222 102 0
[8 6] 162 244 171 0
[8 7] 36 29 234 0
[8 8] 150 87 42 0
[8 9] 42 189 138 0
[9 0] 80 90 120 0
[9 1] 150 32 94 0
[9 2] 73 118 229 0
[9 3] 33 13 128 0
[9 4] 4 217 107 0
[9 5] 158 50 7 0
[9 6] 0 27 58 0
[9 7] 90 31 65 0
[9 8] 222 143 132 0
[9 9] 171 79 51 0
[128 0] 36 211 88 0
[128 1] 127 85 244 0
[128 2] 83 242 103 0
[128 3] 206 66 159 0
[128 4] 238 111 233 0
[128 5] 4 141 38 0
[128 6] 36 148 35 0
[128 7] 68 189 33 0
[128 8] 104 250 63 0
[128 9] 119 250 66 0
[129 0] 77 238 106 0
[129 1] 202 123 20 0
[129 2] 133 4 143 0
[129 3] 226 75 195 0
[129 4] 8 145 242 0
[129 5] 39 179 20 0
[129 6] 65 163 32 0
[129 7] 81 197 59 0
[129 8] 102 244 63 0
[129 9] 114 239 30 0
[130 0] 159 46 124 0
[130 1] 30 174 23 0
[130 2] 168 21 133 0
[130 3] 242 99 202 0
[130 4] 31 179 247 0
[130 5] 56 209 7 0
[130 6] 80 196 21 0
[130 7] 104 216 52 0
[130 8] 117 248 59 0
[130 9] 110 2 39 0
[131 0] 243 136 164 0
[131 1] 88 10 74 0
[131 2] 187 81 169 0
[131 3] 1 144 242 0
[131 4] 43 207 42 0
[131 5] 55 243 70 0
[131 6] 71 252 97 0
[131 7] 102 250 103 0
[131 8] 119 8 84 0
[131 9] 108 32 62 0
[132 0] 34 213 249 0
[132 1] 114 101 190 0
[132 2] 201 156 252 0
[132 3] 11 192 34 0
[132 4] 40 235 95 0
[132 5] 47 21 139 0
[132 6] 67 34 152 0
[132 7] 94 13 124 0
[132 8] 104 13 96 0
[132 9] 101 55 89 0
[133 0] 60 237 39 0
[133 1] 144 131 238 0
[133 2] 233 185 26 0
[133 3] 39 224 53 0
[133 4] 59 16 124 0
[133 5] 63 48 165 0
[133 6] 83 46 150 0
[133 7] 106 21 109 0
[133 8] 116 23 96 0
[133 9] 117 75 123 0
[134 0] 68 229 45 0
[134 1] 171 132 254 0
[134 2] 15 204 73 0
[134 3] 83 2 107 0
[134 4] 109 45 149 0
[134 5] 109 59 166 0
[134 6] 116 57 159 0
[134 7] 131 49 143 0
[134 8] 141 60 138 0
[134 9] 142 93 159 0
[135 0] 79 2 103 0
[135 1] 194 176 66 0
[135 2] 43 249 128 0
[135 3] 111 27 134 0
[135 4] 136 48 146 0
[135 5] 134 56 148 0
[135 6] 136 68 151 0
[135 7] 144 96 173 0
[135 8] 152 113 183 0
[135 9] 156 107 177 0
[136 0] 80 47 160 0
[136 1] 201 224 105 0
[136 2] 56 23 122 0
[136 3] 128 45 135 0
[136 4] 146 68 175 0
[136 5] 135 73 158 0
[136 6] 135 75 140 0
[136 7] 143 108 180 0
[136 8] 159 123 194 0
[136 9] 168 108 177 0
[137 0] 46 38 168 0
[137 1] 181 212 117 0
[137 2] 55 19 143 0
[137 3] 143 60 184 0
[137 4] 169 90 237 0
[137 5] 148 97 216 0
[137 6] 143 102 197 0
[137 7] 153 110 213 0
[137 8] 165 106 196 0
[137 9] 167 103 168 0

```

As we can see the results are the same.
