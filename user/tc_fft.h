/*
 * @Author: Pomin
 * @Date: 2021-11-04 14:48:10
 * @Github: https://github.com/POMIN-163
 * @LastEditTime: 2021-11-09 11:15:27
 * @Description: 本 FFT算法代码无权开源, 请联系 QQ 1637766030
 */
#if !defined(TC_FFT)
#define TC_FFT
#include "tc_common.h"
#include "tc_adc.h"

#define Sample_Num _N
#define PI 3.1415926

struct FFT_DATA {
    float  dataR;          // 6:0   Max number of conversions
    float  dataI;             // 15:7  reserved
};
struct WN {
    float  sin;          // 6:   Max number of conversions
    float  cos;             // 15:7  reserved
};
struct Result{
	uint16_t freq;
	float  angle; 			//FFT结果的相位角
	float  U;
	float  P;
};
extern volatile struct FFT_DATA FFTDATA[Sample_Num];
extern volatile struct Result result[Sample_Num/2],lastresult[Sample_Num/2];
extern void InitForFFT(void);
extern void MakeWave(void);
extern void FFT_N(void);
void test_fft(void);

void fft_init(void);
uint32_t fft_read_useless(void);
uint32_t fft_read(void);
void fft_write(uint32_t value);

#endif // TC_FFT
