#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>

// 移动平均滤波器结构体
typedef struct {
    float *buffer;     // 数据缓冲区
    uint16_t size;     // 滤波器窗口大小
    uint16_t index;    // 当前数据索引
    float sum;         // 当前数据和
	uint16_t count;
} MovingAverageFilter;



// 中值滤波器结构体
typedef struct {
    float *buffer;     // 数据缓冲区  
    uint16_t size;     // 滤波器窗口大小
} MedianFilter;

// 卡尔曼滤波器结构体
typedef struct {
    float q;          // 过程噪声协方差
    float r;          // 测量噪声协方差
    float x;          // 估计值
    float p;          // 估计误差协方差
    float k;          // 卡尔曼增益
} KalmanFilter;

// 低通滤波器结构体
typedef struct {
    float alpha;      // 滤波系数 (0.0-1.0)
    float prev_output;// 前一次输出
} LowPassFilter;

// 高通滤波器结构体
typedef struct {
    float alpha;      // 滤波系数 (0.0-1.0)
    float prev_input;  // 前一次输入
    float prev_output;// 前一次输出
} HighPassFilter;

typedef struct {
    // ADC滑动平均参数
    float *adc1_buf;
    float *adc2_buf;
    float *adc3_buf;
    uint16_t window_size;
    uint16_t index;
    
    // 异常值检测参数
    float mad_threshold;   // MAD阈值倍数（推荐3.0）
    float min_denominator; // 最小允许分母（避免除零）
    
    // 低通滤波参数
    float alpha;           // 低通滤波系数 (0.01-0.3)
    float filtered_rate;   // 最终输出值
} VoltRatioFilter;


// 函数声明
MovingAverageFilter* createMovingAverageFilter(uint16_t size);
float updateMovingAverage(MovingAverageFilter* filter, float input);

MedianFilter* createMedianFilter(uint16_t size);
float updateMedianFilter(MedianFilter* filter, float input);

void initKalmanFilter(KalmanFilter* filter, float q, float r, float initial_value);
float updateKalmanFilter(KalmanFilter* filter, float measurement);

void initLowPassFilter(LowPassFilter* filter, float alpha, float initial_value);
float updateLowPassFilter(LowPassFilter* filter, float input);

void initHighPassFilter(HighPassFilter* filter, float alpha, float initial_value);
float updateHighPassFilter(HighPassFilter* filter, float input);


float AverageFilter(uint16_t * buff,uint16_t size);
int compare_float(const void* a, const void* b);
float update_volt_ratio(VoltRatioFilter* f, float adc1, float adc2, float adc3);
VoltRatioFilter* create_volt_ratio_filter(
    uint16_t window_size, 
    float mad_threshold,
    float min_denominator,
    float alpha
);
#endif // FILTERS_H
