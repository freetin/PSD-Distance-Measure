#include "filters.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 移动平均滤波器实现 --------------------------------------------------------
MovingAverageFilter* createMovingAverageFilter(uint16_t size) {
    MovingAverageFilter* filter = malloc(sizeof(MovingAverageFilter));
    filter->size = size;
    filter->buffer = malloc(size * sizeof(float));
    memset(filter->buffer, 0, size * sizeof(float));
    filter->index = 0;
    filter->sum = 0.0f;
	filter->count = 0;
    return filter;
}

float AverageFilter( uint16_t * buff,uint16_t size) {  
    if (size == 0) {  
        // 防止除以零，返回0.0f或其他合理的默认值  
        return 0.0f;  
    }  
    uint32_t sum = 0;  
    for (uint16_t i = 0; i < size; i++) {  
        sum += buff[i];  
    }  

    float average = (float)sum / size;  
    return average;  
}

float updateMovingAverage(MovingAverageFilter* filter, float input) {
    if(filter->count < filter->size) {
        // 填充阶段：直接累加，不覆盖旧数据
        filter->sum += input;
        filter->buffer[filter->count] = input;
        filter->count++;
    } else {
        // 滑动阶段：替换旧数据
        filter->sum -= filter->buffer[filter->index];
        filter->sum += input;
        filter->buffer[filter->index] = input;
    }
    
    filter->index = (filter->index + 1) % filter->size;
    
    // 根据当前有效数据量计算平均值
    return filter->sum / (filter->count < filter->size ? filter->count : filter->size);
}


// 中值滤波器实现 ----------------------------------------------------------
int compareFloat(const void* a, const void* b) {
    float arg1 = *(const float*)a;
    float arg2 = *(const float*)b;
    return (arg1 > arg2) - (arg1 < arg2);
}

MedianFilter* createMedianFilter(uint16_t size) {
    MedianFilter* filter = malloc(sizeof(MedianFilter));
    filter->size = size;
    filter->buffer = malloc(size * sizeof(float));
    memset(filter->buffer, 0, size * sizeof(float));
    return filter;
}

float updateMedianFilter(MedianFilter* filter, float input) {
    // 移动数据窗口
    memmove(filter->buffer, filter->buffer + 1, (filter->size - 1) * sizeof(float));
    filter->buffer[filter->size - 1] = input;

    // 创建临时数组并排序
    float temp[filter->size];
    memcpy(temp, filter->buffer, filter->size * sizeof(float));
    qsort(temp, filter->size, sizeof(float), compareFloat);

    // 返回中值
    return temp[filter->size / 2];
}

// 卡尔曼滤波器实现 --------------------------------------------------------
void initKalmanFilter(KalmanFilter* filter, float q, float r, float initial_value) {
    filter->q = q;
    filter->r = r;
    filter->x = initial_value;
    filter->p = 1.0f;
    filter->k = 0.0f;
}

float updateKalmanFilter(KalmanFilter* filter, float measurement) {
    // 预测步骤
    filter->p += filter->q;

    // 更新步骤
    filter->k = filter->p / (filter->p + filter->r);
    filter->x += filter->k * (measurement - filter->x);
    filter->p *= (1 - filter->k);
    
    return filter->x;
}

// 低通滤波器实现 ----------------------------------------------------------
void initLowPassFilter(LowPassFilter* filter, float alpha, float initial_value) {
    filter->alpha = alpha;
    filter->prev_output = initial_value;
}

float updateLowPassFilter(LowPassFilter* filter, float input) {
    float output = filter->alpha * input + (1 - filter->alpha) * filter->prev_output;
    filter->prev_output = output;
    return output;
}

// 高通滤波器实现 ----------------------------------------------------------
void initHighPassFilter(HighPassFilter* filter, float alpha, float initial_value) {
    filter->alpha = alpha;
    filter->prev_input = initial_value;
    filter->prev_output = initial_value;
}

float updateHighPassFilter(HighPassFilter* filter, float input) {
    float output = filter->alpha * (filter->prev_output + input - filter->prev_input);
    filter->prev_input = input;
    filter->prev_output = output;
    return output;
}



VoltRatioFilter* create_volt_ratio_filter(
    uint16_t window_size, 
    float mad_threshold,
    float min_denominator,
    float alpha
) {
    VoltRatioFilter* f = malloc(sizeof(VoltRatioFilter));
    f->window_size = window_size;
    f->mad_threshold = mad_threshold;
    f->min_denominator = min_denominator;
    f->alpha = alpha;
    
    f->adc1_buf = calloc(window_size, sizeof(float));
    f->adc2_buf = calloc(window_size, sizeof(float));
    f->adc3_buf = calloc(window_size, sizeof(float));
    f->index = 0;
    f->filtered_rate = 0.0f;
    
    return f;
}



float update_volt_ratio(VoltRatioFilter* f, float adc1, float adc2, float adc3) {
    // 更新ADC滑动平均缓冲区
    f->adc1_buf[f->index] = adc1;
    f->adc2_buf[f->index] = adc2;
    f->adc3_buf[f->index] = adc3;
    f->index = (f->index + 1) % f->window_size;
    
    // 计算当前平均值
    float adc1_ave = 0, adc2_ave = 0, adc3_ave = 0;
    for(int i=0; i<f->window_size; i++) {
        adc1_ave += f->adc1_buf[i];
        adc2_ave += f->adc2_buf[i];
        adc3_ave += f->adc3_buf[i];
    }
    adc1_ave /= f->window_size;
    adc2_ave /= f->window_size;
    adc3_ave /= f->window_size;
    
    float denominator;
	float raw_rate;
	// 计算原始比率
	if(adc2_ave - adc3_ave)
	{denominator = adc2_ave - adc3_ave;}
	if(adc1_ave - adc3_ave)
    {raw_rate = (adc1_ave - adc3_ave) / denominator;}
    
    // MAD异常值检测
    static float history[5] = {0}; // 保存最近5个样本
    static uint8_t hist_idx = 0;
    
    history[hist_idx] = raw_rate;
    hist_idx = (hist_idx + 1) % 5;
    
    // 计算中值绝对偏差
    float temp[5];
    memcpy(temp, history, sizeof(history));
    qsort(temp, 5, sizeof(float), compare_float);
    float median = temp[2];
    
    float deviations[5];
    for(int i=0; i<5; i++) {
        deviations[i] = fabsf(temp[i] - median);
    }
    qsort(deviations, 5, sizeof(float), compare_float);
    float mad = deviations[2] * 1.4826f;
    
    // 判断是否为异常值
    if(fabsf(raw_rate - median) > f->mad_threshold * mad) {
        raw_rate = median; // 用中值替代异常值
    }
    
    // 低通滤波
    f->filtered_rate = f->alpha * raw_rate + (1 - f->alpha) * f->filtered_rate;
    
    return f->filtered_rate;
}

// 浮点数比较函数
int compare_float(const void* a, const void* b) {
    float arg1 = *(const float*)a;
    float arg2 = *(const float*)b;
    return (arg1 > arg2) - (arg1 < arg2);
}
