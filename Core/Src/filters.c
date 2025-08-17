#include "filters.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// �ƶ�ƽ���˲���ʵ�� --------------------------------------------------------
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
        // ��ֹ�����㣬����0.0f�����������Ĭ��ֵ  
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
        // ���׶Σ�ֱ���ۼӣ������Ǿ�����
        filter->sum += input;
        filter->buffer[filter->count] = input;
        filter->count++;
    } else {
        // �����׶Σ��滻������
        filter->sum -= filter->buffer[filter->index];
        filter->sum += input;
        filter->buffer[filter->index] = input;
    }
    
    filter->index = (filter->index + 1) % filter->size;
    
    // ���ݵ�ǰ��Ч����������ƽ��ֵ
    return filter->sum / (filter->count < filter->size ? filter->count : filter->size);
}


// ��ֵ�˲���ʵ�� ----------------------------------------------------------
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
    // �ƶ����ݴ���
    memmove(filter->buffer, filter->buffer + 1, (filter->size - 1) * sizeof(float));
    filter->buffer[filter->size - 1] = input;

    // ������ʱ���鲢����
    float temp[filter->size];
    memcpy(temp, filter->buffer, filter->size * sizeof(float));
    qsort(temp, filter->size, sizeof(float), compareFloat);

    // ������ֵ
    return temp[filter->size / 2];
}

// �������˲���ʵ�� --------------------------------------------------------
void initKalmanFilter(KalmanFilter* filter, float q, float r, float initial_value) {
    filter->q = q;
    filter->r = r;
    filter->x = initial_value;
    filter->p = 1.0f;
    filter->k = 0.0f;
}

float updateKalmanFilter(KalmanFilter* filter, float measurement) {
    // Ԥ�ⲽ��
    filter->p += filter->q;

    // ���²���
    filter->k = filter->p / (filter->p + filter->r);
    filter->x += filter->k * (measurement - filter->x);
    filter->p *= (1 - filter->k);
    
    return filter->x;
}

// ��ͨ�˲���ʵ�� ----------------------------------------------------------
void initLowPassFilter(LowPassFilter* filter, float alpha, float initial_value) {
    filter->alpha = alpha;
    filter->prev_output = initial_value;
}

float updateLowPassFilter(LowPassFilter* filter, float input) {
    float output = filter->alpha * input + (1 - filter->alpha) * filter->prev_output;
    filter->prev_output = output;
    return output;
}

// ��ͨ�˲���ʵ�� ----------------------------------------------------------
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
    // ����ADC����ƽ��������
    f->adc1_buf[f->index] = adc1;
    f->adc2_buf[f->index] = adc2;
    f->adc3_buf[f->index] = adc3;
    f->index = (f->index + 1) % f->window_size;
    
    // ���㵱ǰƽ��ֵ
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
	// ����ԭʼ����
	if(adc2_ave - adc3_ave)
	{denominator = adc2_ave - adc3_ave;}
	if(adc1_ave - adc3_ave)
    {raw_rate = (adc1_ave - adc3_ave) / denominator;}
    
    // MAD�쳣ֵ���
    static float history[5] = {0}; // �������5������
    static uint8_t hist_idx = 0;
    
    history[hist_idx] = raw_rate;
    hist_idx = (hist_idx + 1) % 5;
    
    // ������ֵ����ƫ��
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
    
    // �ж��Ƿ�Ϊ�쳣ֵ
    if(fabsf(raw_rate - median) > f->mad_threshold * mad) {
        raw_rate = median; // ����ֵ����쳣ֵ
    }
    
    // ��ͨ�˲�
    f->filtered_rate = f->alpha * raw_rate + (1 - f->alpha) * f->filtered_rate;
    
    return f->filtered_rate;
}

// �������ȽϺ���
int compare_float(const void* a, const void* b) {
    float arg1 = *(const float*)a;
    float arg2 = *(const float*)b;
    return (arg1 > arg2) - (arg1 < arg2);
}
