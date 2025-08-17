#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>

// �ƶ�ƽ���˲����ṹ��
typedef struct {
    float *buffer;     // ���ݻ�����
    uint16_t size;     // �˲������ڴ�С
    uint16_t index;    // ��ǰ��������
    float sum;         // ��ǰ���ݺ�
	uint16_t count;
} MovingAverageFilter;



// ��ֵ�˲����ṹ��
typedef struct {
    float *buffer;     // ���ݻ�����  
    uint16_t size;     // �˲������ڴ�С
} MedianFilter;

// �������˲����ṹ��
typedef struct {
    float q;          // ��������Э����
    float r;          // ��������Э����
    float x;          // ����ֵ
    float p;          // �������Э����
    float k;          // ����������
} KalmanFilter;

// ��ͨ�˲����ṹ��
typedef struct {
    float alpha;      // �˲�ϵ�� (0.0-1.0)
    float prev_output;// ǰһ�����
} LowPassFilter;

// ��ͨ�˲����ṹ��
typedef struct {
    float alpha;      // �˲�ϵ�� (0.0-1.0)
    float prev_input;  // ǰһ������
    float prev_output;// ǰһ�����
} HighPassFilter;

typedef struct {
    // ADC����ƽ������
    float *adc1_buf;
    float *adc2_buf;
    float *adc3_buf;
    uint16_t window_size;
    uint16_t index;
    
    // �쳣ֵ������
    float mad_threshold;   // MAD��ֵ�������Ƽ�3.0��
    float min_denominator; // ��С�����ĸ��������㣩
    
    // ��ͨ�˲�����
    float alpha;           // ��ͨ�˲�ϵ�� (0.01-0.3)
    float filtered_rate;   // �������ֵ
} VoltRatioFilter;


// ��������
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
