#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include "stdbool.h"

static const int MEDIAN_COUNT = 5;
struct median_filter_t {
    float values[5];
    int count;
};
bool insert(struct median_filter_t *medianFilter, float in);
float calc(struct median_filter_t *medianFilter);
void reset(struct median_filter_t *medianFilter);

#endif