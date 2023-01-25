#include "median_filter.h"
#include "stdlib.h"

void reset(struct median_filter_t *medianFilter) {
    medianFilter->count = 0;
    //medianFilter->values = {};
}

bool insert(struct median_filter_t *medianFilter, float in) {
    if (medianFilter->count == MEDIAN_COUNT) {
        return true;
    }
    // insert new value
    medianFilter->values[medianFilter->count] = in;
    medianFilter->count++;
    // check if there are enough values to calc
    return medianFilter->count == MEDIAN_COUNT;
}

// used for sorting
int compare (const void * a, const void * b) {
    float comparison = ( *(float*)a - *(float*)b );
    if (comparison > 0.0) return -1;
    if (comparison == 0.0) return 0;
    if (comparison < 0.0) return 1;
    return 0;
}

float calc(struct median_filter_t *medianFilter) {
    qsort(medianFilter->values, medianFilter->count, sizeof(float), compare);
    float out = medianFilter->values[MEDIAN_COUNT / 2];
    reset(medianFilter);
    return out;
}