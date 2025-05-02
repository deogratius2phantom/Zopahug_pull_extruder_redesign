#include "thermistor.h"
// Function to get temperature using binary search and interpolation
float getTemperature(int adcValue) {
    int low = 0, high = numReadings - 1;

    // Binary search to find the closest ADC range
    while (low <= high) {
        int mid = (low + high) / 2;
        temp_entry_t midEntry;
        memcpy_P(&midEntry, &temptable_1[mid], sizeof(temp_entry_t));

        if (adcValue == midEntry.adc_value) {
            return midEntry.temperature;  // Exact match
        } else if (adcValue < midEntry.adc_value) {
            high = mid - 1;
        } else {
            low = mid + 1;
        }
    }

    // Get the closest lower and upper entries for interpolation
    temp_entry_t lower, upper;
    if (low == 0) {
        memcpy_P(&lower, &temptable_1[0], sizeof(temp_entry_t));
        return lower.temperature;
    }
    if (low >= numReadings) {
        memcpy_P(&upper, &temptable_1[numReadings - 1], sizeof(temp_entry_t));
        return upper.temperature;
    }

    memcpy_P(&lower, &temptable_1[low - 1], sizeof(temp_entry_t));
    memcpy_P(&upper, &temptable_1[low], sizeof(temp_entry_t));

    // Linear interpolation
    float tempDiff = upper.temperature - lower.temperature;
    float adcDiff = upper.adc_value - lower.adc_value;
    return lower.temperature + ((adcValue - lower.adc_value) * tempDiff / adcDiff);
}