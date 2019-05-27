#include "ebike.h"

uint32_t median_3(uint32_t a, uint32_t b, uint32_t c) {
  uint32_t output;
  if ((a <= b) && (a <= c)) {
    output = (b <= c) ? b : c;
  } else if ((b <= a) && (b <= c)) {
    output = (a <= c) ? a : c;
  } else {
    output = (a <= b) ? a : b;
  }
  return output;
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

void exp_filter(uint32_t *value, uint32_t *output, int filter_alpha) {
    *output = ((*output << filter_alpha)+(*value - *output)) >> filter_alpha;
}

uint32_t linear_cutoff(uint32_t x, uint32_t y, uint32_t x_max, uint32_t x_max_start, uint32_t y_max) {
  // y_min is always zero
  // x_max_start is assumed to be < x_max

  if(x < x_max_start) {
    // y is below the cutoff range
    return min(y, y_max);
  }
  if(x < x_max) {
    // y is in the cutoff range
    uint32_t y_limit = (y_max * (x_max - x)) / (x_max - x_max_start); // cutoff slope times distance from x_max
    return min(y, y_limit);
  }
  else {
    // y is above the cutoff range
    return 0;
  }
}