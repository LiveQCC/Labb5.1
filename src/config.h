#ifndef CONFIG_H
#define CONFIG_H

// Timing configurations
#define SAMPLING_PERIOD 50    // 20Hz sampling
#define ALGO_PERIOD 1000     // Algorithm runs every 2 seconds
#define BUFF_SIZE 60         // Buffer size with 50% margin

// Algorithm parameters
#define MIN_SD 50           // Minimum standard deviation
#define K 1.5               // Step detection threshold multiplier
#define MIN_INTRA_STEP_TIME 250  // Minimum time between steps
#define STEPS_GOAL 15    // Daily step goal

// GPIO configurations
#define LED_PIN 2
#define BUTTON_PIN 0

#endif
