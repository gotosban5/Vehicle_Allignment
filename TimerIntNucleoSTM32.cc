// Pin D13 is connected to the on-board LED on the Nucleo STM32 board
const int LED_PIN = 13;

volatile int timerCount = 0;

// Interrupt Service Routine for the timer interrupt
void TIM3_IRQHandler() {
  if(TIM3->SR & TIM_SR_UIF) { // Check if interrupt flag is set
    timerCount++; // Increment timer count
    TIM3->SR &= ~TIM_SR_UIF; // Clear interrupt flag
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT); // Set the LED pin as an output

  // Set up timer 3 for interrupt at 1Hz
  RCC->AHB1ENR |= RCC_APB1ENR1_TIM3EN; // Enable TIM3 clock
  TIM3->PSC = 7999; // Set prescaler to 7999
  TIM3->ARR = 999; // Set auto-reload value to 999
  TIM3->DIER |= TIM_DIER_UIE; // Enable update interrupt
  TIM3->CR1 |= TIM_CR1_CEN; // Enable timer

  // Enable timer interrupt in NVIC
  NVIC_EnableIRQ(TIM3_IRQn);
}

void loop() {
  // Check if 1 second has passed
  if(timerCount >= 1) {
    timerCount = 0; // Reset timer count
    blinkLED(); // Call blinkLED() routine
  }
}

// blinkLED() routine to blink the LED
void blinkLED() {
  digitalWrite(LED_PIN, HIGH); // Turn the LED on
  delay(100); // Wait for 100ms
  digitalWrite(LED_PIN, LOW); // Turn the LED off
  delay(100); // Wait for 100ms
}
