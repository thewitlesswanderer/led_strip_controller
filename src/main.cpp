#include <avr/io.h>
#include <avr/interrupt.h>

#define ENC_PIN0      2
#define ENC_PIN1      3
#define BUTTON_PIN    4
#define LED_PWM_PIN   0
#define RED_LED_PIN   1

#define LED_DEFAULT_BRIGHTNESS 100
#define LED_PWM_INCREMENT      5

#define RED_LED_BRIGHTNESS     240
#define RED_LED_FLASH_INTERVAL 75
#define RED_LED_BLANK_INTERVAL 50 

#define ENC_SAMPLE_INTERVAL 5 //~2ms
#define DEBOUNCE_MASK 0b11000111
#define DOUBLE_PRESS_INTERVAL 100 //~50ms

uint8_t powerState = 0;

uint8_t pwmValue = LED_DEFAULT_BRIGHTNESS;

volatile uint16_t timerOverflowCount = 0;

uint8_t buttonState = 0x00;
uint16_t prevButtonSampleTime = 0;
uint16_t prevButtonPressTime = 0;
uint8_t buttonPressInc = 0;

uint8_t currentEncA = 1;
uint8_t prevEncA = 1;
uint8_t prevEncSampleTime = 0;
int8_t encInc = 0;

//Main system tick
ISR(TIM0_OVF_vect)
{
  timerOverflowCount++; // Increment counter by one
}

//flash pattern for red led's to show we're at min or max.
//NOTE this is blocking all other input until the pattern is finished 
void flashLimits(void) {
  uint16_t flashStartTime = timerOverflowCount;
  OCR0B = 0; //max
  while ((timerOverflowCount - flashStartTime) < RED_LED_FLASH_INTERVAL) {};
  OCR0B = 255; //off
  while ((timerOverflowCount - flashStartTime) < RED_LED_FLASH_INTERVAL + RED_LED_BLANK_INTERVAL) {};
  OCR0B = 0; //max
  while ((timerOverflowCount - flashStartTime) < 2 * RED_LED_FLASH_INTERVAL + RED_LED_BLANK_INTERVAL) {};
  OCR0B = RED_LED_BRIGHTNESS; //back to normal
}

int main(void){

  TCCR0A |= _BV(WGM00) | _BV(COM0B1) |_BV(COM0A1); //Phase Correct PWM
  TCCR0B = _BV(CS01); //PWM frequency = (F_CPU/510) / 8 (~2353Hz)
  TIMSK0 |= _BV(TOIE0); //Interupt on Timer overflow  (~2353Hz)
  sei();

  OCR0A = 0; //LED default to off
  OCR0B = RED_LED_BRIGHTNESS; //Red LED Default Brightness

  DDRB |= _BV(LED_PWM_PIN) | _BV(RED_LED_PIN); //Setup PWM pins as output
  DDRB &= ~(_BV(ENC_PIN0) | _BV(ENC_PIN1) | _BV(BUTTON_PIN)); //Setup Encoder pins as input
  PORTB |= 0 | _BV(ENC_PIN0) | _BV(ENC_PIN1) | _BV(BUTTON_PIN); //Input pullups on encoder pins

  while(1){

    if ((timerOverflowCount - prevButtonSampleTime) >= ENC_SAMPLE_INTERVAL ) { //button sampling interval
      prevButtonSampleTime = timerOverflowCount;    
      buttonState = (buttonState<<1); //shift button history
      buttonState |= ((PINB & _BV(BUTTON_PIN)) >> BUTTON_PIN); //read Button, shift into 8 bit state register of last 8 button readings 
    }

    if ((timerOverflowCount - prevEncSampleTime) >= ENC_SAMPLE_INTERVAL ) { //only read encoder if 2ms from previous 0>1 transistion of Encoder pin 0 to debounce encoder read
    currentEncA = ((PINB & _BV(ENC_PIN0)) >> ENC_PIN0); //get current pin 0 value
      if (currentEncA == 1 && prevEncA == 0) { //we have a 0>1 transition
        prevEncSampleTime = timerOverflowCount; //save time for debouncing
        if (PINB & _BV(ENC_PIN1)) { //direction based on current pin 1 value
          encInc--;
        } else {
          encInc++;
        }
      }
      prevEncA = currentEncA; //save as previous state
    }

    if ((buttonState & DEBOUNCE_MASK) == 0b11000000) { //0b00000111) { //button state is a 11XXX000, button is pressed and stable
      buttonState = 0b00000000; //0b11111111; //button pressed, add hysteresis to history
      buttonPressInc++; //increment button press counter so we can determine single or double press
      if(buttonPressInc >= 2) { //double press occured no need to save time as count is reset
        pwmValue = 255; //max brightness
        OCR0A = pwmValue;
        powerState = 1; //will also turn power state if required
        buttonPressInc = 0; //reset button press counter
      } else {
        prevButtonPressTime = timerOverflowCount; //save time of press
      }
    }

    if (buttonPressInc == 1 && (timerOverflowCount - prevButtonPressTime) >= DOUBLE_PRESS_INTERVAL) { //second press hasn't occured in window. Action single press action
      if (!powerState) { //led's are off, turn them on
        OCR0A = pwmValue;
        powerState = 1;
      } else { //led's are on, turn them off
        OCR0A = 0;
        powerState = 0;
      }
      buttonPressInc = 0; //reset button press counter
    }

    if (powerState && encInc) { //led's are on and encoder moved, change brightness
      if (encInc > 0 && pwmValue == 255) { //already max brightness
        flashLimits();
      } else if (encInc > 0 && pwmValue > (255 - LED_PWM_INCREMENT)) { //just increased to max brightness
        pwmValue = 255;
      } else if (encInc < 0 && pwmValue == 1) { //already min brightness
        flashLimits();
      } else if(encInc < 0 && pwmValue < (1 + LED_PWM_INCREMENT)) { //just decreased to min brightness
        pwmValue = 1;
      } else {
        pwmValue += encInc * LED_PWM_INCREMENT;
      }
      OCR0A = pwmValue; //update pwm duty cycle
      encInc = 0; //reset encoder  increment  
    }


 
  }
  return 1;
}