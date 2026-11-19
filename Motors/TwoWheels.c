#include <msp430.h>
#include <stdint.h>

#define SLAVE_ADDR 0x2A

// ---------- Servo settings (microseconds) ----------
#define SERVO_PERIOD_TICKS 20971 // 20 ms at SMCLK = 1.048576 MHz
#define STEER_CENTER_US 1500     // SG92R straight ahead (trim if needed)
#define STEER_RANGE_US 500   // lower to 400 if the SG92R buzzes at full lock
#define FS90R_STOP_US_L 1500 // trim so each wheel is truly still (1450-1550)
#define FS90R_STOP_US_R 1500
#define FS90R_RANGE_US 300 // +/-300 us => full speed each way

// ---------- Tilt mapping (BMA250, 10-bit, ~256 counts per g) ----------
#define FORWARD_X_SIGN 1 // flip if forward tilt drives backward
#define RIGHT_Y_SIGN 1 // flip if right tilt turns left
#define ACCEL_DEADZONE 40   // drive: about 0.15 g
#define ACCEL_FULLSCALE 180 // drive: about 0.7 g = 100%
#define STEER_DEADZONE 15   // steering: about 0.06 g
#define STEER_FULLSCALE 70   // steering: about 0.27 g = full lock

// Steering ramp: each packet moves 1/STEER_SMOOTH_DIV of the remaining
// distance. 1 = no smoothing (fastest), 2 = quick, 4 = smooth, 6 = very smooth.
#define STEER_SMOOTH_DIV 2

#define FAILSAFE_TICKS 25 // 25 x 20 ms = 0.5 s without input => stop

volatile int16_t accelX = 0;
volatile int16_t accelY = 0;
volatile int16_t accelZ = 0;
volatile uint16_t packetCount = 0;
volatile uint16_t startCount = 0;
volatile uint16_t tick20ms = 0;
volatile uint16_t lastPacketTick = 0;

uint8_t rxBuffer[6] = {0, 0, 0, 0, 0, 0};
volatile uint8_t byteCounter = 0;

int16_t steerNow = 0; // smoothed steering value, main loop only

// ---------- UART (USCI_A1 -> LaunchPad USB backchannel, 9600 baud) ----------
static void uartInit(void) {
  P4SEL |= BIT4 | BIT5; // P4.4 = TX, P4.5 = RX
  UCA1CTL1 |= UCSWRST;
  UCA1CTL1 |= UCSSEL__SMCLK;
  UCA1BR0 = 109; // 1048576 / 9600
  UCA1BR1 = 0;
  UCA1MCTL = UCBRS_2;
  UCA1CTL1 &= ~UCSWRST;
  UCA1IE |= UCRXIE; // receive interrupt for key control
}

static void uartChar(char c) {
  while (!(UCA1IFG & UCTXIFG))
    ;
  UCA1TXBUF = c;
}

static void uartStr(const char *s) {
  while (*s)
    uartChar(*s++);
}

static void uartInt(int16_t v) {
  char buf[7];
  uint8_t i = 0;
  uint16_t u;
  if (v < 0) {
    uartChar('-');
    u = (uint16_t)(-(int32_t)v);
  } else {
    u = (uint16_t)v;
  }
  if (u == 0) {
    uartChar('0');
    return;
  }
  while (u > 0) {
    buf[i++] = '0' + (u % 10);
    u /= 10;
  }
  while (i > 0)
    uartChar(buf[--i]);
}

// ---------- Servo PWM (Timer_A0) ----------
// Left FS90R  -> P1.5 (TA0.4)
// Right FS90R -> P1.3 (TA0.2)
// SG92R steer -> P1.4 (TA0.3)
static void setPulseUs(volatile unsigned int *ccr, uint16_t us) {
  *ccr = (uint16_t)(((uint32_t)us * 1049UL) / 1000UL); // 1 us ~ 1.049 ticks
}

static int16_t clamp(int16_t v, int16_t lo, int16_t hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// steer: -100 (left) .. +100 (right)
static void setSteering(int16_t steer) {
  steer = clamp(steer, -100, 100);
  setPulseUs(&TA0CCR3, STEER_CENTER_US + (int32_t)steer * STEER_RANGE_US / 100);
}

// speed: -100 (reverse) .. +100 (forward), both wheels equal.
// The two FS90Rs face opposite directions, so one is mirrored.
static void setDrive(int16_t speed) {
  speed = clamp(speed, -100, 100);
  int16_t d = (int32_t)speed * FS90R_RANGE_US / 100;
  setPulseUs(&TA0CCR4, FS90R_STOP_US_L + d); // left  (P1.5)
  setPulseUs(&TA0CCR2, FS90R_STOP_US_R - d); // right (P1.3)
}

// Drive with differential assist: the inside wheel slows as you steer.
// steer: -100 (left) .. +100 (right)
// If the car turns the wrong way, swap the 'r' and 'l' lines.
static void setDriveMix(int16_t speed, int16_t steer) {
  speed = clamp(speed, -100, 100);
  steer = clamp(steer, -100, 100);
  int16_t l = speed, r = speed;
  if (speed == 0) { // Sideways tilt alone drives only the outside wheel.
    l = steer > 0 ? steer : 0;
    r = steer < 0 ? -steer : 0;
  } else if (steer > 0)
    r = (int32_t)speed * (100 - steer) / 100; // turning right: slow right wheel
  else if (steer < 0)
    l = (int32_t)speed * (100 + steer) / 100; // turning left: slow left wheel
  int16_t dl = (int32_t)l * FS90R_RANGE_US / 100;
  int16_t dr = (int32_t)r * FS90R_RANGE_US / 100;
  setPulseUs(&TA0CCR4, FS90R_STOP_US_L + dl); // left  (P1.5)
  setPulseUs(&TA0CCR2, FS90R_STOP_US_R - dr); // right (P1.3), mirrored
}

static void servoInit(void) {
  P1DIR |= BIT3 | BIT4 | BIT5;
  P1SEL |= BIT3 | BIT4 | BIT5; // TA0.2, TA0.3, TA0.4

  TA0CCR0 = SERVO_PERIOD_TICKS - 1;
  TA0CCTL0 = CCIE; // 20 ms tick for the failsafe timer
  TA0CCTL2 = OUTMOD_7;
  TA0CCTL3 = OUTMOD_7;
  TA0CCTL4 = OUTMOD_7;

  setDrive(0);
  setSteering(0);

  TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;
}

// Raw accel -> -100..+100 with deadzone
static int16_t accelToPercent(int16_t a, int16_t dead, int16_t full) {
  a = clamp(a, -full, full);
  if (a >= -dead && a <= dead)
    return 0;
  int16_t beyondDeadzone = a > 0 ? a - dead : a + dead;
  return (int32_t)beyondDeadzone * 100 / (full - dead);
}

int main(void) {
  WDTCTL = WDTPW | WDTHOLD;

  P1DIR |= BIT0;
  P1OUT &= ~BIT0; // red LED: I2C start
  P4DIR |= BIT7;
  P4OUT &= ~BIT7; // green LED: full packet

  uartInit();
  servoInit();

  // I2C pins: P3.0 = SDA, P3.1 = SCL
  P3SEL |= BIT0 | BIT1;

  UCB0CTL1 |= UCSWRST;
  UCB0CTL0 = UCMODE_3 | UCSYNC;
  UCB0I2COA = SLAVE_ADDR;
  UCB0CTL1 &= ~UCSWRST;
  UCB0IE |= UCRXIE + UCSTTIE + UCSTPIE;

  __bis_SR_register(GIE);

  uartStr("MSP430 ready\r\n");

  uint16_t lastPrinted = 0;
  uint16_t lastPrintCount = 0;
  while (1) {
    if (packetCount != lastPrinted) {
      lastPrinted = packetCount;

      __disable_interrupt();
      int16_t x = accelX, y = accelY, z = accelZ;
      uint16_t n = packetCount;
      __enable_interrupt();

      int16_t throttle = FORWARD_X_SIGN * accelToPercent(x, ACCEL_DEADZONE, ACCEL_FULLSCALE);
      int16_t target = RIGHT_Y_SIGN * accelToPercent(y, STEER_DEADZONE, STEER_FULLSCALE);

      // Ease the steering servo toward the target (always reaches it exactly)
      steerNow +=
          (target - steerNow + (target > steerNow ? 1 : -1)) / STEER_SMOOTH_DIV;

      setDriveMix(throttle, target); // inside wheel slows while turning
      setSteering(steerNow);

      // Print about every 5 packets
      if ((uint16_t)(n - lastPrintCount) >= 5) {
        lastPrintCount = n;
        uartStr("X=");
        uartInt(x);
        uartStr(" Y=");
        uartInt(y);
        uartStr(" Z=");
        uartInt(z);
        uartStr("\r\n");
      }
    }

    // Failsafe: no input for 0.5 s => stop and straighten
    if ((uint16_t)(tick20ms - lastPacketTick) > FAILSAFE_TICKS) {
      steerNow = 0;
      setDrive(0);
      setSteering(0);
    }
  }
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void) { tick20ms++; }

#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
  switch (__even_in_range(UCA1IV, 4)) {
  case 2: { // RX byte (W/S/A/D/X keys, backup control)
    char c = UCA1RXBUF;
    switch (c) {
    case 'w':
      setDrive(70);
      setSteering(0);
      break;
    case 's':
      setDrive(-70);
      setSteering(0);
      break;
    case 'a':
      setDriveMix(0, -70);
      setSteering(-70);
      break;
    case 'd':
      setDriveMix(0, 70);
      setSteering(70);
      break;
    case 'x':
      setDrive(0);
      setSteering(0);
      break;
    }
    lastPacketTick = tick20ms;
  } break;
  default:
    break;
  }
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void) {
  switch (__even_in_range(UCB0IV, 12)) {

  case 0x06: // START
    byteCounter = 0;
    startCount++;
    P1OUT ^= BIT0;
    break;

  case 0x08: // STOP
    byteCounter = 0;
    break;

  case 0x0A: // RX byte
  {
    uint8_t rxByte = UCB0RXBUF;

    if (byteCounter < 6) {
      rxBuffer[byteCounter++] = rxByte;

      if (byteCounter == 6) {
        accelX = (int16_t)(rxBuffer[0] | (rxBuffer[1] << 8));
        accelY = (int16_t)(rxBuffer[2] | (rxBuffer[3] << 8));
        accelZ = (int16_t)(rxBuffer[4] | (rxBuffer[5] << 8));

        packetCount++;
        lastPacketTick = tick20ms;
        P4OUT ^= BIT7;
        byteCounter = 0;
      }
    } else {
      byteCounter = 0;
    }
  } break;

  default:
    break;
  }
}
