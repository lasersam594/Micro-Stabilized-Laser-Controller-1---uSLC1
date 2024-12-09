// Controller firmware for Sam's Micro Stabilized HeNe Laser 1.  Compatible with SP 117/A and MG STP-901 laser heads.
// V01 - Basic implementation to test real time clock, HeNe P and S mode input and LEDs, and heater control.
// v02 - Automagic locking at appriximately 50 percent heater power.
// V03 - Integral and AverageError
// V04 - Moved control loop to interrupt routine in preparation for GUI monitor
// V05 - Comm link: Pmode, Smode, Mode Error, HeaterValue, State, TotalTime
// V06 - First uSLC1 with GUI USB communications
// V07 - Added warmup state
// V08 - State select from GUI
// V09 - Duration and locktimout
// V11 - Variables for PGain and IGain
// V15 - Changed clock to 0.1 s increments
// V22 - Sending state changes and parameters implemented
// V23 - Organized parameters into control blocks but not implemented
// V24 - Convert to DCB, LCB, UCB array structure
// V29 - Reduced comm baud rate to 57600 due to glitches  u
// V30 - Disabled interrupts within FIFO code (write and read)
// V33 - Red/blue side lock select
// V35 - Added offset and intensity
// V37 - Read firmware LCB at GUI startup
// V41 - EEPROM save/recall locking parameters
// V51 - Changed pin assignments to support REF input for GUI frequency counter
// V53 - Test with PCB, added Lamp Test ;-)
// V54 - Added averaging to Pmode, Smode, and Heater Drive samples sent to GUI
// V56 - Fixed Relock Count and Hangout Duration parameter save/load errors
// V57 - Future enhacements.
// V58 - Fixed Reset bug if both modes are too high or too low.  V58 is standard uSLC1.

// Implementation: Preheat and track Pmode and Smode finding min and max; Wait until period < 20 seconds; Locking.
// Real time clock provided using PWM on pin D3 to generate interrupt which increments TimeAccumulator.

int FirmwareVersion = 158;
int EEPROMFormatValid = 143; // Magic number in UCB (first compatible FW version) to confirm valid EEPROM/UCB format

#include <EEPROM.h>

// Digial inputs
#define REFInput 5       // D5 REF input signal for frequency counter
#define Interlock 17     // D17 Interlock (LOW to enable, future)
#define FreqIntSelect 19 // D19 Frequency/Intensity- locking selection (LOW for I, future)

// Analog inputs
#define PmodeInput 0     // A0 Green mode
#define SmodeInput 1     // A1 Orange mode
#define ExtModulation 6  // A6 External modulation (future)
#define ExtIntensity 7   // A7 External intensity setting (future)

// Digial outputs
#define StateBit0LED 7   // D7 State LEDs: Idle: 0, Startup: 1, Warmup: 2, Locking: 3, Locked: 4, Error: 7.
#define StateBit1LED 8   // D8
#define StateBit2LED 9   // D9
#define LockedLED 10     // D10 Lock status for system without any other indicators or GUI
#define ErrorLED 12      // D11 Error status
#define AtmegaLED 13     // D12 On-board LED
#define LaserEnable 18   // D13 Laser Enable (HIGH to turn on laser, future)

// PWM outputs
#define PmodeLED 3      // D3 Green LED tracks Pmode input - offset
#define HeaterDrive 6   // D6 Transistor drive signal (heater and LED)
#define SmodeLED 11     // D11 Orange LED tracks Smode input - offset

// Clock info
#define RTClockFrequency    977
#define RTClockFrequencyD10  98

// Display values to send

struct dataPoint                // stucture for one data packet; The FIFO queue is a queue of these structs
  {
    int displayPmode;           // P-Mode 0-1023
    int displaySmode;           // P-Mode 0-1023
    int displayHeater;          // Heater drive 0-255
    long displaySequenceNumber; // Sequence number
    int displayLowSpeedCode;    // Select type of low speed data
    long displayLowSpeedData;   // Low speed data
  };

#define FIFO_SIZE 25 // size of circular buffer

struct dataPoint dataPoint[FIFO_SIZE]; // circular buffer of data to be sent to USB

volatile int head = 0;  // beginning of circular buffer; empty when both head and tail are the same value
volatile int tail = 0;  // end of circular buffer; full when tail is one slot before head we are full

static char outbuffer[300];  // Space for the routine to convert numbers to text to send
static char inbuffer[50];    // Space text received from GUI

static int bufferOverflowCounter = 0;

static int LowSpeedCode = 0;
static long LowSpeedData = 0;
static long SequenceNumber = 0;
static int LowSpeedCodeSelect = 0;

// volatile long StateParams[2][32];   // State-specific parameters  for defaults and user

volatile int State = 0;               // 0: Startup, 1: Warmup, 2: Heating, 3: Locking, 4: Cooling, 5: Locked, 6: Hangout, 7: Error.

static int PreviousPmode = 0;
int Pmode = 0;                        // P-polarized mode signal
long PmodeLong = 0;                   // 32 bit Pmode
static int PmodeAverage = 0;          // Average over 16 samples to send

static int PreviousSmode = 0;
int Smode = 0;                        // S-polarized mode signal
long SmodeLong = 0;                   // 32 bit Smode
static int SmodeAverage = 0;          // Average over 16 samples to send

// LCB - Lock Control Block

volatile int LCB[32]; // Active Locking Control Block
volatile int DCB[32]; // Default Locking Control Block - These do not change
volatile int UCB[32]; // User Locking Control Block - These will start off being the same as DCB

// LCB/DCB/UCB format (Durations are in 10ths of seconds; Heater Drive values will be 0-255 for 0-100%; Voltages are 0-1023 for 0-5V; All else are integers

#define EEPROMFormat 0
#define PmodeMin 1
#define SmodeMin 2
#define PmodeMax 3
#define SmodeMax 4
#define ModePeriod 5
#define ControlRegister 6
#define LockingTolerance 7
#define ReLockCount 8
#define ProportionalGain 9
#define IntegralGain 10
#define DifferentialGain 11
#define Duration0 12
#define Duration1 13
#define Duration2 14
#define Duration3 15
#define Duration4 16
#define Duration5 17
#define Duration6 18
#define Duration7 19
#define HeaterValue0 20
#define HeaterValue1 21
#define HeaterValue2 22
#define HeaterValue3 23
#define HeaterValue4 24
#define HeaterValue5 25
#define HeaterValue6 26
#define HeaterValue7 27
#define Spare1 28
#define Offset 29
#define Intensity 30
#define CheckSum 31

volatile int Duration = 120;
volatile int HeaterValue = 0;          // Heater value for state 6.
volatile int HeaterValueAverage = 0;   // Send average of 16 samples.
volatile int ReLockCounter = 1;        // Number of times to fine tune lock temperature

volatile long IntegralAccum = 0;       // Integrator "capacitor"
static int IntegralTerm = 0;
static int NoLaserLight = 0;           // Set to 1 if laser does not come on within 60 seconds.
static int LoopDifference = 0;         // Current Pmode-Smode
static int PreviousLoopDifference = 0; // Average Current Pmode-Smode
static int PreviousAverageLoopDifference = 0; // Previous Average Pmode-Smode
static int AverageLoopDifference = 0;  // Moving average of Mode Error
static int LoopDifferenceStamp = 0;    // Time at which Mode Error is sampled

volatile long LockTime = 0;            // Time when laser goes to state 5 in 0.1 s increments
volatile long RTClockTicks1 = 0;       // RTC LSB counter 1
volatile long TotalTime = 0;           // Total time in 0.1 s increments from reset
volatile long TotalTotalTime = 0;      // Total time in RT clock ticks from reset
volatile long CurrentTime = 0;         // Saved current time in seconds 
volatile int TotalTimeTickFlag = 0;    // Set to 1 if TotalTime crossed a 0.1 s boundary
volatile long TimeStamp = 0;           // Saved current time in seconds
volatile int REFCount = 0;             // Number of REF clocks between interrupts

int i = 0;
int j = 0;
int k = 0;

int Temp1 = 0;

volatile int Command = 0;
volatile int Param1 = 0;
volatile int Param2 = 0;
volatile int Param3 = 0;

volatile int Value0 = 0;
volatile int Value1 = 0;
volatile int Value2 = 0;
volatile int Value3 = 0;

volatile int nda;
volatile bool NewCommand = false;
volatile bool ReadbackLCB = false;
static int ReadbackIndex = -1;
static int UCBValid = 0;

void setup()
{
// Specify pin functions
 
  pinMode(PmodeLED, OUTPUT);            // P-mode red LED pin 3
  pinMode(FreqIntSelect, INPUT_PULLUP); // Frequency-/Intensity lock select pin 4
  pinMode(REFInput, INPUT_PULLUP);      // REF frequency signal pin 5
  pinMode(HeaterDrive, OUTPUT);         // Heater, yellow LED pin 6
  pinMode(StateBit0LED, OUTPUT);        // Startup mode pin 7
  pinMode(StateBit1LED, OUTPUT);        // Startup mode pin 8
  pinMode(StateBit2LED, OUTPUT);        // Startup mode pin 9
  pinMode(LockedLED, OUTPUT);           // Locked LED pin 10
  pinMode(SmodeLED, OUTPUT);            // S-mode green LED pin 11
  pinMode(ErrorLED, OUTPUT);            // Error LED pin 12
  pinMode(AtmegaLED, OUTPUT);           // On board LED pin 13

// Initialize pins - Lamp Test for 2 seconds
  digitalWrite(StateBit0LED, HIGH);
  digitalWrite(StateBit1LED, HIGH);
  digitalWrite(StateBit2LED, HIGH);
  digitalWrite(PmodeLED, HIGH);
  digitalWrite(SmodeLED, HIGH);
  digitalWrite(LockedLED, HIGH);
  digitalWrite(ErrorLED, HIGH);
  digitalWrite(HeaterDrive, HIGH);
  digitalWrite(AtmegaLED, HIGH);

  delay(2000); // Interrupts must not be disabled for delay to work properly

  digitalWrite(StateBit0LED, LOW);
  digitalWrite(StateBit1LED, LOW);
  digitalWrite(StateBit2LED, LOW);
  digitalWrite(PmodeLED, LOW);
  digitalWrite(SmodeLED, LOW);
  digitalWrite(LockedLED, LOW);
  digitalWrite(ErrorLED, LOW);
  digitalWrite(HeaterDrive, LOW);
  digitalWrite(AtmegaLED, LOW);

  noInterrupts(); // Disable interrupts while messing with timers

// Initialize Timer0 PWM B for RTC Interrupt
  TCNT0  = 0;                                      // Value
  OCR0B = 128;                                     // Compare match register
  TCCR0B &= !(_BV(CS02) | _BV(CS01) | _BV(CS00));  // Clear CS bits (0b0111)
  TCCR0B |= _BV(CS01) | _BV(CS00);                 // Set prescaler to 64 (0b0011)
  TIMSK0 |= _BV(OCIE0B);                           // Enable timer compare interrupt (bit 0b0100)

// Initialize Timer1 for REF frequency counter (refer atmega168.pdf chapter 16-bit counter1)
  TCCR1A = 0;  // reset timer/counter1 control register A
  TCCR1B = 0;  // reset timer/counter1 control register A
  TCNT1=0;     // counter value = 0
               // set timer/counter1 hardware as counter, counts events on
               // pin T1 ( arduino pin 5), normal mode, wgm10 .. wgm13 = 0

  TCCR1B |= (_BV(CS10) | _BV(CS11) | _BV(CS12)); // Clock select external rising

// Initialize DCB with default values

DCB[0] = 1;
DCB[PmodeMin] = 0;
DCB[SmodeMin] = 0;
DCB[PmodeMax] = 1023;
DCB[SmodeMax] = 1023;
DCB[ModePeriod] = 150;
DCB[LoopDifference] = 0;
DCB[LockingTolerance] = 8;
DCB[ReLockCount] = 1;
DCB[ProportionalGain] = 10;
DCB[IntegralGain] = 10;
DCB[DifferentialGain] = 0;
DCB[Duration0] = 1200;
DCB[Duration1] = 18000;
DCB[Duration2] = 300;
DCB[Duration3] = 100;
DCB[Duration4] = 150;
DCB[Duration5] = 3000;
DCB[Duration6] = 3600;
DCB[Duration7] = 1200;
DCB[HeaterValue0] = 0;
DCB[HeaterValue1] = 255;
DCB[HeaterValue2] = 255;
DCB[HeaterValue3] = 127;
DCB[HeaterValue4] = 0;
DCB[HeaterValue5] = 127;
DCB[HeaterValue6] = 0;
DCB[HeaterValue7] = 0;
DCB[Spare1] = 0;
DCB[Offset] = 0;
DCB[Intensity] = 511;
DCB[CheckSum] = 0;

// Copy UCB or DCB to LCB

UCBValid = 0;
EEPROM.get(0, i);
if (i == EEPROMFormatValid) // This value indicates data is valid (First firmware rev this format)
  {
    for (i = 0;i < 32; i++)
      {
        EEPROM.get(i<<1, LCB[i]); // Address in bytes for EEPROM.get regardless of datatype for value
      }
    UCBValid = 1;
  }
else
  { 
    DCB[0] = EEPROMFormatValid;
    for (i = 0;i < 32; i++)                              // Load Default Locking Parameters
      {
        LCB[i] = DCB[i];                                 // LCD loaded from Default Control Block
        EEPROM.update(i<<1, LCB[i] & 0xff);              // Low byte of LCB to EEPROM UCB
        EEPROM.update((i<<1) + 1, (LCB[i] >> 8) & 0xff); // High byte of LCB to EEPROM UCB
      }
  }
  
// Initial values

  State = 0;
  Duration = LCB[Duration0];
  HeaterValue = LCB[HeaterValue0];

// Open USB serial port

  Serial.begin(57600); // Buad rate (probably doesn't matter for USB)
  
  interrupts();
  }

  ISR(TIMER0_COMPB_vect) // Timer compare ISR
   {
     if ((TotalTotalTime & 0xf) == 0)
       {
         REFCount = TCNT1; // Read Timer1
         TCNT1 = 0;        // Reset Timer1
       }
       
     RTClockTicks1++;
     TotalTotalTime++;
     
     if (RTClockTicks1 >= RTClockFrequencyD10)
       {
         TotalTime++;
         TotalTimeTickFlag = 1;
         RTClockTicks1 = 0;
         digitalWrite(AtmegaLED, !digitalRead(AtmegaLED)); // Heartbeat 5 Hz
       }

    interrupts(); // Enable interrupts to permit UART to function

 // Acquire and monitor polarized modes, and compute Mode Error

    PreviousPmode = Pmode;
    PreviousSmode = Smode;
    PreviousLoopDifference = LoopDifference;
    PreviousAverageLoopDifference = LoopDifference;

// Mode gain and offset correction

    PmodeLong = analogRead(PmodeInput) - LCB[PmodeMin];
    PmodeLong = (PmodeLong * 1024) / (LCB[PmodeMax] - LCB[PmodeMin]);

    SmodeLong = analogRead(SmodeInput) - LCB[SmodeMin];
    SmodeLong = (SmodeLong * 1024) / (LCB[SmodeMax] - LCB[SmodeMin]);

    Pmode = PmodeLong;
    Smode = SmodeLong;
    
    if (Pmode < 0) Pmode = 0;
    if (Smode < 0) Smode = 0;

    if (Pmode > 1023) Pmode = 1023;
    if (Smode > 1023) Smode = 1023;

    PmodeAverage += Pmode;
    SmodeAverage += Smode;
    
    if ((LCB[ControlRegister] & 0x2) == 0x2) LoopDifference = (Pmode - LCB[Intensity]); 
    else if ((LCB[ControlRegister] & 0x4) == 0x4) LoopDifference = (Smode - LCB[Intensity]); 
    else LoopDifference = (Pmode - Smode) + LCB[Offset];
    
    if (LoopDifference < -1023) LoopDifference = -1023;
    if (LoopDifference >  1023) LoopDifference =  1023;

// Loop Difference LED update

    if (LoopDifference > 0) analogWrite(PmodeLED,(LoopDifference/4)); // Green LED
    else analogWrite(PmodeLED,(0));

    if (LoopDifference < 0) analogWrite(SmodeLED,(-LoopDifference/4)); // Red LED
    else analogWrite(SmodeLED,(0));

// Mode LED update - substitute for the ones below if mode instead of loop difference display is preferred

//    analogWrite(PmodeLED,(Pmode/4)); // Green LED
//    analogWrite(SmodeLED,(Smode/4)); // Red LED

// Check for Lock Side

    if ((LCB[ControlRegister] & 0x1) == 0x1) LoopDifference = -LoopDifference; // Red/blue side

    AverageLoopDifference = ((15 * AverageLoopDifference) + LoopDifference) >> 4; // 16 point moving average

// State machine

Value0 = TotalTime - CurrentTime;

    switch(State) {
    
// Startup - State 0 - Check for laser emission
 
    case 0:
    
// HeaterValue = 0; // Heater off

Value1 = 0;
Value2 = 0;
Value3 = 0;

    if (((TotalTime - CurrentTime) > 20) && ((Pmode > 100) || (Smode > 100)))
      {
        NoLaserLight = 0;
        TimeStamp = TotalTime;
        CurrentTime = TotalTime;
        Duration = LCB[Duration1];
        HeaterValue = LCB[HeaterValue1]; 
        ReLockCounter = LCB[ReLockCount];
        State = 1; // Go to warmup
      }
    else if ((TotalTime - CurrentTime) > Duration)
      {
        NoLaserLight = 1; // Laser does not turn on
        CurrentTime = TotalTime;
        Duration = LCB[Duration7];
        HeaterValue = LCB[HeaterValue7];
        digitalWrite(ErrorLED, HIGH); 
        State = 7;
      }
    break;

// Warmup - State 1 - Increase temperature of laser tube while monitoring mode sweep rate

   case 1:

Value1 = TotalTime - TimeStamp;
Value2 = LoopDifference;
Value3 = LoopDifferenceStamp;

    if (LoopDifference < 0)

//    if ((LoopDifference < 0) && (PreviousLoopDifference >= 0)) // Zero crossing + to -
      {
        TimeStamp = TotalTime; // Yes, reset reference time
      }    
    else if ((TotalTime - TimeStamp) > (LCB[ModePeriod] >> 1)) // Mode sweep slow enough to go to locking state?
      {
        LoopDifferenceStamp = LoopDifference;
        CurrentTime = TotalTime;
        Duration = LCB[Duration3];
        HeaterValue = LCB[HeaterValue3];
        State = 3; // Tube hot enough to start locking
      }
    else if ((TotalTime - CurrentTime) > Duration)
      {
        CurrentTime = TotalTime;
        Duration = LCB[Duration7];
        HeaterValue = LCB[HeaterValue7];
        digitalWrite(ErrorLED, HIGH); 
        State = 7;
      }
    break;

// Heating - State 2 - Increase temperature of laser tube

    case 2:

Value1 = 0;
Value2 = 0;
Value3 = 0;

    if ((TotalTime - CurrentTime) > Duration)
      {
        LoopDifferenceStamp = 0;
        TimeStamp = TotalTime;
        AverageLoopDifference = 0;
        CurrentTime = TotalTime;
      	Duration = LCB[Duration3];
        HeaterValue = LCB[HeaterValue3];
      	State = 3;
      }
    break;
    
// Locking - State 3 - Wait until laser locks, then fine tune heater voltage

    case 3:
      
    Temp1 = LoopDifference * LCB[ProportionalGain];
    if (Temp1 < -2048) HeaterValue = 0;
    else if (Temp1 >  2047) HeaterValue = 255;
    else HeaterValue = (Temp1 >> 4) + 128;

Value1 = TotalTime - TimeStamp;
Value2 = AverageLoopDifference;
Value3 = LoopDifferenceStamp;

    if (((AverageLoopDifference - LoopDifferenceStamp) > LCB[LockingTolerance]) || ((AverageLoopDifference - LoopDifferenceStamp) < -LCB[LockingTolerance])) // Is Error large?
      {
        TimeStamp = TotalTime; // It's not locked, reset reference time
        LoopDifferenceStamp = AverageLoopDifference;
      }    
    else if (((Pmode < 51) && (Smode < 51)) || ((Pmode > 972) && (Smode > 972)))
      { // Mode signals out of range
        CurrentTime = TotalTime;
        Duration = LCB[Duration7];
        HeaterValue = LCB[HeaterValue7];
        digitalWrite(ErrorLED, HIGH); 
        State = 7;
      }
    else if ((TotalTime - TimeStamp) > Duration) // Is Error within locked bounds for long enough to call it locked
      {
        if (AverageLoopDifference < -LCB[LockingTolerance])
          {
            CurrentTime = TotalTime;
            Duration = LCB[Duration2];
            HeaterValue = LCB[HeaterValue2];
            State = 2; // Heat for fixed time
          }
        else if (AverageLoopDifference > LCB[LockingTolerance])
          {
            CurrentTime = TotalTime;
            Duration = LCB[Duration4];
            HeaterValue = LCB[HeaterValue4];
            State = 4; // Cool for fixed time
          }
        else
          {
            IntegralAccum = 0;
            LockTime = TotalTime;
            CurrentTime = TotalTime;
            Duration = LCB[Duration5];
            HeaterValue = LCB[HeaterValue5];
            State = 5; // Locked in Lock State :)
          }
      }
    else if ((TotalTime - CurrentTime) > 3600)
      {
        CurrentTime = TotalTime;
        Duration = LCB[Duration7];
        HeaterValue = LCB[HeaterValue7];
        digitalWrite(ErrorLED, HIGH); 
        State = 7;
      }
    break;

// Cooldown - State 4 - Decrease temperature of laser tube

    case 4:
  
Value1 = 0;
Value2 = 0;
Value3 = 0;

if ((TotalTime - CurrentTime) > Duration)
      {
        LoopDifferenceStamp = 0;
        TimeStamp = TotalTime;
        AverageLoopDifference = 0;
        CurrentTime = TotalTime;
        Duration = LCB[Duration3];
        HeaterValue = LCB[HeaterValue3];
        State = 3;
      }
      
    break;

// Locked - State 5 - Stay here forever once laser has locked at 50 percent +/-12.5 percent heater after checking ReLockCount times

    case 5:

Value1 = AverageLoopDifference;
Value2 = IntegralAccum;
Value3 = ReLockCounter;

if (((AverageLoopDifference) > (LCB[LockingTolerance] << 4)) || ((AverageLoopDifference) < (-LCB[LockingTolerance] << 1))) // Is Error large?
      { // Way out of range, go back to proportional-only loop
         IntegralAccum = 0;
         IntegralTerm = 0;
      }
    else
      {
        IntegralAccum += LoopDifference * LCB[IntegralGain];
        IntegralTerm = IntegralAccum >> 10;  
      }
        
    Temp1 = LoopDifference * LCB[ProportionalGain] + IntegralTerm;
    if (Temp1 < -4096) HeaterValue = 0;
    else if (Temp1 >  4095) HeaterValue = 255;
    else HeaterValue = (Temp1 >> 5) + 128;

    if (((TotalTime - CurrentTime) > Duration) && (ReLockCounter > 0)) // Go to locking state to readjust after some time - usually infinity. :)
      { // Time to fine tune lock point
        ReLockCounter--;
        LoopDifferenceStamp = 0;
        TimeStamp = TotalTime;
        AverageLoopDifference = 0;
        CurrentTime = TotalTime;
        Duration = LCB[Duration3];
        State = 3;
      }
    else if (((Pmode < 51) && (Smode < 51)) || ((Pmode > 972) && (Smode > 972)))

      { // Mode signals out of range
        CurrentTime = TotalTime;
        Duration = LCB[Duration7];
        HeaterValue = LCB[HeaterValue7];
        digitalWrite(ErrorLED, HIGH); 
        State = 7;
      }
    break;

// Hangout - State 6 - Test hangs in this state

    case 6:

Value1 = 0;
Value2 = 0;
Value3 = 0;

// Nothing to do here except hang out
    
    break;

// Error - State 7 - Currently only if tube never lights
    
    case 7:

Value1 = 0;
Value2 = 0;
Value3 = 0;

if ((analogRead(PmodeInput) > 100) || (analogRead(SmodeInput) > 100))
      {
        NoLaserLight = 0; // Laser is on
        CurrentTime = TotalTime;
        TimeStamp = TotalTime;
        Duration = LCB[Duration1];
        HeaterValue = LCB[HeaterValue1];
        digitalWrite(ErrorLED, LOW); 
        State = 1; // Laser is on, go to Warmup
      }
    else if ((TotalTime - CurrentTime) > Duration)
      {
        NoLaserLight = 1; // Laser does not turn on but let's be optimistic! :)
        CurrentTime = TotalTime;
        Duration = LCB[Duration0];
        HeaterValue = LCB[HeaterValue0];
        digitalWrite(ErrorLED, LOW); 
        State = 0; // Try again
      }
    break;    
    }  

    analogWrite(HeaterDrive,HeaterValue); // Load heater drive PWM
    HeaterValueAverage += HeaterValue;
    
// End of control code

// Update COMM data if necessary
  
    noInterrupts();

    if ((TotalTotalTime & 0xf) == 0xf) // Update queue on every 16th interrupt, 1/61.25th of a second
      {
        LowSpeedCode = 0;
        LowSpeedData = 0;
        SequenceNumber++;
        LowSpeedCodeSelect = SequenceNumber & 0xf;

        if (ReadbackLCB == false)
          {
            if (LowSpeedCodeSelect == 0) // Send time (redundant, serial number also is time but....)
              {
                LowSpeedCode = 12;
                LowSpeedData = TotalTime;
              }
            else if (LowSpeedCodeSelect == 1) // Send sample frequency
              {
                LowSpeedCode = 8;
                LowSpeedData = 6104;
              }
            else if (LowSpeedCodeSelect == 2) // Send firmware version
              {
                LowSpeedCode = 10;
                LowSpeedData = FirmwareVersion;
              }
            else if (LowSpeedCodeSelect == 3) // Send state
              {
                LowSpeedCode = 11;
                LowSpeedData = State;
              }
            else if (LowSpeedCodeSelect == 4) // Send lock time
              {
                LowSpeedCode = 15;
                LowSpeedData = LockTime;
              }
            else if (LowSpeedCodeSelect == 5) // Send command
              {
                LowSpeedCode = 101;
                LowSpeedData = Command;
              }
            else if (LowSpeedCodeSelect == 6) // Send parameter 1
              {
                LowSpeedCode = 102;
                LowSpeedData = Param1;
              }
            else if (LowSpeedCodeSelect == 7) // Send parameter 2
              {
                LowSpeedCode = 103;
                LowSpeedData = Param2;
              }
            else if (LowSpeedCodeSelect == 8) // Send parameter 3
              {
                LowSpeedCode = 104;
                LowSpeedData = Param3;
              }
            else if (LowSpeedCodeSelect == 9)  // Send Diagnostic Value 0
              {
                LowSpeedCode = 110;
                LowSpeedData = Value0;
              }
            else if (LowSpeedCodeSelect == 10) // Send Diagnostic Value 1
              {
                LowSpeedCode = 111;
                LowSpeedData = Value1;
              }
            else if (LowSpeedCodeSelect == 11) // Send Diagnostic Value 2
              {
                LowSpeedCode = 112;
                LowSpeedData = Value2;
              }
            else if (LowSpeedCodeSelect == 12) // Send Diagnostic Value 3
              {
                LowSpeedCode = 113;
                LowSpeedData = Value3;
              }
            else if (LowSpeedCodeSelect == 13) // UCB status 1 if valid
              {
                LowSpeedCode = 105;
                LowSpeedData = UCBValid;
              }
            else if (LowSpeedCodeSelect == 14) // REF counts in 1/977th s
              {
                LowSpeedCode = 20;
                LowSpeedData = REFCount;
              }

          }   

        else // Read back entire LCB
          {
            LowSpeedCode = ReadbackIndex + 130;
            LowSpeedData = LCB[ReadbackIndex];
        
            if (ReadbackIndex == 31)
              {
                 ReadbackLCB = false;
                 ReadbackIndex = -1;
              }
            else
              {
              	ReadbackIndex++;
              }
          }
              
        if (!(((0==tail)&&((FIFO_SIZE-1)==head))||(head==(tail-1)))) // check if buffer is full; either tail is at beginning of array and head is at end, or head is right before tail
          {
// head points to the element to store next data

            dataPoint[head].displayPmode = PmodeAverage >> 4;
            dataPoint[head].displaySmode = SmodeAverage >> 4;
            dataPoint[head].displayHeater = HeaterValueAverage >> 4;
            dataPoint[head].displaySequenceNumber = SequenceNumber;
            dataPoint[head].displayLowSpeedCode = LowSpeedCode;
            dataPoint[head].displayLowSpeedData = LowSpeedData;

            PmodeAverage = 0;
            SmodeAverage = 0;
            HeaterValueAverage = 0;

// increment head
            if ((FIFO_SIZE-1)==head)
              head = 0;
            else
              head = head +1;
          }
        else
          {
            bufferOverflowCounter++;  // buffer is full. Increment counter
          }
      }
   interrupts();
} 
static String readString;   //main captured String

volatile char c;

volatile int ind1;
volatile int ind2;
volatile int ind3;
volatile int ind4;

volatile int sn;

void loop()
  { 
    if ((State & 0x1) == 0x1) digitalWrite(StateBit0LED, HIGH);
    else digitalWrite(StateBit0LED, LOW);
    if ((State & 0x2) == 0x2) digitalWrite(StateBit1LED, HIGH);
    else digitalWrite(StateBit1LED, LOW);
    if ((State & 0x4) == 0x4) digitalWrite(StateBit2LED, HIGH);
    else digitalWrite(StateBit2LED, LOW);

    if (State == 5) digitalWrite(LockedLED, HIGH);
    else digitalWrite(LockedLED, LOW);
 
    int length = 0;
    if (head != tail) // only send a new data packet if the queue is not empty; probably superfluous since serial.println blocks
      {
        noInterrupts();

        length += sprintf(outbuffer,"%d ",dataPoint[tail].displayPmode);
        length += sprintf(outbuffer+length,"%d ",dataPoint[tail].displaySmode);
        length += sprintf(outbuffer+length,"%d ",dataPoint[tail].displayHeater);
        length += sprintf(outbuffer+length,"%lu ",dataPoint[tail].displaySequenceNumber);
        length += sprintf(outbuffer+length,"%d ",dataPoint[tail].displayLowSpeedCode);
        length += sprintf(outbuffer+length,"%ld",dataPoint[tail].displayLowSpeedData);
 
// update queue
        if ((FIFO_SIZE-1)==tail)  // check for wrap around
          tail = 0;               // update tail location
        else  
          tail = tail +1;         // update tail location        
  
        interrupts();

// Send data
        Serial.println(outbuffer);
     }

// Read commands from GUI - expect a string: Command:Param1:Param2:Param3*

    nda = Serial.available();
    if (nda > 0)
      {
        c = Serial.read();  // gets one byte from serial buffer
        if (c == '*') 
          {       // do stuff
            ind1 = readString.indexOf(':');                               //finds location of first :
            Command = (readString.substring(0, ind1)).toInt();        //captures first data String
            ind2 = readString.indexOf(':', ind1+1);                       //finds location of second :
            Param1 = (readString.substring(ind1+1, ind2+1)).toInt();      //captures second data String
            ind3 = readString.indexOf(':', ind2+1);
            Param2 = (readString.substring(ind2+1, ind3+1)).toInt();
            ind4 = readString.indexOf(':', ind3+1);
            Param3 = (readString.substring(ind3+1, ind4+1)).toInt();
            NewCommand = true;
                     
            readString = "";  //clears variables for new input
          } 
        else
          {     
            readString += c; //makes the string readString
          }
      }

    if (NewCommand == true)
      {
        if ((Command >= 10) && (Command < 18)) // State parameter command
          {          	
            switch (Command - 10)
              {
                case 0:
                  {
                    LCB[Duration0] = Param1;
                  }
                break;
        
                case 1:
                  {
                    LCB[Duration1] = Param1;
                    LCB[ModePeriod] = Param2;
                  }
                break;

                case 2:
                  {
                    LCB[Duration2] = Param1;
                    LCB[HeaterValue2] = Param2;
                  }
                break;
    
                case 3:
                  {
                    LCB[Duration3]  = Param1;
                    LCB[LockingTolerance] = Param2;
                  }
                break;

                case 4:
                  {
                    LCB[Duration4] = Param1;
                    LCB[HeaterValue4] = Param2;
              }
                break;

                case 5:
                  {
                    LCB[Duration5] = Param1;
                    LCB[ReLockCount] = Param2;
                  }
                break;

                case 6:
                  {
                    LCB[Duration6] = Param1;
                    LCB[HeaterValue6] = Param2;
                  }
                break;

                case 7:
                  {
                    LCB[Duration7] = Param1;
                  }
                break;
              }
          }
          
        else if ((Command >= 20) && (Command < 28)) // State change command
          {
            State = Command - 20;
            CurrentTime = TotalTime;
            TimeStamp = TotalTime;
            AverageLoopDifference = 0;
            Duration = Param1;
           
            switch (Command - 20)
              {
                case 0:
                  {
                    LCB[Duration0] = Duration;
                    HeaterValue = LCB[HeaterValue0];
                  }
                break;
        
                case 1:
                  {
                    LCB[Duration1] = Duration;
                    LCB[ModePeriod] = Param2;
                    HeaterValue = LCB[HeaterValue1];
                  }
                break;

                case 2:
                  {
                    LCB[Duration2] = Duration;
                    HeaterValue = Param2;
                    LCB[HeaterValue2] = HeaterValue;
                  }
                break;
    
                case 3:
                  {
                    LoopDifferenceStamp = LoopDifference;
                    LCB[Duration3] = Duration;
                    LCB[LockingTolerance] = Param2;
                  }
                break;

                case 4:
                  {
                    LCB[Duration4] = Duration;
                    HeaterValue = Param2;
                    LCB[HeaterValue4] = HeaterValue;
                  }
                break;

                case 5:
                  {
                    LCB[Duration5] = Duration;
                    ReLockCounter = Param2;
                    LCB[ReLockCount] = ReLockCounter;
                    IntegralAccum = 0;
                    LockTime = TotalTime;
                  }
                break;

                case 6:
                  {
                    LCB[Duration6] = Duration;
                    HeaterValue = Param2;
                    LCB[HeaterValue6] = HeaterValue;
                  }
                break;

                case 7:
                  {
                    LCB[Duration7] = Duration;
                    HeaterValue = LCB[HeaterValue7];
                  }
                break;
              }
          }

        else if ((Command >= 30) && (Command < 50)) // Pmode and Smode limits and PID gains 
          {
        
            switch (Command)
              {
                case 31:
                  {
                    LCB[PmodeMin] = Param1;
                  }
                break;

                case 32:
                  {
                    LCB[PmodeMax] = Param1;
                  }
                break;

                case 33:
                  {
                    LCB[SmodeMin] = Param1;
                  }
                break;

                case 34:
                  {
                    LCB[SmodeMax] = Param1;
                  }
                break;

                case 36:
                  {
                    LCB[ProportionalGain] = Param1;
                  }
                break;

                case 37:
                  {
                    LCB[IntegralGain] = Param1;
                  }
                break;

                case 38:
                  {
                    LCB[DifferentialGain] = Param1;
                  }
                break;

                case 40:
                  {
                  	LCB[Offset] = Param1;
                  	LCB[Intensity] = Param2;
                  }

                break;
                
                case 41:
                  {
                    LCB[ControlRegister] = Param1;
                    State = 3;
                    CurrentTime = TotalTime;
                    TimeStamp = TotalTime;
                    AverageLoopDifference = 0;
                    LoopDifferenceStamp = LoopDifference;
                    Duration = LCB[Duration3];
                  }
                break;
              }
          }
        else if ((Command >= 10000) && (Command < 11000)) // Data transfer and EEPROM commands
          {
            switch (Command)
              {
                case 10101: // Load Default Parameters: Reload LCB from DCB and readback to GUI
                  {
                  	for (i = 0;i < 32; i++)
                      {
                        LCB[i] = DCB[i];
                      }
                  	ReadbackIndex = 0;
                    ReadbackLCB = true;
                  }
                break;     

               case 10202: // Load User Parameters: Reload LCB from UCB and readback to GUI
                  {
                  	UCBValid = 0;
                    EEPROM.get(0, i);
                    if (i == EEPROMFormatValid) // 1234 indicates data is valid
                      {
                        for (i = 0;i < 32; i++)
                          {
                            EEPROM.get(i<<1, LCB[i]); // Address in bytes for EEPROM.get regardless of datatype for value
                          }
        
                        UCBValid = 1;
                        ReadbackIndex = 0;
                        ReadbackLCB = true;
                      }
                  }
                break;     

                case 10303: // Save User Parameters: Copy LCB to UCB in EEPROM
                  {
                  LCB[0] = EEPROMFormatValid;
                  for (i = 0;i < 32; i++)
                      {
                        EEPROM.update(i<<1, LCB[i] & 0xff);              // Low byte of LCB
                        EEPROM.update((i<<1) + 1, (LCB[i] >> 8) & 0xff); // High byte of LCB
                      }
                  }
                break;
                
                case 10505: // Readback LCB to GUI
                  {
                  	ReadbackIndex = 0;
                    ReadbackLCB = true;
                  }
                break;   
              }
          } 
  	    NewCommand = false;  
    }
  }
