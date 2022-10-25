/****************************************************************************

  Ferriscope Rotor firmware.

  Bull.Miletic
  Ferriscope
  1893-2020

  https://bull.miletic.info

  The Rotor code is written by Magnus Sjursen.
  This version is modified by Jensa to match
  the new pinout of the PSI board.

*****************************************************************************/

#include <stdlib.h>;
#include <avr/interrupt.h>;
#include <math.h>;

// LED indicators for internal program state
#define RUN_INTERNAL_PIN A0 // Output, drives blue LED
#define INDEX_INTERNAL_PIN A1 // Output, drives red LED
#define STATE_RED_PIN A5 // Output, drives red colour of RGB LED
#define STATE_GREEN_PIN A2 // Output, drives green colour of RGB LED
#define STATE_BLUE_PIN A4 // Output, drives blue colour of RGB LED

// Ferriscope input
#define RUN_PIN_PIN PIND    // Port D Input Pins Address
#define RUN_PIN_PORT PORTD  // Port D Data Register
#define RUN_PIN_DDR DDRD    // Port D Data Direction Register

// Motor interface
//#define MOTOR_DISABLE_PIN 5 // Output, disables motor
#define MOTOR_DISABLE_PIN_AVR PC6 // Arduino pin 5
// 
// When high, the motor will turn anti-clockwise
#define RUN_PIN_AVR PD0     // Arduino pin 4  on Jensa PCB, moved to PD0
#define MOTOR_ANTICLOCKWISE_PIN 4 // Output, selects motor direction
// 
#define MOTOR_STEP_PIN 2 // Output, steps motor
#define MOTOR_STEP_PIN_AVR PD1
// (note hardwired solvering on back of PCB, bridging pins 7&8!)
#define MOTOR_INDEX_PIN 7 // Input, signals shaft index
#define MOTOR_INDEX_PIN_AVR PE6
  
const int ledPin = LED_BUILTIN;

// Serial terminal verbosity level.
// 0: Print nothing.
// 1: Print some things.
// 2: Print everything.
const uint8_t g_verbosity = 0;

const float main_clock_frequency = 16e6;
const uint16_t steps_per_round = 200 * 16;
volatile uint16_t g_position = 0; // Current position (steps within one round)
// The position at which we started decelerating (steps within one round)
volatile int16_t g_position_dec_start;
volatile int16_t g_turns = 0; // Current position (entire turns)
// The position at which we started decelerating (entire turns)
volatile int16_t g_turns_dec_start;
volatile bool g_index = 0; // Whether we index is in front of the sensor
volatile bool g_has_seen_index = 0; // Whether we have seen the index since booting
// The position where the last rising edge of the index sensor occured.
volatile uint16_t g_index_position = 0;
// Offset between index and home position
const uint16_t g_home_offset = -2220 + 200 * 16;
// Ferris #2 uses 515 (eller -2685), Ferris #3 uses -2220 as base

float max_delta_time = 0;

/*
   Notes about the homing:
   - We don't know exactly how this works due to the AVR'ness of it all,
     but we know it's counting ticks and when it finds the index, it
     will add g_home_offset to the index position.
   - The original calculation from Magnus was something like this:
     g_home_offset = 392 + 200*8; This no longer works...
   - My attempt at finding something that works led me to
     g_home_offset = 508 + 200 * 8; but that does not center perfectly
     every time

*/

// The home position (That is where the motor stops.)
volatile uint16_t g_home_position = 0;

// The control PCB has a LED indicating the program state.
enum class State_color {
  black,
  red,
  yellow,
  green,
  cyan,
  blue,
  purple,
  white
};

void setup_state_color() {
  pinMode(STATE_RED_PIN, OUTPUT);
  pinMode(STATE_GREEN_PIN, OUTPUT);
  pinMode(STATE_BLUE_PIN, OUTPUT);

  state_color(State_color::black);
}

void state_color(State_color color) {
  uint8_t rgb;

  switch (color) {
    case State_color::red:    rgb = 0b100; break;
    case State_color::yellow: rgb = 0b110; break;
    case State_color::green:  rgb = 0b010; break;
    case State_color::cyan:   rgb = 0b011; break;
    case State_color::blue:   rgb = 0b001; break;
    case State_color::purple: rgb = 0b101; break;
    case State_color::black:  rgb = 0b000; break;
    case State_color::white:  rgb = 0b111; break;
    default: rgb = 0b000;
  }

  bool r = (1 << 2) & rgb;
  if (r) {
    digitalWrite(STATE_RED_PIN, HIGH);
  } else {
    digitalWrite(STATE_RED_PIN, LOW);
  }
  bool g = (1 << 1) & rgb;
  if (g) {
    digitalWrite(STATE_GREEN_PIN, HIGH);
  } else {
    digitalWrite(STATE_GREEN_PIN, LOW);
  }
  bool b = (1 << 0) & rgb;
  if (b) {
    digitalWrite(STATE_BLUE_PIN, HIGH);
  } else {
    digitalWrite(STATE_BLUE_PIN, LOW);
  }
}


// The Ferriscope runs a state machine.
enum class State {
  boot, // Setup microcontroller.
  index, // Search for the shaft index.
  idle, // Wait for the run command.
  accelerate, // Gradually increase speed.
  // Find a better name - run is also the name of the input signal
  run, // Run at constant speed.
  decelerate // Gradually decrease speed, eventually stopping at the index.
};

volatile State g_current_state;
volatile State previous_state;

String state_string(State state = State::boot) {
  String s;
  switch (state) {
    case State::boot: s = "boot";
      break;
    case State::index: s = "index";
      break;
    case State::idle: s = "idle";
      break;
    case State::accelerate: s = "accelerate";
      break;
    case State::run: s = "run";
      break;
    case State::decelerate: s = "decelerate";
      break;
    default: s = "default";
  }
  return s;
}

void state(State new_state = State::boot) {
  // The default state is to work around a defect of the Arduino IDE's
  // automatic prototyping mechanism. The mechanism makes code with
  // a function with enum class arguments fail to compile, unless the
  // function has a default argument. See this link for details:
  // https://fowkc.wordpress.com/2013/12/04/how-the-arduino-ide-tries-to-be-too-helpful/

  String s = state_string(new_state);
  if ( g_current_state != new_state )
  {
    //Serial.println(String("set state(") + s + String(")"));
  }

  previous_state = g_current_state;
  g_current_state = new_state;

  State_color color;
  switch (new_state) {
    case State::boot: color = State_color::red;
      break;
    case State::index: color = State_color::yellow;
      break;
    case State::idle: color = State_color::green;
      /*
        Serial.print("Go idle at position: ");
        Serial.print( g_position );
        Serial.print(" index: ");
        Serial.println( g_index_position );
      */
      break;
    case State::accelerate: color = State_color::cyan;
      break;
    case State::run: color = State_color::blue;
      break;
    case State::decelerate: color = State_color::purple;
      break;
    default: color = State_color::white;
  }

  state_color(color);
  //Serial.println(String("set state(") + s + String(")"));
}

void setup_state() {
  g_current_state = State::boot;
  //previous_state = State::boot;
  state(State::boot);
}

void setup_internal_state_led() {
  pinMode(RUN_INTERNAL_PIN, OUTPUT);
  digitalWrite(RUN_INTERNAL_PIN, LOW);

  pinMode(INDEX_INTERNAL_PIN, OUTPUT);
  digitalWrite(INDEX_INTERNAL_PIN, LOW);
}

volatile bool g_current_run_pin = 0;
volatile bool g_previous_run_pin = 0;

void setup_run_pin() {
  // Set the run pin as input.
  RUN_PIN_DDR = RUN_PIN_DDR & ~(1 << RUN_PIN_AVR);
  //const bool run = (bool) ((1 << RUN_PIN_AVR) & RUN_PIN_PIN);
  bool g_current_run_pin = 0;
  bool g_previous_run_pin = 0;
}

void setup_motor_control() {
  DDRC = DDRC | (1 << MOTOR_DISABLE_PIN_AVR);
  PORTC = PORTC | (1 << MOTOR_DISABLE_PIN_AVR);

  pinMode(MOTOR_ANTICLOCKWISE_PIN, OUTPUT);
  digitalWrite(MOTOR_ANTICLOCKWISE_PIN, HIGH);

  DDRD = DDRD | (1 << MOTOR_STEP_PIN_AVR);
  PORTD = PORTD & ~(1 << MOTOR_STEP_PIN_AVR);
  //digitalWrite(MOTOR_STEP_PIN, LOW);

  pinMode(8, INPUT);
  pinMode(MOTOR_INDEX_PIN, INPUT_PULLUP);

  //digitalWrite(MOT1R_DISABLE_PIN, HIGH);
  PORTC = PORTC & ~(1 << MOTOR_DISABLE_PIN_AVR);
}

// The step timer - counts timer ticks used to generate
// motor step ouput signal.
volatile int32_t g_ticks = 0;

const uint16_t ocr_max = UINT16_MAX - 5000; // UINT16_MAX = 65535

// Step timer limit.
volatile int32_t g_ticks_target = 4 * ((int32_t) ocr_max);

void setup_step_timer() {
  TCCR3A = (0 << COM3A1) | // Turn off waveform generator.
           (0 << COM3A0) |
           (0 << COM3B1) |
           (0 << COM3B0) |
           (0 << COM3C1) |
           (0 << COM3C0) |
           (0 << WGM31) | // Clear timer on compare match.
           (0 << WGM30);

  TCCR3B = (0 << ICNC3) |
           (0 << ICES3) |
           (0 << 5) |
           (0 << WGM33) | // Clear timer on compare match.
           (1 << WGM32) |
           (0 << CS32) | // Stop clock.
           (0 << CS31) |
           (0 << CS30);

  g_ticks = 0;
  g_ticks_target = 4 * ((int32_t) ocr_max);

  // This is the speed at which the ISR will be called https://youtu.be/3kCWRyYfLDA?t=500
  OCR3A = ocr_max;

  TCCR3C = (0 << FOC3A) |
           (0 << 6) |
           (0 << 5) |
           (0 << 4) |
           (0 << 3) |
           (0 << 2) |
           (0 << 1) |
           (0 << 0);

  TIMSK3 = (0 << 7) |
           (0 << 6) |
           (0 << ICIE3) |
           (0 << 4) |
           (0 << OCIE3C) |
           (0 << OCIE3B) |
           (1 << OCIE3A) | // Interrupt on OCR1 match.
           (0 << TOIE3);
}

void inline start_step_timer() {
  TCCR3B = (0 << ICNC3) |
           (0 << ICES3) |
           (0 << 5) |
           (0 << WGM33) | // Clear timer on compare match.
           (1 << WGM32) |
           (0 << CS32) | // Start clock.
           (0 << CS31) |
           (1 << CS30);
}

// Never used...?
/*
  void inline stop_step_timer() {
  //TCCR3B &= ~(1 << CS30);
  TCCR3B = (0 << ICNC3) |
           (0 << ICES3) |
           (0 << 5) |
           (0 << WGM33) | // Clear timer on compare match.
           (1 << WGM32) |
           (0 << CS32) | // Stop clock.
           (0 << CS31) |
           (0 << CS30);
  }
*/
int32_t compute_speed_ticks(float speed) {
  // Compute compare match limit to put in OCR3A to
  // generate the target speed.
  const int32_t ticks = (int32_t)(
                          main_clock_frequency / (2 * steps_per_round * speed));

  return ticks;
}

// The time to wait at the end of the loop function.
const uint32_t g_t_loop_ms = 10;
volatile uint32_t g_t_last_loop = 0;
const float g_t_loop_s = ((float) g_t_loop_ms) * 0.001;

// WARNING: Do not set g_v_max above 10.0 Hz. When decelerating from a higher
// speed (12.0 Hz), the microcontroller crashes just as deceleration starts.
// This leads to a hard stop, which damages the shaft coupling. I'm not sure
// why the controller crashes. It might be an EMI issue, or a software one.
const float g_v_max = 8.0; // Hz shaft speed
const float v_min = 0.0001; // Hz shaft speed
// Hz, speed below which hard stopping is permissible
const float v_hardstop_max = 0.5;
const float g_v_index = 0.10; // Hz | speed at which to index
const float g_v_indexread_max = 1.0; // Hz | Speed below which it is permissible to read the index sensor. (The position error becomes larger at higher speeds.)

const int32_t ticks_hardstop_min = compute_speed_ticks(v_hardstop_max);
const int32_t ticks_min = compute_speed_ticks(g_v_max);
const int32_t g_ticks_indexread_min = compute_speed_ticks(g_v_indexread_max);

volatile float g_v = v_min;
// Limit discrete speed differences to avoid accelerating too
// hard, say if the loop sleeps for too long.
const float max_dv = 0.2; // Hz
const float t_acc = 60.0; // s
const float g_t_1 = 59.1; // s | The deceleration time
volatile float g_v_0 = 0.0; // Hz | The speed when deceleration began
const float g_v_1 = 0.05; // Hz | The speed just before stopping after decelerating
volatile uint32_t g_t_start_deceleration = 0;
const uint32_t g_t_deceleration_time_ms = ((uint32_t) (1000.0 * g_t_1)); // ms
volatile uint32_t g_t_finish_dec_ms = 0;
volatile uint32_t g_t_finish_dec_actual_ms = 0;
volatile bool g_flag_dec_print = 0;
// Accelerate to max speed in t_accel.
const float acceleration = g_v_max / t_acc; // Hz^2
// Used when decelerating to stop at the home position
//volatile float g_a_dec = 0;

// Constant for use in acceleration. We accelerate such that
// a = g_k_acc*t^g_acc_power . This is to make the acceleration
// lower at low speeds.
const float g_acc_power = 6.0;
const float g_k_acc = g_acc_power * g_v_max / pow(t_acc, g_acc_power);

volatile uint32_t time_ms = 0; // Timestamp for loop function

// Text from https://www.messletters.com/en/big-text/
char * name[3] =
{ " ___  _  _ _    _     _  _ _ _    ____ ___ _ ____ ",
  " |__] |  | |    |     |\\/| | |    |___  |  | |    ",
  " |__] |__| |___ |___ .|  | | |___ |___  |  | |___ "
};

char * title[3] =
{ "   _   _   _    _   ___   __   _   _    _    _ ",
  "  |_  |_  |_)  |_)   |   (_   /   / \\  |_)  |_ ",
  "  |   |_  | \\  | \\  _|_  __)  \\_  \\_/  |    |_ "
};

void setup() {
  // Wait to make time for reprogramming, in case our code
  // interferes with the bootloader.
  delay(2000);

  Serial.begin(115200);
  pinMode(8, INPUT);

  //Serial.println(name[0]);
  //Serial.println(name[1]);
  //Serial.println(name[2]);

  //Serial.println("Bull.Miletic");

  //Serial.println("");

  if (g_verbosity >= 1) {
    Serial.println(title[0]);
    Serial.println(title[1]);
    Serial.println(title[2]);

    Serial.println("");
    delay(500);
  }

  Serial.print("Ferriscope reset");
  delay(1000);
  Serial.print(" 3");
  delay(1000);
  Serial.print(" 2");
  delay(1000);
  Serial.println(" 1");
  delay(1000);

  Serial.print("Setup motor control ");
  setup_motor_control();
  Serial.println("[ OK ]");
  delay(50);

  //Serial.print("Setup run pin ");
  setup_run_pin();
  //Serial.println(" [ OK ]");
  delay(50);

  //Serial.print("Setup state LED ");
  setup_internal_state_led();
  //Serial.println("[ OK ]");
  delay(50);

  //Serial.print("Setup state color ");
  setup_state_color();
  //Serial.println("[ OK ]");
  delay(50);

  //Serial.print("Setup state ");
  setup_state();
  //Serial.println("[ OK ]");
  delay(50);

  //Serial.print("Setup step timer ");
  setup_step_timer();
  //Serial.println("[ OK ]");
  delay(50);

  // Setup heartbeat LED
  //pinMode(heartbeat_pin, OUTPUT);
  //digitalWrite(heartbeat_pin, LOW);

  //Serial.print("Start step timer");
  start_step_timer();
  //Serial.println(" [ OK ]");

  g_position = 0;
  g_index = !(PINE & (1 << MOTOR_INDEX_PIN_AVR));
  g_index_position = 0;

  delta_time();

  state(State::index); // FIXME: This reenables the motor!
  //Serial.println("Ferriscope setup complete");
}

float delta_time() {
  // Compute the time in s since this function was last called.
  const uint32_t current_time_ms = millis();
  const uint32_t previous_time_ms = time_ms;

  const float delta = ((float) (current_time_ms - previous_time_ms)) * 0.001;

  time_ms = current_time_ms;
  return delta;
}

volatile float g_a_0 = 0.0;
volatile float g_k_a = 0.0;

// Compute the minimum number of turns we can select for deceleration,
// to ensure that our velocity decreases monotonically.
float compute_d_1_min(float t_1, float v_0, float v_1, float d_0) {
  return d_0 + v_0 * t_1 + 2.0 / 3.0 * t_1 * (v_1 - v_0);
}

// Compute the maximum number of turns we can select for deceleration,
// to ensure that we never have positive acceleration.
float compute_d_1_max(float t_1, float v_0, float v_1, float d_0) {
  return d_0 + v_0 * t_1 + 1.0 / 3.0 * t_1 * (v_1 - v_0);
}

// Compute a_0 used for the deceleration curve.
float compute_a_0(float t_1, float v_0, float v_1, float d_0, float d_1) {
  return (6.0 * (d_1 - d_0 - v_0 * t_1) - 2 * t_1 * (v_1 - v_0)) / pow(t_1, 2);
}

// Compute k_a used for the deceleration curve.
float compute_k_a(float t_1, float v_0, float v_1, float d_0, float d_1) {
  return (6.0 * t_1 * (v_1 - v_0) - 12 * (d_1 - d_0 - v_0 * t_1)) / pow(t_1, 3);
}

volatile float g_d_1_min = 0.0;
volatile float g_d_1_max = 0.0;
volatile float g_d_1_guess = 0.0;
volatile int16_t g_d_1_guess_ = 0;
volatile float g_d_1 = 0.0;

void commutate_state() {
  // FIXME: Make this function readable.
  // Read the run pin.
  const bool run = (bool) ((1 << RUN_PIN_AVR) & RUN_PIN_PIN);
  g_previous_run_pin = g_current_run_pin;
  g_current_run_pin = run;

  cli();
  const State current_state = g_current_state;
  sei();

  if (current_state == State::boot) {
    // Start indexing.
    //Serial.println(String("commutate [biiard] boot -> index"));
    //Serial.println(String("          [L^    ]"));
    state(State::index);
  } else if (current_state == State::index) {
    // Do nothing (transistion to idle is handled by ISR).
  } else if (current_state == State::idle) {
    if (run) {
      // Start accelerating.
      g_v = v_min;
      //Serial.println(String("commutate idle -> accelerate"));
      //Serial.println(String("commutate [biiard] idle -> accelerate"));
      //Serial.println(String("          [  L^  ]"));
      state(State::accelerate);
    }
  } else if (current_state == State::decelerate) {
    if (run) {
      if (g_v < g_v_max) {
        // Accelerate
        //Serial.println(String("commutate decelerate -> accelerate"));
        //Serial.println(String("commutate [biiard] decelerate -> accelerate"));
        //Serial.println(String("          [   ^_J]"));
        state(State::accelerate);
      } else {
        // Run at max speed
        //Serial.println(String("commutate decelerate -> run"));
        //Serial.println(String("commutate [biiard] decelerate -> run"));
        //Serial.println(String("          [    ^J]"));
        state(State::run);
      }
    } else if (((g_t_last_loop - g_t_start_deceleration + 100)
                > (g_t_deceleration_time_ms)) &&
               // We might have to stop regardless.
               (g_v <= v_hardstop_max)) {
      // If we are decelerating, we should stop at the right time.
      //Serial.println(String("commutate decelerate -> idle"));
      //Serial.println(String("commutate [biiard] decelerate -> idle"));
      //Serial.println(String("          [  ^__J]"));
      state(State::index);
      /*
      Serial.print("g_position: ");
      Serial.print(g_position);
      Serial.print(" g_turns: ");
      Serial.println(g_turns);
      */
      g_v = v_min;
      g_t_finish_dec_actual_ms = millis();
      g_flag_dec_print = 1;
      //Serial.print(String("g_t_finish_dec_ms        ") +
      //  g_t_finish_dec_ms);
      //Serial.println(String(" ms"));
      //Serial.print(String("g_t_finish_dec_actual_ms ") +
      //  g_t_finish_dec_actual_ms);
      //Serial.println(String(" ms"));
      //Serial.print(String("planned t dec            ") +
      //  (g_t_finish_dec_ms - g_t_start_deceleration));
      //Serial.println(String(" ms"));
      //Serial.print(String("actual t dec             ") +
      //  (g_t_finish_dec_actual_ms - g_t_start_deceleration));
      //Serial.println(String(" ms"));
    }
  } else {
    // current_state == State::accelerate or State::run
    if (run) { // we're not getting the run signal
      if (g_v < g_v_max) {
        // Accelerate
        state(State::accelerate);
      } else {
        // Run at max speed
        state(State::run);
      }
    } else { // we're not getting the run signal
      if (g_previous_run_pin && !g_current_run_pin) {
        // Prepare to decelerate
        // Record state at start of deceleration
        cli();
        const int32_t current_position = ((int32_t) g_position);
        g_position_dec_start = g_position;
        g_turns_dec_start = g_turns;
        g_v_0 = g_v;
        g_t_start_deceleration = millis();
        sei();

        // The time at which to stop
        g_t_finish_dec_ms = g_t_start_deceleration +
                            ((uint32_t) (1000.0 * g_t_1));

        // In order to get a nice deceleration curve, we need to be careful
        // with our choice of how many rounds to go before stopping.
        const float d_1_min = compute_d_1_min(g_t_1, g_v_0, g_v_1, 0.0);
        const float d_1_max = compute_d_1_max(g_t_1, g_v_0, g_v_1, 0.0);
        const float d_1_guess = ceil((d_1_min + d_1_max) / 2);
        g_d_1_min = d_1_min;
        g_d_1_max = d_1_max;
        g_d_1_guess = d_1_guess;

        // Calculate how many steps to travel
        int32_t steps_to_travel = g_home_position - current_position - 100;
        if (steps_to_travel < 0) {
          steps_to_travel += steps_per_round;
        }
        const float home_diff =
          ((float) steps_to_travel) / ((float) steps_per_round);

        // The actual number of turns to travel
        const float d_1 = ceil(d_1_guess - home_diff) + home_diff;
        g_d_1 = d_1;

        // Calculate the deceleration curve parameters.
        g_a_0 = compute_a_0(g_t_1, g_v_0, g_v_1, 0.0, g_d_1);
        g_k_a = compute_k_a(g_t_1, g_v_0, g_v_1, 0.0, g_d_1);

        // Set the turns counter
        // The ISR will stop the motor when it reaches 0:home.
        //cli();
        //g_turns = -((int32_t) floor(turns_to_travel));
        //g_turns = 0;
        //sei();

        state(State::decelerate);
        //Serial.print(String("g_t_start_deceleration ") +
        //  g_t_start_deceleration);
        //Serial.println(String(" ms"));
      }
    }
  }
}

void update_speed() {
  // Limit speed range.
  if (g_v > g_v_max) {
    g_v = g_v_max;
  } else if (g_v < v_min) {
    g_v = v_min;
  }

  // Adjust timer to generate correct speed.
  const int32_t new_ticks_target = compute_speed_ticks(g_v);
  //Serial.print(String(" t: ") + new_ticks_target);
  set_speed(new_ticks_target);
}

// Limit x to within [x_min, x_max]
float limit(float x, float x_min, float x_max) {
  x = (x >= x_min) ? x : x_min;
  x = (x <= x_max) ? x : x_max;
  return x;
}

// Compute new velocity
float compute_v_new(float dt, float t_now, float t, float v_old,
                    float* p_new) {
  float v_new = 0.0;
  float p_now = 0.0;
  float p_err = 0.0;
  float v_adj = 0.0;
  if (g_current_state == State::accelerate) {
    // Quadratic
    //const float t_old = sqrt(2*v_old/g_k_acc);
    //v_new = 0.5*g_k_acc*pow(t_old + dt, 2);
    // Cubic
    const float t_old = pow(g_acc_power * v_old / g_k_acc, 1.0 / g_acc_power);
    const float t_new = t_old + dt;
    v_new = (1.0 / g_acc_power) * g_k_acc * pow(t_new, g_acc_power);
  } else if (g_current_state == State::decelerate) {
    // Let a(t) = g_a_0 + g_k_a*t, where t is the time since the start
    // of deceleration. Then v(t) = 1/2*g_k_a*pow(t, 2) + g_a_0*t + v_0.

    // Improve positioning by computing for the time about halfway
    // between this and the next cycle.
    //const float t_offset = t + 0.5*g_t_loop_s;
    //Serial.print(String("g_t_loop_s "));
    //Serial.println(g_t_loop_s, 3);
    const float t_offset = t + 0.53 * g_t_loop_s;

    // Figure out where we are.
    cli();
    const int16_t position = g_position;
    const int16_t turns = g_turns;
    sei();

    // FIXME: Could we end up with an overflow problem here?
    const int16_t p_offset = position - g_position_dec_start;
    const int16_t g_offset = turns - g_turns_dec_start;
    const int32_t p_total_offset = ((int32_t) p_offset) +
                                   ((int32_t) g_offset) * steps_per_round;
    p_now = ((float) p_total_offset) / ((steps_per_round));

    // A proportional regulator regulates the position during deceleration.
    // P = 20.0 gives oscillation.
    // P = 10.0 gives slight oscillation.
    // P = 7.5 is still slightly too much.
    const float g_reg_prop_v = 2.0; // P

    // Evaluate the deceleration curve.
    if (t_offset <= 0.0) {
      *p_new = 0.0;
      p_err = *p_new - p_now;
      v_adj = limit(g_reg_prop_v * p_err, -0.1, 0.1);
      v_new = g_v_0;
      v_new += v_adj;
    } else if (t_offset <= g_t_1) {
      *p_new = (1.0 / 6.0) * g_k_a * pow(t, 3) + 0.5 * g_a_0 * pow(t, 2) +
               g_v_0 * t;
      p_err = *p_new - p_now;
      v_adj = limit(g_reg_prop_v * p_err, -0.1, 0.1);
      v_new = 0.5 * g_k_a * pow(t_offset, 2) + g_a_0 * t_offset + g_v_0;
      v_new += v_adj;
    } else {
      *p_new = g_d_1;
      p_err = *p_new - p_now;
      v_adj = g_reg_prop_v * p_err;
      v_new = v_min;
      v_new += v_adj;
    }
  } else {
    v_new = v_old;
  }

  // FIXME: Don't print large floats.
  //Serial.print(String("t ") + t);
  //Serial.print(String(" v_new ") + v_new);
  //Serial.print(String(" p_new ") + *p_new);
  //Serial.print(String(" p_now ") + p_now);
  //Serial.print(String(" p_err "));
  //Serial.print(p_err, 4);
  //Serial.print(String(" v_adj "));
  //Serial.print(v_adj, 4);
  //Serial.println("");

  return v_new;
}

void loop() {
  // Run loop at fixed time intervals.
  while (1) {
    const uint32_t t_loop_start = millis();
    if ((t_loop_start - g_t_last_loop) >= g_t_loop_ms) {
      g_t_last_loop = t_loop_start;
      break;
    }
  }

  // Print data from the deceleration routine in the ISR.
  /*if (g_verbosity >= 1) {
    cli();
    const bool flag_dec_print = g_flag_dec_print;
    sei();
    if (flag_dec_print) {
      cli();
      g_flag_dec_print = 0;
      sei();
      Serial.println(String("ISR dec stop        ") +
        g_t_finish_dec_ms);
      Serial.println(String("ISR dec stop actual ") +
        g_t_finish_dec_actual_ms);
      Serial.println(String("ISR dec period        ") +
        (g_t_finish_dec_actual_ms - g_t_start_deceleration));
      Serial.println(String("ISR dec period actual ") +
        (g_t_finish_dec_actual_ms - g_t_start_deceleration));
    }
    }*/

  // Compute actual time since last iteration.
  const float dt = delta_time();
  if ( dt > max_delta_time )
  {
    max_delta_time = dt;
    //Serial.print("max_delta_time: ");
    //Serial.println(max_delta_time, 5);
  }

  commutate_state();

  // Compute new speed
  const float v_old = g_v;
  const uint32_t t_now = millis();
  const float t = ((float) (t_now - g_t_start_deceleration)) * 0.001;
  float p_new = 0.0;
  const float v_new = compute_v_new(dt, t_now, t, v_old, &p_new);

  // Limit the velocity difference
  float dv = v_new - v_old;
  dv = (dv >= -max_dv) ? dv : -max_dv;
  dv = (dv <= max_dv) ? dv : max_dv;
  float v_limited = v_old + dv;

  //Serial.print(String("dv ") + dv + String(" Hz "));

  // Should we limit acceleration here?

  // Limit velocity
  v_limited = (v_limited >= v_min) ? v_limited : v_min;
  v_limited = (v_limited <= g_v_max) ? v_limited : g_v_max;

  // Apply the new speed
  switch (g_current_state) {
    case State::boot: g_v = v_min; break;
    case State::index: g_v = g_v_index; break;
    case State::idle: g_v = v_min; break;
    case State::accelerate: g_v = v_limited; break;
    case State::run: break;
    case State::decelerate: g_v = v_limited; break;
  }
  update_speed();


  // Read current position.
  cli();
  const uint16_t current_position = g_position;
  const int16_t current_turns = g_turns;
  sei();

  // Print index position.
  //Serial.print(String("g_home_position ") + g_home_position);
  // Print current position.
  //Serial.print(String(" pos ") + current_turns);
  //Serial.println(String(" : ") + current_position);

  // Print time remaining to stop (during deceleration)
  //Serial.println(String("millis ") + millis());
  //Serial.println(String("g_t_f_d ") + g_t_finish_deceleration);
  //Serial.print(String("g_a_0 "));
  //Serial.print(g_a_0, 3);
  //Serial.print(String(" g_k_a "));
  //Serial.print(g_k_a, 3);
  //Serial.println(String(" g_v_0 ") + g_v_0);

  //if (g_verbosity >= 2) {
  if (0) {
    //Serial.print(String("g_d_1_min ") + g_d_1_min);
    //Serial.print(String(" g_d_1_max ") + g_d_1_max);
    //Serial.print(String(" g_d_1_guess ") + g_d_1_guess);
    //Serial.println(String(" g_d_1 ") + g_d_1);
  }

  if (0) {
    // Inspect ticks and ticks_target
    cli();
    const int32_t ticks_b = g_ticks;
    const int32_t ticks_target_b = g_ticks_target;
    sei();
    //Serial.println(String("ticks ") + ticks_b);
    //Serial.println(String("ticks_target ") + ticks_target_b);
  }
  //Serial.println(String("state ") + state_string(g_current_state));
}

void set_speed(int32_t new_ticks_target) {
  // Enforce sane values
  if (new_ticks_target < ticks_min) {
    new_ticks_target = ticks_min;
  }

  cli();
  g_ticks_target = new_ticks_target;
  sei();
}

void inline step_pin() {
  // Toggle pin
  bool step_pin = PIND & (1 << MOTOR_STEP_PIN_AVR);
  if (step_pin) {
    // Falling edge of step pin
    PORTD = PORTD & ~(1 << MOTOR_STEP_PIN_AVR);
  } else {
    // Rising edge
    PORTD = PORTD | (1 << MOTOR_STEP_PIN_AVR);

    // Count position of step pin
    g_position += 1;
    if (g_position >= steps_per_round) {
      g_position = 0;
      g_turns += 1;
    }

    // If we are indexing, we should stop a fixed amount after the index.
    if (g_has_seen_index &&
        (g_current_state == State::index)) {
      if (g_position == g_home_position) {
        // We should stop here, unless our speed is too high
        /*
          Serial.print("Stop for home position: ");
          Serial.print( g_v );
          Serial.print(" : ");
          Serial.println( v_hardstop_max );
        */
        if (g_v <= v_hardstop_max) {
          state(State::idle);
          g_v = v_min;
        }
      }
    }

    // Read position of index,
    // but only at low speeds.
    const bool current_index = !(PINE & (1 << MOTOR_INDEX_PIN_AVR));
    if ((g_ticks_target >= g_ticks_indexread_min) &&
        (!g_index && current_index) &&
        ((g_current_state == State::index) || (g_current_state == State::decelerate)) ) {
      // Rising edge of index signal
      g_index_position = g_position;
      /*
      Serial.print("Index set to: ");
      Serial.print( g_position );
      Serial.print(", g_ticks_target ");
      Serial.print( g_ticks_target );
      Serial.print(" >= g_ticks_indexread_min: ");
      Serial.println( g_ticks_indexread_min );
      */
      g_has_seen_index = 1;
      g_home_position = g_index_position + g_home_offset;
      if (g_home_position >= steps_per_round) {
        g_home_position -= steps_per_round;
      }
    }
    g_index = current_index;
  }
}

ISR(TIMER3_COMPA_vect) {
  cli();

  // Calculate total number of ticks this cycle.
  g_ticks += ((uint16_t) OCR3A);

  // Step if due
  int32_t remaining_ticks = g_ticks_target - g_ticks;
  if (remaining_ticks <= 0) {
    if ((g_current_state != State::boot) &&
        (g_current_state != State::idle)) {
      step_pin();
    }
    g_ticks = 0;
    remaining_ticks = g_ticks_target;
  }

  // Adjust OCR3A
  if (remaining_ticks <= ocr_max) {
    // Make sure not to set OCR3A below the current TCNT3
    const uint16_t ofs = 25;
    const uint16_t new_ocr = (TCNT3 + ofs > remaining_ticks) ?
                             TCNT3 + ofs : remaining_ticks;
    OCR3A = (uint16_t) new_ocr;
  } else if (remaining_ticks <= 2 * ((int32_t) ocr_max)) {
    // Try to avoid making the last OCR3A value before
    // a step too short.
    OCR3A = (uint16_t) (remaining_ticks / 2);
  } else {
    OCR3A = ocr_max;
  }

  sei();
}
