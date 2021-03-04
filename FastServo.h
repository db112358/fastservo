#ifndef FastServo_h
#define FastServo_h

#include "Arduino.h"

// max frequency for minimum of 8 bit resolution PWM would be to set ICR1 to 255
// phase correct mode / freq correct mode means counts up and then back down again per period
// 16MHz / (2 * 255) = ~31.4 Khz 
// 25Khz is the upper limit of BTS7960 H-Bridge

// 16MHz clock means 1/16MHz = 62.5nS per cpu tick
// 25kHz = 40uS period, 1 tick/62.5nS = 640 clock ticks per 40uS period 
// 10kHz = 100uS period, 1 tick/62.5nS = 1600 clock ticks per 100uS period
// 1kHz = 1ms period, 1 tick/62.5nS = 16000 clock ticks per 1ms period

// e.g 10Khz:
// 1600 / 2 = 800 ticks till timer hits TOP (ICR1) and starts counting back down again
// ICR1 = 800;

// 8000 / PWM_FREQ = desired ICR value

#define FS_LATENCY 40 // [ms]

// 2000 errors per second / 32 ms = 62.5 errors per control loop
#define FS_MAX_V_ERRORS 60

// 8000 / 15 kHz = 533.33
#define FS_PWM_ICR 8000 / 10 // khz

#define DS_BUFFER_SIZE 4
#define VD_BUFFER_SIZE 4

// state
#define FS_ERROR		-1 // TODO: error state
#define FS_DISABLED		0 // emergency stop: motor disabled, tracking disabled
#define FS_ENABLED		1 // motor enabled, tracking position set point
#define FS_STOPPING		2 // stop manoeuvre 

class FastServo
{
public:
	FastServo();

	// everything required to initialise servo 
	bool init(	volatile int16_t * enc_pos,
				uint8_t pwm_ch,
				bool reverse,
				uint8_t en_pin,
				int16_t v_max,
				int16_t v_min,
				int16_t a_max
				);

	// give our servo execution time (plz)
	void tick(uint32_t t_now);

	// are we at our target position?
	bool atTarget();

	// jammed / reverse detection
	bool throttleError();

	// move to position
	void moveTo(int16_t s);

	// set position
	void setPos(int16_t s);

	// change states
	void disable();
	void enable();
	void stop();

// parameter access ----------------------------------------------------------------------

	// get lag
	uint32_t getLag();

	// get current position
	int32_t getPos();

	// get target position
	int32_t getTargetPos();

	// get current speed
	int16_t getVelocity();

	// get target velocity
	int16_t getTargetVelocity();

	// get ideal velocity
	int16_t getIdealVelocity();

	// get velocity error
	float getVelocityError();

	// get current PID terms
	float getP();
	float getI();
	float getD();

	// get current throttle
	int16_t getThrottle();

	// set position deadzone
	void setPositionDeadzone(uint8_t s);

	// sets motor PID constants
	void setConstants(float Kp, float Ki, float Kd, float Ki_decay, float KvFF);

	// set max velocity
	void setMaxVelocity(int16_t v_max);

	// set min velocity
	void setMinVelocity(int16_t v_min);

	// set max acceleration 
	void setMaxAcceleration(int16_t a_max);

	// set throttle deadzone
	void setThrottleDeadzone(int8_t deadzone);

	// set max throttle
	void setMaxThrottleDuty(int8_t duty);
private:

// configuration ------------------------------------------------------------------------

	// pwm channel - mega:[1-4]
	uint8_t _pwm_ch = 1;

	// reverse motor direction?
	bool _reverse = false;

	// keep a track of our pin usage
	uint8_t _en_pin = 0; // motor output enable pin

	// +- positional deadzone
	uint8_t _s_deadzone = 2;

	// maximum velocity
	int16_t _v_max = 0;

	// minimum velocity
	int16_t _v_min = 0;

	// maximum acceleration
	int16_t _a_max = 0;

	// max acceleration per control loop (cached)
	float _a_inc = 0;

	// PID parameters
	float _k_vP = 0;
	float _k_vI = 0;
	float _k_vI_decay = 0;
	float _k_vD = 0;
	float _k_vFF = 0;

	// throttle deadzone
	uint8_t _thr_deadzone = 0;

	// max throttle
	int16_t _thr_max = FS_PWM_ICR;


// runtime variables --------------------------------------------------------------------
	
	// runtime state
	int8_t _state = FS_DISABLED;

	// throttle error detected?
	bool _throttle_error = false;

	// found home?
	bool _found_home = false;

	// keep a track of wether we're initialised
	// 0b0000000x = init has run
	// 0b000000x0 = we have PID constants
	// 0b00000x00 = max speed is set
	// 0b0000x000 = max accel is set
	// 0b00011111 = ready to run
	uint8_t _init = 0b00000000;

	// last time control loop was run
	uint32_t _t_prev = 0;

	// benchmarking 
	uint32_t _lag = 0;

	// pointer to volatile encoder position
	volatile int16_t *_enc_pos = 0;

	// current position in steps
	int16_t _s = 0;

	// position during the last run of the control loop
	int16_t _s_prev = 0;

	// target position
	int16_t _s_target = 0;

	// stopping distance at _v_max in steps
	int16_t _s_stop = 0;

	// current speed - updated every time control loop is run
	float _v = 0;

	// delta position buffer used to compute moving average of velocity
	int16_t _ds_buffer[DS_BUFFER_SIZE] = { 0 };
	uint8_t _ds_i = 0;
	int32_t _ds_sum = 0;

	// target speed - adjusted every time control loop is run
	// needs float precision to accumulate fractional amounts
	float _v_target = 0;

	// ideal velocity given current set point
	float _v_ideal = 0;

	// error in velocity
	float _v_ut = 0;

	// number of control loops velocity has been in error state
	uint8_t _v_errors;

	// PID terms
	float _vP = 0;
	float _vI = 0;
	float _vD = 0;

	// calculate moving average of derivative of velocity error
	float _v_ut_prev = 0;
	int16_t _vd_buffer[VD_BUFFER_SIZE] = { 0 };
	uint8_t _vd_i = 0;
	int16_t _vd_sum = 0;

	// throttle output
	int16_t _thr = 0;


// private functions --------------------------------------------------------------------

	// reset state
	void _reset();

	// calculate velocity
	void _updateVelocity(int16_t ds);

	// calculate velocity set point
	void _updateTargetVelocity();

	// calculate throttle based on velocity vs velocity set point
	void _updateThrottle();

	// update pwm outputs to motor based on throttle
	void _updateMotor();

	// helper function to perform 16-bit write to PWM registers
	void _setPWM(uint16_t thr_a, uint16_t thr_b);

	// helper function to calculate stopping distance given v and a
	int16_t _calcStop(int16_t v, int16_t a);
};

#endif