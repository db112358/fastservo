#include "FastServo.h"

FastServo::FastServo()
{
	// where dreams come true
}

bool FastServo::init(	volatile int16_t * enc_pos,
						uint8_t pwm_ch,
						bool reverse,
						uint8_t en_pin,
						int16_t v_max,
						int16_t v_min,
						int16_t a_max
						)
{
	// pwm channel sanity check
	if (pwm_ch < 1 || pwm_ch > 4)
		return false;

	// store configuration
	_enc_pos = enc_pos;
	_pwm_ch = pwm_ch;
	_reverse = reverse;
	_en_pin = en_pin;

	// set the speed limits
	setMaxVelocity(v_max);
	setMinVelocity(v_min);
	setMaxAcceleration(a_max);

	// setup appropriate timer for pwm based on pwm channel & freq
	// store interrupt state register and then disable them for atomic 16-bit writes
	uint8_t sreg = SREG;
	noInterrupts();

	switch (_pwm_ch) {
#if defined(__AVR_ATmega328P__)
	case 1:
		// timer 1 
		pinMode(9, OUTPUT); // OC1A
		pinMode(10, OUTPUT); // OC1B

		// set ICR1 to obtain PWM_FREQ [kHz] operation
		ICR1 = FS_PWM_ICR;

		// ensure pwm output is zero
		OCR1A = 0;
		OCR1B = 0;

		// setup timer 1 to phase/freq correct PWM using OC1A & OC1B
		TCCR1A = 0b10100010;
		TCCR1B = 0b00010001;

		break;
#elif defined(__AVR_ATmega2560__)
	case 1:
		// timer 1 
		pinMode(11, OUTPUT); // OC1A
		pinMode(12, OUTPUT); // OC1B

		// set ICR1 to obtain PWM_FREQ [kHz] operation
		ICR1 = FS_PWM_ICR;

		// ensure pwm output is zero
		OCR1A = 0;
		OCR1B = 0;

		// setup timer 1 to phase/freq correct PWM using OC1A & OC1B
		TCCR1A = 0b10100010;
		TCCR1B = 0b00010001;

		break;

	case 2:
		// timer 4
		pinMode(6, OUTPUT); // OC4A
		pinMode(7, OUTPUT); // OC4B

		// set ICR3 to obtain PWM_FREQ [kHz] operation
		ICR4 = FS_PWM_ICR;

		// ensure pwm output is zero
		OCR4A = 0;
		OCR4B = 0;

		// setup timer 4 to phase/freq correct PWM using OC4A & OC4B
		TCCR4A = 0b10100010;
		TCCR4B = 0b00010001;

		break;

	case 3:
		// timer 3
		pinMode(2, OUTPUT); // OC3B
		pinMode(3, OUTPUT); // OC3C

		// set ICR3 to obtain PWM_FREQ [kHz] operation
		ICR3 = FS_PWM_ICR;

		// ensure pwm output is zero
		OCR3B = 0;
		OCR3C = 0;

		// setup timer 3 to phase/freq correct PWM using OC3B & OC3C
		TCCR3A = 0b00101010;
		TCCR3B = 0b00010001;

		break;

	case 4:
		// timer 5
		pinMode(44, OUTPUT); // OC5C
		pinMode(45, OUTPUT); // OC5B

		// set ICR5 to obtain PWM_FREQ [kHz] operation
		ICR5 = FS_PWM_ICR;

		// ensure pwm throttle is zero
		OCR5C = 0;
		OCR5B = 0;

		// setup timer 5 to phase/freq correct PWM using OC5B & OC5C
		TCCR5A = 0b00101010;
		TCCR5B = 0b00010001;

		break;
#endif
	}
	
	// restore interrupts
	SREG = sreg;

	// setup pins
	pinMode(_en_pin, OUTPUT);

	// raise the init flag
	_init = _init | 0b00000001;

	// start disabled
	disable();
}

void FastServo::tick(uint32_t t_now)
{
	// disable interrupts
	uint8_t sreg = SREG;
	noInterrupts();

	// update position
	_s = *_enc_pos;

	// restore interrupts
	SREG = sreg;


	// emergency stop manoeuvre
	if (_state == FS_STOPPING)
	{
		// set current position as the setpoint
		_s_target = _s;

		// once we come to a stop
		if (atTarget())
		{
			// hold still
			_state = FS_ENABLED;
		}
	}

	if (_state >= FS_ENABLED)
	{
		// are we due to run our control loop?
		if (t_now - _t_prev > FS_LATENCY)
		{
			// benchmark control loop
			//uint32_t _t_start = micros();

			// update velocity with ds
			_updateVelocity(_s - _s_prev); // 52
			
			// store position
			_s_prev = _s;

			// update our velocity setpoint based on the state of our servo
			_updateTargetVelocity(); // 40
			
			// update throttle 
			_updateThrottle(); // 200

			// update motor
			_updateMotor(); // 8

			// store timestamp
			_t_prev = t_now;

			// benchmark control loop
			//_lag = micros() - _t_start;
		}
	}
}

bool FastServo::atTarget()
{
	if ( abs(_s_target - _s) <= _s_deadzone && _v == 0)
	{
		return true;
	}
	return false;
}

bool FastServo::throttleError()
{
	return _throttle_error;
}

void FastServo::moveTo(int16_t s)
{
		_s_target = s;
}

void FastServo::setPos(int16_t s)
{
	_s = s;
	_s_prev = s; // avoid crazy ds
	
	// disable interrupts
	uint8_t sreg = SREG;
	noInterrupts();

	// update encoder position
	*_enc_pos = s;

	// restore interrupts
	SREG = sreg;
}

void FastServo::disable()
{
	_state = FS_DISABLED;

	// disable motor h-bridge
	digitalWrite(_en_pin, 0);

	// zero throttle
	_setPWM(0, 0);
}

void FastServo::enable()
{
	_state = FS_ENABLED;

	_reset();

	// enable motor h-bridge
	digitalWrite(_en_pin, 1);
}

void FastServo::stop()
{
	// jump on the brakes mate
	_state = FS_STOPPING;
}

uint32_t FastServo::getLag()
{
	return _lag;
}

int32_t FastServo::getPos()
{
	return _s;
}

int32_t FastServo::getTargetPos()
{
	return _s_target;
}

int16_t FastServo::getVelocity()
{
	return _v;
}

int16_t FastServo::getTargetVelocity()
{
	return _v_target;
}

int16_t FastServo::getIdealVelocity()
{
	return _v_ideal;
}

float FastServo::getVelocityError()
{
	return _v_ut;
}

float FastServo::getP()
{
	return _vP;
}

float FastServo::getI()
{
	return _vI;
}

float FastServo::getD()
{
	return _vD;
}

int16_t FastServo::getThrottle()
{
	return _thr;
}

void FastServo::setPositionDeadzone(uint8_t s)
{
	_s_deadzone = s;
}

void FastServo::setConstants(float Kp, float Ki, float Kd, float Ki_decay, float KvFF)
{
	_k_vP = Kp;
	_k_vI = Ki;
	_k_vD = Kd;
	_k_vI_decay = Ki_decay; // integral term rate of decay
	_k_vFF = KvFF; // velocity feedforward
}

void FastServo::setMaxVelocity(int16_t v_max)
{
	if (v_max > 0) {
		// update max velocity
		_v_max = v_max;


		// calculate stopping distance 
		_s_stop = _calcStop(_v_max, _a_max);
	}
}

void FastServo::setMinVelocity(int16_t v_min)
{
	if (v_min >= 0) {
		_v_min = v_min;
	}
}

void FastServo::setMaxAcceleration(int16_t a_max)
{
	if (a_max > 0) {

		// update max acceleration
		_a_max = a_max;

		// acceleration per control loop
		_a_inc = _a_max * (FS_LATENCY / float(1000));

		// stopping distance
		_s_stop = _calcStop(_v_max, _a_max);
	}
}

void FastServo::setThrottleDeadzone(int8_t deadzone)
{
	if (deadzone < 0 || deadzone > 100)
		return;

	_thr_deadzone = (FS_PWM_ICR * deadzone) / 100;
}

void FastServo::setMaxThrottleDuty(int8_t duty)
{
	if (duty < 0 || duty > 100)
		return;

	_thr_max = FS_PWM_ICR * ((float)duty / 100);
}

void FastServo::_reset()
{
	// disable interrupts
	uint8_t sreg = SREG;
	noInterrupts();

	// grab position
	_s = *_enc_pos;

	// restore interrupts
	SREG = sreg;

	// reset position and velocity setpoints
	_s_target = _s;
	_v_target = 0;

	// clear previous position
	_s_prev = _s;

	// reset velocity buffer
	_v = 0;
	_ds_i = 0;
	_ds_sum = 0;
	for (uint8_t i = 0; i < DS_BUFFER_SIZE; i++)
	{
		_ds_buffer[i] = 0;
	}

	// reset velocity error derivative buffer
	_v_ut = 0;
	_v_ut_prev = 0;
	_vd_i = 0;
	_vd_sum = 0;
	for (uint8_t i = 0; i < VD_BUFFER_SIZE; i++)
	{
		_vd_buffer[i] = 0;
	}

	// clear PID terms
	_vP = 0;
	_vI = 0;
	_vD = 0;

	// zero throttle
	_thr = 0;
	_setPWM(0, 0);

	// clear throttle error
	_throttle_error = false;
}

void FastServo::_updateVelocity(int16_t ds)
{
	// remove the old value from the sum
	_ds_sum -= _ds_buffer[_ds_i];

	// add the new value to the sum
	_ds_sum += ds;

	// store the value in the buffer, overwriting the old value
	_ds_buffer[_ds_i] = ds;

	// advance the index
	_ds_i++;
	_ds_i %= DS_BUFFER_SIZE;

	// calculate and store velocity
	_v = (_ds_sum * 1000) / (FS_LATENCY * DS_BUFFER_SIZE);
}

void FastServo::_updateTargetVelocity()
{
	// _s			= current position
	// _s_target	= desired position

	// _v			= current velocity
	// _v_max		= maximum velocity
	// _a_max		= desired acceleration

	if (atTarget())
	{
		_v_target = 0;
	}
	else {
		// if we're trying to stop 
		if (_state == FS_STOPPING) {
			// then ideal speed is 0
			_v_ideal = 0;

		}
		else {
			// positional error is positive if we need to move clockwise
			int32_t _s_ut = _s_target - _s;

			// start with full speed
			_v_ideal = (_s_ut > 0) ? _v_max : -_v_max;

			// if we're within the stopping distance 
			if (abs(_s_ut) <= _s_stop)
			{
				_v_ideal = ((_s_ut * (_v_max - _v_min)) / _s_stop) + ((_s_ut > 0) ? _v_min : -_v_min);
			}
		}

		// which way are we going?
		if (_v_target < _v_ideal)
		{
			_v_target += _a_inc;
			if (_v_target > _v_ideal) _v_target = _v_ideal; // clamp
		}
		else if (_v_target > _v_ideal) {
			_v_target -= _a_inc;
			if (_v_target < _v_ideal) _v_target = _v_ideal; // clamp
		}
	}
}

void FastServo::_updateThrottle()
{
	float pid_thr = 0;

	// calculate velocity error - positive error means we need to increase speed
	_v_ut = _v_target - _v;

	// jammed / reverse detection
	// if magnitude of error is larger than or equal to requested 
	if (abs(_v_ut) >= 0.9*abs(_v_target) && abs(_v_target) > 30)
	{
		// if magnitude of error is larger than or equal to requested 
		// have we run out of strikes?
		if (_v_errors < FS_MAX_V_ERRORS)
		{
			_v_errors++;
		}
		else {
			// jammed / reverse condition detected
			_throttle_error = true;

			// disable!
			disable();
		}
	}
	else if(_v_errors > 0) {
		_v_errors--;
	}

	// PID time!

	// calculate proportional term
	// will take a positive error and generate a positive feedback to increase speed
	_vP = _k_vP * _v_ut;

	// calculate integral term
	// will accumulate a positive feedback term over time in response to a positive error
	_vI += _k_vI * _v_ut;

	// clamp to the remaining space
	if ( _vI > _thr_max/4) _vI = _thr_max/4;
	if ( _vI < -_thr_max/4) _vI = -_thr_max/4;

	// reset I term if we're at rest at our target
	if (atTarget())
	{
		// decay _vI term
		_vI += _k_vI_decay * -_vI;

		// clamp to zero once below a significant value (throttle is an integer)
		if (abs(_vI) < 0.5)
			_vI = 0;
	}

	// derivative term - don't bother if D term is 0
	if (_k_vD != 0)
	{
		// remove the old value from the sum
		_vd_sum -= _vd_buffer[_vd_i];

		// store the value in the buffer, overwriting the old value
		_vd_buffer[_vd_i] = _v_ut - _v_ut_prev;

		_v_ut_prev = _v_ut;

		// add the new value to the sum
		_vd_sum += _vd_buffer[_vd_i];

		// advance the index
		_vd_i++;
		_vd_i %= VD_BUFFER_SIZE;

		// calculate and store velocity
		_vD = (_k_vD * _vd_sum) / VD_BUFFER_SIZE;
	}

	// calculate throttle output as the summation of feedforward, P, I and D terms
	pid_thr = (_v_target * _k_vFF) + _vP + _vI +_vD;

	// clamp pid_thr between -PWM_ICR and PWM_ICR accounting for throttle deadzone
	if (pid_thr <= -_thr_max + _thr_deadzone) pid_thr = -_thr_max + _thr_deadzone;
	if (pid_thr >= _thr_max - _thr_deadzone) pid_thr = _thr_max - _thr_deadzone;

	// map pid_thr to area above and below deadzone
	if (pid_thr > 0)
	{
		pid_thr = pid_thr + _thr_deadzone;
		_thr = (int16_t)pid_thr + 0.5;
	} else if (pid_thr < 0)
	{
		pid_thr = pid_thr - _thr_deadzone;
		_thr = (int16_t)pid_thr - 0.5;
	}
	else {
		_thr = 0;
	}
}

void FastServo::_updateMotor()
{
	// which direction are we going?
	if (_thr > 0) {
		// run motor anti-clockwise
		if (!_reverse)
		{
			_setPWM(_thr, 0);
		} else {
			_setPWM(0, _thr);
		}	
	}
	else if (_thr < 0) {
		// run motor clockwise
		if (!_reverse)
		{
			_setPWM(0, abs(_thr));
		} else {
			_setPWM(abs(_thr), 0);
		}
	}
	else {
		// zero throttle
		_setPWM(0, 0);
	}
}

void FastServo::_setPWM(uint16_t thr_a, uint16_t thr_b)
{

	// store the current interrupt state register
	uint8_t sreg = SREG;

	// disable interrupts incase it was enabled for 16-bit writes to OCRnA/OCRnB/OCRnC etc
	noInterrupts();

	switch (_pwm_ch) {
	case 1: // timer 1
		OCR1A = thr_a;
		OCR1B = thr_b;
		break;
#if defined(__AVR_ATmega2560__)
	case 2: // timer 4
		OCR4A = thr_a;
		OCR4B = thr_b;
		break;

	case 3: // timer 3
		OCR3B = thr_a;
		OCR3C = thr_b;
		break;

	case 4: // timer 5
		OCR5B = thr_a;
		OCR5C = thr_b;
		break;
#endif
	}

	// restore interrupts
	SREG = sreg;
}

int16_t FastServo::_calcStop(int16_t v, int16_t a)
{
	return ((int32_t)v*v) / (2 * a);
}

