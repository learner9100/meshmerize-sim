#pragma once
#include <Arduino.h>
#include "config.h"

// TB6612FNG: one PWM pin plus two direction pins per motor, STBY high to enable.
// Uses the Arduino-ESP32 3.x LEDC API (ledcAttach / ledcWrite by pin).
class Motors{
    private:
        static void drive(int pwm_pin, int in1, int in2, int speed, bool reversed){
            if(reversed) speed = -speed;
            if(speed > PWM_MAX) speed = PWM_MAX;
            if(speed < -PWM_MAX) speed = -PWM_MAX;
            digitalWrite(in1, speed > 0 ? HIGH : LOW);
            digitalWrite(in2, speed < 0 ? HIGH : LOW);
            ledcWrite(pwm_pin, speed < 0 ? -speed : speed);
        }

    public:
        void begin(){
            pinMode(M_LEFT_IN1, OUTPUT);  pinMode(M_LEFT_IN2, OUTPUT);
            pinMode(M_RIGHT_IN1, OUTPUT); pinMode(M_RIGHT_IN2, OUTPUT);
            pinMode(M_STBY, OUTPUT);
            ledcAttach(M_LEFT_PWM, PWM_FREQ_HZ, PWM_BITS);
            ledcAttach(M_RIGHT_PWM, PWM_FREQ_HZ, PWM_BITS);
            digitalWrite(M_STBY, HIGH);
            stop();
        }

        // -PWM_MAX .. PWM_MAX each
        void set(int left, int right){
            drive(M_LEFT_PWM, M_LEFT_IN1, M_LEFT_IN2, left, M_LEFT_REVERSED);
            drive(M_RIGHT_PWM, M_RIGHT_IN1, M_RIGHT_IN2, right, M_RIGHT_REVERSED);
        }

        void stop(){ set(0, 0); }

        void brake(){
            digitalWrite(M_LEFT_IN1, HIGH);  digitalWrite(M_LEFT_IN2, HIGH);
            digitalWrite(M_RIGHT_IN1, HIGH); digitalWrite(M_RIGHT_IN2, HIGH);
            ledcWrite(M_LEFT_PWM, PWM_MAX);
            ledcWrite(M_RIGHT_PWM, PWM_MAX);
        }
};

// Quadrature encoders, counted on every edge of channel A.
namespace enc{
    volatile long left_count = 0;
    volatile long right_count = 0;

    void IRAM_ATTR onLeft(){
        const bool a = digitalRead(E_LEFT_A);
        const bool b = digitalRead(E_LEFT_B);
        left_count += ((a == b) != E_LEFT_REVERSED) ? 1 : -1;
    }
    void IRAM_ATTR onRight(){
        const bool a = digitalRead(E_RIGHT_A);
        const bool b = digitalRead(E_RIGHT_B);
        right_count += ((a == b) != E_RIGHT_REVERSED) ? -1 : 1;
    }
}

class Encoders{
    public:
        void begin(){
            pinMode(E_LEFT_A, INPUT_PULLUP);  pinMode(E_LEFT_B, INPUT_PULLUP);
            pinMode(E_RIGHT_A, INPUT_PULLUP); pinMode(E_RIGHT_B, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(E_LEFT_A), enc::onLeft, CHANGE);
            attachInterrupt(digitalPinToInterrupt(E_RIGHT_A), enc::onRight, CHANGE);
            reset();
        }

        long left() const { return enc::left_count; }
        long right() const { return enc::right_count; }

        void reset(){
            noInterrupts();
            enc::left_count = 0;
            enc::right_count = 0;
            interrupts();
        }

        static float mmPerCount(){
            return WHEEL_DIAMETER_MM * PI / (COUNTS_PER_WHEEL_REV * 2);   // x2 because we count both edges of A
        }

        float leftMm() const { return left() * mmPerCount(); }
        float rightMm() const { return right() * mmPerCount(); }
        float mm() const { return (leftMm() + rightMm()) / 2; }
};
