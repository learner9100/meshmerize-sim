#pragma once

// xorshift32: tiny, and the same seed gives the same maze on every machine
class Rng{
    private:
        unsigned int s_;

    public:
        explicit Rng(unsigned int seed = 1){ reseed(seed); }

        void reseed(unsigned int seed){ s_ = seed ? seed : 0x9E3779B9u; }   // xorshift dies at 0

        unsigned int next(){
            unsigned int x = s_;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            s_ = x;
            return x;
        }

        int below(int n){ return (int)(next() % (unsigned int)n); }
};
