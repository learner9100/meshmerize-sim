#pragma once

// xorshift32. Tiny, portable, and the same seed gives the same numbers on
// every machine, which is what makes a batch run reproducible.
class Rng{
    private:
        unsigned int s_;

    public:
        explicit Rng(unsigned int seed = 1){ reseed(seed); }

        // xorshift gets stuck at zero forever, so seed 0 is mapped elsewhere.
        void reseed(unsigned int seed){ s_ = seed ? seed : 0x9E3779B9u; }

        unsigned int next(){
            unsigned int x = s_;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            s_ = x;
            return x;
        }

        // uniform in [0, n). n must be > 0.
        int below(int n){ return (int)(next() % (unsigned int)n); }
};
