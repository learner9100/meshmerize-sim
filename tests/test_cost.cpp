#include <cstdio>
#include "cost.h"
#include "rng.h"

static int failures=0;

static void check(const char* name, bool ok){
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if(!ok) failures++;
}

int main(){
    check("turn: same direction is straight", turnKind(1, 0, 1, 0) == Turn::Straight);
    check("turn: same direction, different length, still straight", turnKind(230, 0, 690, 0) == Turn::Straight);
    check("turn: right angle is a corner", turnKind(1, 0, 0, 1) == Turn::Corner);
    check("turn: left angle is a corner", turnKind(0, -1, 1, 0) == Turn::Corner);
    check("turn: reverse is a u-turn", turnKind(0, 1, 0, -1) == Turn::UTurn);
    check("turn: no incoming direction counts as straight", turnKind(0, 0, 1, 0) == Turn::Straight);
    check("turn: no outgoing direction counts as straight", turnKind(1, 0, 0, 0) == Turn::Straight);

    CostModel m;
    m.mm_per_s = 500;
    m.node_ms = 300;
    m.turn_ms = 400;
    m.uturn_ms = 1000;
    check("cost: 1000 mm straight = 2000 + 300", moveCostMs(m, 1000, Turn::Straight) == 2300);
    check("cost: corner adds 400", moveCostMs(m, 1000, Turn::Corner) == 2700);
    check("cost: u-turn adds 1000", moveCostMs(m, 1000, Turn::UTurn) == 3300);
    check("cost: 230 mm at 900 mm/s rounds down to 255 ms", moveCostMs(CostModel{900, 0, 0, 0}, 230, Turn::Straight) == 255);

    check("isqrt: perfect squares", isqrt(0) == 0 && isqrt(1) == 1 && isqrt(144) == 12 && isqrt(1000000) == 1000);
    check("isqrt: rounds down", isqrt(2) == 1 && isqrt(99) == 9 && isqrt(1999999) == 1414);
    check("isqrt: negative is 0", isqrt(-5) == 0);

    Node a, b;
    a.x_mm = 0;    a.y_mm = 0;
    b.x_mm = 3000; b.y_mm = 4000;
    check("straight: 3-4-5", straightMm(a, b) == 5000 && straightMm(b, a) == 5000);
    check("manhattan: 3000 + 4000", manhattanMm(a, b) == 7000 && manhattanMm(b, a) == 7000);
    check("straight never exceeds manhattan", straightMm(a, b) <= manhattanMm(a, b));

    Rng r1(42), r2(42);
    bool same = true;
    for(int i=0;i<1000;i++) if(r1.next() != r2.next()) same = false;
    check("rng: same seed, same sequence", same);

    Rng r3(43);
    check("rng: different seed, different first value", Rng(42).next() != r3.next());

    Rng z(0);
    unsigned int first = z.next();
    check("rng: seed 0 does not get stuck", first != 0 && z.next() != first);

    Rng b1(7);
    bool in_range = true;
    int hits[5] = {0,0,0,0,0};
    for(int i=0;i<5000;i++){
        int v = b1.below(5);
        if(v < 0 || v >= 5) in_range = false; else hits[v]++;
    }
    bool all_hit = true;
    for(int i=0;i<5;i++) if(hits[i] < 700) all_hit = false;
    check("rng: below(5) stays in range", in_range);
    check("rng: below(5) spreads over every value", all_hit);

    return failures;
}
