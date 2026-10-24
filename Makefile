CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O2 -Icore -Isim -Iexamples
BUILD    := build
HEADERS  := $(wildcard core/*.h sim/*.h examples/*.h)
TESTS    := $(patsubst tests/%.cpp,$(BUILD)/%,$(wildcard tests/test_*.cpp))
TOOLS    := $(patsubst tools/%.cpp,$(BUILD)/%,$(wildcard tools/*.cpp))
.RECIPEPREFIX = >

# knobs for `make show` and `make compare`
SEED  ?= 1
W     ?= 10
H     ?= 10
LOOPS ?= 3
CPS   ?= 0
SEEDS ?= 500
STRAT ?=
ROBOT ?=
CSV   ?=
EXTRA ?=

.PHONY: all test tools show compare clean

all: test tools

test: $(TESTS)
> @for t in $(TESTS); do ./$$t || exit 1; done

tools: $(TOOLS)

show: $(BUILD)/show
> @./$(BUILD)/show --seed $(SEED) --size $(W) $(H) --loops $(LOOPS) --checkpoints $(CPS) \
>   $(if $(STRAT),--strategy $(STRAT),) $(if $(ROBOT),--robot $(ROBOT),) $(EXTRA)

compare: $(BUILD)/compare
> @./$(BUILD)/compare --seeds $(SEEDS) --size $(W) $(H) --loops $(LOOPS) --checkpoints $(CPS) \
>   $(if $(ROBOT),--robot $(ROBOT),) $(if $(CSV),--csv $(CSV),) $(EXTRA)

$(BUILD)/%: tests/%.cpp $(HEADERS) | $(BUILD)
> $(CXX) $(CXXFLAGS) $< -o $@

$(BUILD)/%: tools/%.cpp $(HEADERS) | $(BUILD)
> $(CXX) $(CXXFLAGS) $< -o $@

$(BUILD):
> mkdir -p $(BUILD)

clean:
> rm -rf $(BUILD)