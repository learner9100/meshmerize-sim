CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O2 -Icore -Isim
BUILD    := build
TESTS    := $(patsubst tests/%.cpp,$(BUILD)/%,$(wildcard tests/test_*.cpp))
.RECIPEPREFIX = >

.PHONY: test clean

test: $(TESTS)
> @for t in $(TESTS); do ./$$t || exit 1; done

$(BUILD)/%: tests/%.cpp $(wildcard core/*.h) | $(BUILD)
> $(CXX) $(CXXFLAGS) $< -o $@

$(BUILD):
> mkdir -p $(BUILD)

clean:
> rm -rf $(BUILD)