#pragma once
#include <cstring>
#include <cstdlib>
#include <cstdio>

// Just enough command line parsing for the tools: --flag, --flag value,
// --flag v1 v2. Unknown flags are reported by the caller via check().
struct Args{
    int argc;
    char** argv;

    Args(int c, char** v) : argc(c), argv(v) {}

    int find(const char* flag) const {
        for(int i=1;i<argc;i++) if(!std::strcmp(argv[i], flag)) return i;
        return -1;
    }

    bool has(const char* flag) const { return find(flag) >= 0; }

    int intAfter(const char* flag, int fallback, int offset = 1) const {
        const int i = find(flag);
        if(i < 0 || i + offset >= argc) return fallback;
        return std::atoi(argv[i + offset]);
    }

    double doubleAfter(const char* flag, double fallback) const {
        const int i = find(flag);
        if(i < 0 || i + 1 >= argc) return fallback;
        return std::atof(argv[i + 1]);
    }

    const char* strAfter(const char* flag, const char* fallback) const {
        const int i = find(flag);
        if(i < 0 || i + 1 >= argc) return fallback;
        return argv[i + 1];
    }

    // Every argv that starts with "--" must be one of `known`.
    bool onlyKnown(const char* const* known, int n) const {
        bool ok = true;
        for(int i=1;i<argc;i++){
            if(std::strncmp(argv[i], "--", 2)) continue;
            bool found = false;
            for(int k=0;k<n;k++) if(!std::strcmp(argv[i], known[k])) found = true;
            if(!found){ std::fprintf(stderr, "unknown option: %s\n", argv[i]); ok = false; }
        }
        return ok;
    }
};
