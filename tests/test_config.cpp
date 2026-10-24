#include <cstdio>
#include "robot_model.h"

static int failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%s: %s\n", name, ok ? "OK" : "FAIL");
    if (!ok) failures++;
}

static bool writeFile(const char* path, const char* text){
    FILE* f = std::fopen(path, "w");
    if(!f) return false;
    std::fputs(text, f);
    std::fclose(f);
    return true;
}

int main() {
    const char* tmp = "build/_robot_test.cfg";

    {
        RobotConfig cfg;
        check("shipped robot.cfg parses", loadRobotConfig("robot.cfg", cfg) == 0);
        check("shipped robot.cfg matches the built-in placeholders",
              cfg.dry.mm_per_s == dryRunModel().mm_per_s && cfg.dry.node_ms == dryRunModel().node_ms
              && cfg.run.mm_per_s == actualRunModel().mm_per_s && cfg.run.uturn_ms == actualRunModel().uturn_ms
              && !cfg.use_odometry);
    }

    {
        writeFile(tmp,
            "# measured on 2026-11-01\n"
            "dry.mm_per_s = 620\n"
            "  run.turn_ms=150   # tight turns\n"
            "odom.enabled = 1\n"
            "odom.scale_err = 0.035\n"
            "odom.merge_mm = 80\n"
            "\n");
        RobotConfig cfg;
        check("custom file parses", loadRobotConfig(tmp, cfg) == 0);
        check("values override the defaults", cfg.dry.mm_per_s == 620 && cfg.run.turn_ms == 150);
        check("untouched values keep the defaults", cfg.dry.node_ms == 300 && cfg.run.mm_per_s == 900);
        check("odometry switches on with its parameters",
              cfg.use_odometry && cfg.odom.scale_err > 0.0349 && cfg.odom.scale_err < 0.0351 && cfg.merge_mm == 80);
    }

    {
        writeFile(tmp, "dry.mm_per_s = 500\nwheel.diameter = 32\n");
        RobotConfig cfg;
        check("unknown key reports its line number", loadRobotConfig(tmp, cfg) == 2);

        writeFile(tmp, "dry.mm_per_s 500\n");
        check("missing '=' reports line 1", loadRobotConfig(tmp, cfg) == 1);

        writeFile(tmp, "dry.mm_per_s = 0\n");
        check("zero speed is refused", loadRobotConfig(tmp, cfg) == -2);

        check("missing file gives -1", loadRobotConfig("build/does_not_exist.cfg", cfg) == -1);
    }

    std::remove(tmp);
    return failures;
}
