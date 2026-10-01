/* Offline x86 test of rat_vst.cpp (test.sh builds it with ASan/UBSan): the circuit's gain,
 * clipping and filter (sine in, harmonics out), the diode types' levels, the Ruetz mod's bass
 * cut, MIX 0 = dry, stereo independence, chunk restore, NaN/denormal-free output.
 * With "bench" as the second argument it only measures CPU load. Prints PASSED/FAILED. */
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <dlfcn.h>

struct AEffect;
typedef intptr_t (*audioMasterCallback)(AEffect *, int32_t, int32_t, intptr_t, void *, float);
struct AEffect {
    int32_t magic;
    intptr_t (*dispatcher)(AEffect *, int32_t, int32_t, intptr_t, void *, float);
    void (*process)(AEffect *, float **, float **, int32_t);
    void (*setParameter)(AEffect *, int32_t, float);
    float (*getParameter)(AEffect *, int32_t);
    int32_t numPrograms, numParams, numInputs, numOutputs, flags;
    intptr_t resvd1, resvd2;
    int32_t initialDelay, realQualities, offQualities;
    float ioRatio;
    void *object, *user;
    int32_t uniqueID, version;
    void (*processReplacing)(AEffect *, float **, float **, int32_t);
    void (*processDoubleReplacing)(AEffect *, double **, double **, int32_t);
    char future[56];
};
static int automated = 0;
static intptr_t master(AEffect *, int32_t op, int32_t, intptr_t, void *, float) { if (op == 0) automated++; return 0; }
static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)
static const float SR = 44100;
static const int BS = 256;
static bool bad = false;

static int param(AEffect *e, const char *name) {
    char buf[64];
    for (int i = 0; i < e->numParams; i++) { buf[0] = 0; e->dispatcher(e, 8, i, 0, buf, 0); if (!std::strcmp(buf, name)) return i; }
    std::printf("FAIL: no parameter %s\n", name); fails++; return 0;
}
static std::string display(AEffect *e, const char *name) { char buf[64] = {0}; e->dispatcher(e, 7, param(e, name), 0, buf, 0); return buf; }
static void set(AEffect *e, const char *name, int v) {
    struct R { const char *n; int lo, hi; };
    static const R r[] = {{"Distortion", 0, 100}, {"Filter", 0, 100}, {"Volume", 0, 100}, {"Diodes", 0, 3}, {"Ruetz", 0, 1}, {"Input", -12, 12}, {"Mix", 0, 100}};
    for (auto &x : r) if (!std::strcmp(x.n, name)) { e->setParameter(e, param(e, name), (float)(v - x.lo) / (x.hi - x.lo)); return; }
}
/* a sine through the effect; returns the right channel's output (left gets silence unless both) */
static std::vector<float> run(AEffect *e, double f, double amp, double sec, std::vector<float> *left = nullptr, bool leftSilent = false) {
    std::vector<float> outR, il(BS), ir(BS), ol(BS), orr(BS);
    float *in[2] = {il.data(), ir.data()}, *out[2] = {ol.data(), orr.data()};
    static double ph = 0;
    for (int b = 0; b < (int)(sec * SR / BS); b++) {
        for (int i = 0; i < BS; i++) { float s = (float)(amp * std::sin(ph)); ph += 2 * M_PI * f / SR; il[i] = leftSilent ? 0 : s; ir[i] = s; }
        e->processReplacing(e, in, out, BS);
        for (int i = 0; i < BS; i++) {
            for (float s : {ol[i], orr[i]}) if (!std::isfinite(s) || std::fpclassify(s) == FP_SUBNORMAL) bad = true;
            outR.push_back(orr[i]);
            if (left) left->push_back(ol[i]);
        }
    }
    return outR;
}
static double goertzel(const std::vector<float> &a, double f, double t0) {
    size_t i0 = (size_t)(t0 * SR), i1 = a.size();
    double w = 2 * M_PI * f / SR, c = std::cos(w), s1 = 0, s2 = 0;
    for (size_t i = i0; i < i1; i++) { double s0 = a[i] + 2 * c * s1 - s2; s2 = s1; s1 = s0; }
    return 2 * std::sqrt(std::max(0.0, s1 * s1 + s2 * s2 - 2 * c * s1 * s2)) / (i1 - i0);
}
static double rms(const std::vector<float> &a, double t0 = 0.2) {
    double s = 0; size_t i0 = (size_t)(t0 * SR);
    for (size_t i = i0; i < a.size(); i++) s += (double)a[i] * a[i];
    return std::sqrt(s / (a.size() - i0));
}
/* total harmonic distortion of f0 (harmonics 2..10) */
static double thd(const std::vector<float> &a, double f0) {
    double h1 = goertzel(a, f0, 0.2), hs = 0;
    for (int k = 2; k <= 10; k++) { double h = goertzel(a, f0 * k, 0.2); hs += h * h; }
    return std::sqrt(hs) / std::max(h1, 1e-12);
}
static double db(double x) { return 20 * std::log10(x + 1e-12); }

int main(int argc, char **argv) {
    void *h = dlopen(argc > 1 ? argv[1] : "./rat.so", RTLD_NOW | RTLD_LOCAL);
    if (!h) { std::printf("FAILED: dlopen %s\n", dlerror()); return 1; }
    auto mainf = (AEffect * (*)(audioMasterCallback)) dlsym(h, "VSTPluginMain");
    AEffect *e = mainf(master);
    CHECK(e->numInputs == 2 && e->numOutputs == 2, "not a stereo effect");
    e->dispatcher(e, 0, 0, 0, nullptr, 0);
    e->dispatcher(e, 10, 0, 0, nullptr, SR);
    if (argc > 2 && !std::strcmp(argv[2], "bench")) {
        set(e, "Distortion", 100);
        auto t0 = std::chrono::steady_clock::now();
        run(e, 110, 0.3, 10.0);
        double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("bench (this CPU, one core): stereo, 4x oversampled: %.1f %%\n", 10.0 * s);
        return 0;
    }
    std::printf("defaults: %s / %s / %s, %s\n", display(e, "Distortion").c_str(), display(e, "Filter").c_str(),
                display(e, "Volume").c_str(), display(e, "Diodes").c_str());

    /* 1. distortion: THD climbs with the knob; at 0 it is nearly clean */
    set(e, "Filter", 0); set(e, "Volume", 70);
    double t[3], lv[3];
    int dk[3] = {0, 40, 100};
    for (int k = 0; k < 3; k++) {
        set(e, "Distortion", dk[k]);
        auto a = run(e, 220, 0.1, 1.0);
        t[k] = thd(a, 220); lv[k] = rms(a);
        std::printf("  distortion %3d: THD %6.1f %%, out %.1f dBFS\n", dk[k], 100 * t[k], db(lv[k]));
    }
    CHECK(t[0] < 0.03, "distortion 0 is not clean (%.1f %%)", 100 * t[0]);
    CHECK(t[1] > 0.2 && t[2] > t[1], "distortion does not rise");

    /* 2. FILTER darkens: the 9th harmonic falls */
    set(e, "Distortion", 70);
    double hi[2];
    for (int k = 0; k < 2; k++) {
        set(e, "Filter", k ? 100 : 0);
        auto a = run(e, 220, 0.2, 1.0);
        hi[k] = goertzel(a, 9 * 220, 0.2) / goertzel(a, 220, 0.2);
    }
    std::printf("  filter: 9th harmonic %.1f dB (open) vs %.1f dB (closed)\n", db(hi[0]), db(hi[1]));
    CHECK(hi[1] < hi[0] * 0.35, "FILTER does not darken");   /* one pole: ~ -12 dB at 2 kHz */

    /* 3. diodes: all four at a similar level, LED cleaner than silicon at the same drive */
    set(e, "Filter", 30); set(e, "Distortion", 60);
    double dl[4], dt[4];
    const char *dn[4] = {"RAT", "TURBO", "GE", "OP-AMP"};
    for (int d = 0; d < 4; d++) {
        set(e, "Diodes", d);
        auto a = run(e, 220, 0.1, 1.0);
        dl[d] = rms(a); dt[d] = thd(a, 220);
        std::printf("  diodes %-6s: out %.1f dBFS, THD %.1f %%\n", dn[d], db(dl[d]), 100 * dt[d]);
    }
    for (int d = 1; d < 4; d++) CHECK(std::fabs(db(dl[d] / dl[0])) < 8, "%s level %.1f dB off RAT", dn[d], db(dl[d] / dl[0]));
    set(e, "Diodes", 0);

    /* 4. Ruetz: less bass into the clipper (a quiet 80 Hz tone at low drive) */
    set(e, "Distortion", 50); set(e, "Filter", 0);
    double rb[2];
    for (int k = 0; k < 2; k++) {
        set(e, "Ruetz", k);
        auto a = run(e, 80, 0.002, 1.0);
        rb[k] = goertzel(a, 80, 0.3);
    }
    std::printf("  ruetz: 80 Hz %.1f dB off, %.1f dB on\n", db(rb[0]), db(rb[1]));
    CHECK(rb[1] < rb[0] * 0.6, "RUETZ does not cut bass");
    set(e, "Ruetz", 0);

    /* 5. MIX 0 is the dry signal, exactly; stereo channels are independent */
    set(e, "Mix", 0);
    {
        auto a = run(e, 330, 0.25, 0.5);
        double err = 0; static double ph2 = 0; (void)ph2;
        double r = rms(a, 0.1);
        err = std::fabs(r - 0.25 / std::sqrt(2.0));
        std::printf("  mix 0: rms %.4f (dry %.4f)\n", r, 0.25 / std::sqrt(2.0));
        CHECK(err < 0.002, "MIX 0 is not dry");
    }
    set(e, "Mix", 100);
    {
        std::vector<float> L;
        auto R = run(e, 220, 0.2, 0.6, &L, true);
        std::printf("  stereo: left (silent in) rms %.6f, right rms %.4f\n", rms(L, 0.3), rms(R, 0.3));
        CHECK(rms(L, 0.3) < 1e-4 && rms(R, 0.3) > 0.01, "channels not independent");
    }

    /* 6. loud input stays below 0 dBFS */
    set(e, "Diodes", 1); set(e, "Volume", 100); set(e, "Input", 12);
    {
        auto a = run(e, 110, 1.0, 0.5);
        double pk = 0; for (float s : a) pk = std::max(pk, (double)std::fabs(s));
        std::printf("  hot: peak %.3f\n", pk);
        CHECK(pk < 1.0, "output above 0 dBFS");
    }

    /* 7. chunk */
    set(e, "Diodes", 2); set(e, "Ruetz", 1); set(e, "Distortion", 83); set(e, "Input", -4);
    void *chunk = nullptr;
    intptr_t len = e->dispatcher(e, 23, 0, 0, &chunk, 0);
    std::vector<uint8_t> copy((uint8_t *)chunk, (uint8_t *)chunk + len);
    AEffect *e2 = mainf(master);
    e2->dispatcher(e2, 10, 0, 0, nullptr, SR);
    e2->dispatcher(e2, 24, 0, (intptr_t)copy.size(), copy.data(), 0);
    for (const char *k : {"Diodes", "Ruetz", "Distortion", "Input"})
        CHECK(display(e2, k) == display(e, k), "chunk: %s %s vs %s", k, display(e2, k).c_str(), display(e, k).c_str());
    std::printf("  chunk %ld bytes: %s %s %s %s\n", (long)len, display(e2, "Diodes").c_str(), display(e2, "Ruetz").c_str(),
                display(e2, "Distortion").c_str(), display(e2, "Input").c_str());
    e2->dispatcher(e2, 1, 0, 0, nullptr, 0);

    CHECK(!bad, "NaN, Inf or denormals in the output");
    e->dispatcher(e, 1, 0, 0, nullptr, 0);
    std::printf(fails ? "FAILED (%d)\n" : "PASSED\n", fails);
    return fails ? 1 : 0;
}
