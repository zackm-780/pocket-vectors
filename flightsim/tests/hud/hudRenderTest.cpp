// Hud::draw against a fake GL: one batched draw call, declutter levels change the amount of
// geometry, warnings and messages add geometry, an invalid state draws nothing and the H3 overlay
// hook is called. No GL context needed.
#include <doctest/doctest.h>

#include <cstring>

#include "core/MessageQueue.h"
#include "core/Units.h"
#include "gfx/Camera.h"
#include "hud/Hud.h"

using namespace fsim;
using namespace fsim::gfx;

namespace {

struct HudFakeGl {
    static inline int drawArrays = 0;
    GlApi api;
    HudFakeGl() {
        drawArrays = 0;
        api.Viewport = [](GLint, GLint, GLsizei, GLsizei) {};
        api.Enable = [](GLenum) {};
        api.Disable = [](GLenum) {};
        api.BlendFunc = [](GLenum, GLenum) {};
        api.GenBuffers = [](GLsizei n, GLuint* b) {
            for (GLsizei i = 0; i < n; ++i) b[i] = 7;
        };
        api.DeleteBuffers = [](GLsizei, const GLuint*) {};
        api.BindBuffer = [](GLenum, GLuint) {};
        api.BufferData = [](GLenum, GLsizeiptr, const void*, GLenum) {};
        api.CreateShader = [](GLenum) -> GLuint { return 1; };
        api.ShaderSource = [](GLuint, GLsizei, const GLchar* const*, const GLint*) {};
        api.CompileShader = [](GLuint) {};
        api.GetShaderiv = [](GLuint, GLenum, GLint* p) { *p = 1; };
        api.GetShaderInfoLog = [](GLuint, GLsizei, GLsizei*, GLchar*) {};
        api.DeleteShader = [](GLuint) {};
        api.CreateProgram = []() -> GLuint { return 3; };
        api.AttachShader = [](GLuint, GLuint) {};
        api.BindAttribLocation = [](GLuint, GLuint, const GLchar*) {};
        api.LinkProgram = [](GLuint) {};
        api.GetProgramiv = [](GLuint, GLenum, GLint* p) { *p = 1; };
        api.GetProgramInfoLog = [](GLuint, GLsizei, GLsizei*, GLchar*) {};
        api.UseProgram = [](GLuint) {};
        api.DeleteProgram = [](GLuint) {};
        api.GetUniformLocation = [](GLuint, const GLchar*) -> GLint { return 0; };
        api.Uniform4f = [](GLint, GLfloat, GLfloat, GLfloat, GLfloat) {};
        api.EnableVertexAttribArray = [](GLuint) {};
        api.DisableVertexAttribArray = [](GLuint) {};
        api.VertexAttribPointer = [](GLuint, GLint, GLenum, GLboolean, GLsizei, const void*) {};
        api.DrawArrays = [](GLenum, GLint, GLsizei) { ++drawArrays; };
        api.DrawElements = [](GLenum, GLsizei, GLenum, const void*) {};
    }
};

struct Scene {
    HudFakeGl gl;
    Renderer2D r2d{gl.api, GlFlavor::Desktop21};
    Camera cam;
    AircraftState s;
    ControlInputs c;
    AircraftProfile p;
    MessageQueue msgs;
    HudOptions opt;
    Hud hud;

    Scene() {
        r2d.init();
        s.valid = true;
        s.latDeg = 40;
        s.lonDeg = -100;
        s.altMslFt = 3000;
        s.altAglFt = 1000;
        s.pitchRad = 0.05f;
        s.headingTrueRad = 1.5f;
        s.iasKt = 110;
        s.velEastMps = 55;
        s.engineCount = 1;
        s.engines[0].running = true;
        s.engines[0].rpm = 2300;
        s.gearPosNorm = 1;
        p.engineCount = 1;
        const char* labels[] = {"UP", "10", "20", "30"};
        for (int i = 0; i < 4; ++i) {
            FlapDetent d;
            d.cmdNorm = i / 3.0f;
            std::strncpy(d.label, labels[i], sizeof d.label - 1);
            p.flapDetents.push_back(d);
        }
        p.vs0Kt = 40;
        p.vs1Kt = 48;
        p.vfeKt = 85;
        p.vneKt = 163;
        cam.setLens(55.0f);
        cam.setViewport(640, 480);
        cam.setPoseFromAircraft(s);
    }

    int vertices() {
        r2d.begin(640, 480);
        hud.draw(r2d, cam, s, c, p, msgs, opt);
        const int n = r2d.pendingVertexCount();
        r2d.end();
        return n;
    }
};

struct CountingOverlay : HudOverlay {
    int calls = 0;
    float headingMagDeg = -1;
    void draw(const HudFrame& f) override {
        ++calls;
        headingMagDeg = f.headingMagDeg;
    }
};

}  // namespace

TEST_CASE("hud: a frame is one batched draw call") {
    Scene sc;
    sc.r2d.begin(640, 480);
    sc.hud.draw(sc.r2d, sc.cam, sc.s, sc.c, sc.p, sc.msgs, sc.opt);
    sc.r2d.end();
    CHECK(sc.r2d.stats().drawCalls == 1);
    CHECK(sc.r2d.stats().triangles > 500);  // a real HUD, not an empty batch
    // The 4k line-quad budget of the perf table (PLAN) is about 8k triangles.
    CHECK(sc.r2d.stats().triangles < 8000);
}

TEST_CASE("hud: invalid aircraft state draws nothing") {
    Scene sc;
    sc.s.valid = false;
    CHECK(sc.vertices() == 0);
}

TEST_CASE("hud: declutter levels reduce the geometry FULL > STD > MIN") {
    Scene sc;
    sc.opt.declutter = Declutter::Full;
    const int full = sc.vertices();
    sc.opt.declutter = Declutter::Std;
    const int std_ = sc.vertices();
    sc.opt.declutter = Declutter::Min;
    const int min = sc.vertices();
    CHECK(full > std_);
    CHECK(std_ > min);
    CHECK(min > 0);  // attitude, tapes and FPV stay
}

TEST_CASE("hud: warnings and fresh messages add geometry, old messages do not") {
    Scene sc;
    const int base = sc.vertices();

    sc.s.stallWarning = true;
    sc.msgs.tick(1.0);
    const int stall = sc.vertices();  // flashing: first frame of the warning is visible
    CHECK(stall > base);
    sc.s.stallWarning = false;

    CHECK(sc.vertices() == base);
    sc.msgs.post("FLAPS 20");
    CHECK(sc.vertices() > base);
    sc.msgs.tick(2.1);  // older than the 2 s life
    CHECK(sc.vertices() == base);

    // Only the newest three show.
    sc.msgs.post("X");
    sc.msgs.post("X");
    sc.msgs.post("X");
    const int three = sc.vertices();
    sc.msgs.post("X");
    CHECK(sc.vertices() == three);  // the fourth message is not drawn
}

TEST_CASE("hud: paused flag shows PAUSED even when MIN declutter hides the status rows") {
    Scene sc;
    sc.opt.declutter = Declutter::Min;
    const int base = sc.vertices();
    sc.opt.paused = true;
    CHECK(sc.vertices() > base);
}

TEST_CASE("hud: H3 overlay is called once per frame with the magnetic heading") {
    Scene sc;
    CountingOverlay overlay;
    sc.hud.setOverlay(&overlay);
    sc.opt.magVarDeg = 10.0f;
    sc.vertices();
    CHECK(overlay.calls == 1);
    CHECK(overlay.headingMagDeg == doctest::Approx(radToDeg(1.5) - 10.0).epsilon(1e-3));
}

TEST_CASE("hud: wide screens and both HUD colors draw without fault") {
    Scene sc;
    sc.cam.setViewport(1280, 720);
    sc.cam.setPoseFromAircraft(sc.s);
    sc.opt.color = HudPalette::Amber;
    sc.r2d.begin(1280, 720);
    sc.hud.draw(sc.r2d, sc.cam, sc.s, sc.c, sc.p, sc.msgs, sc.opt);
    sc.r2d.end();
    CHECK(sc.r2d.stats().drawCalls == 1);
    // Every vertex stays on the canvas (plus a little for outlines) at 16:9.
    for (int i = 0; i < sc.r2d.batchVertexCount(); ++i) {
        const auto& v = sc.r2d.batchData()[i];
        CHECK(v.x > -40);
        CHECK(v.x < sc.r2d.virtualWidth() + 40);
    }
}
