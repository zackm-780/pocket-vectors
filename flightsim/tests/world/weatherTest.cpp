// X1: METAR parser, wind profile, gust statistics, QNH -> indicated altitude, scenario keys.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <string>

#include "app/Scenario.h"
#include "core/Baro.h"
#include "core/Units.h"
#include "world/Weather.h"
#include "world/WeatherMetar.h"

using namespace fsim;
using namespace fsim::world;

namespace {

// Absolute-tolerance comparison (doctest 2.4 Approx has no margin).
struct Near {
    double value, tol;
    Near(double v, double t) : value(v), tol(t) {}
    friend bool operator==(double a, const Near& n) { return std::fabs(a - n.value) <= n.tol; }
    friend bool operator==(const Near& n, double a) { return a == n; }
    friend std::ostream& operator<<(std::ostream& os, const Near& n) {
        return os << n.value << " +-" << n.tol;
    }
};

Metar parse(const std::string& text) {
    Metar m;
    REQUIRE_MESSAGE(parseMetar(text, m), text);
    return m;
}

}  // namespace

TEST_CASE("weather: METAR KPDX style, gusts, temperature and altimeter") {
    const Metar m = parse(
        "METAR KPDX 201653Z 02012G22KT 10SM FEW040 SCT250 18/09 A3004 RMK AO2 SLP170 T01830094");
    CHECK(m.station == "KPDX");
    CHECK(m.day == 20);
    CHECK(m.hour == 16);
    CHECK(m.minute == 53);
    CHECK(m.hasWind);
    CHECK_FALSE(m.windVariable);
    CHECK(m.windFromTrueDeg == doctest::Approx(20.0f));
    CHECK(m.windKt == doctest::Approx(12.0f));
    CHECK(m.gustKt == doctest::Approx(22.0f));
    CHECK(m.visibilityM == doctest::Approx(10 * 1609.344f).epsilon(0.001));
    CHECK_FALSE(m.visibilityAtLeast);
    REQUIRE(m.clouds.size() == 2);
    CHECK(m.clouds[0].cover == MetarCloud::Cover::Few);
    CHECK(m.clouds[0].baseAglFt == doctest::Approx(4000.0f));
    CHECK(m.ceilingFt < 0);
    CHECK(m.tempC == doctest::Approx(18.0f));
    CHECK(m.dewPointC == doctest::Approx(9.0f));
    CHECK(m.altimeterHpa == doctest::Approx(30.04f * 33.8638867f).epsilon(0.0005));
}

TEST_CASE("weather: METAR KOTH style with ceiling, mist, fraction visibility") {
    const Metar m =
        parse("KOTH 201656Z AUTO 27005KT 1 1/2SM BR OVC004 11/10 A2998 RMK AO2 SLP158 T01060100");
    CHECK(m.station == "KOTH");
    CHECK(m.windFromTrueDeg == doctest::Approx(270.0f));
    CHECK(m.windKt == doctest::Approx(5.0f));
    CHECK(m.gustKt == 0.0f);
    CHECK(m.visibilityM == doctest::Approx(1.5f * 1609.344f).epsilon(0.001));
    CHECK(m.ceilingFt == doctest::Approx(400.0f));
    REQUIRE(m.clouds.size() == 1);
    CHECK(m.clouds[0].cover == MetarCloud::Cover::Overcast);
    CHECK(m.tempC == doctest::Approx(11.0f));
}

TEST_CASE("weather: METAR variable wind, 1/4 mile, vertical visibility") {
    const Metar m = parse("KXYZ 010000Z VRB03KT 1/4SM FG VV002 05/05 A3012");
    CHECK(m.windVariable);
    CHECK(m.windKt == doctest::Approx(3.0f));
    CHECK(m.visibilityM == doctest::Approx(0.25f * 1609.344f).epsilon(0.001));
    CHECK(m.ceilingFt == doctest::Approx(200.0f));
    CHECK(m.clouds[0].cover == MetarCloud::Cover::VerticalVis);
}

TEST_CASE("weather: METAR with variable direction range and sub-1/4 mile (M1/4SM)") {
    const Metar m =
        parse("KABC 151255Z 18008KT 150V210 M1/4SM R28L/1200FT FZFG VV001 M02/M03 A2985");
    CHECK(m.windFromTrueDeg == doctest::Approx(180.0f));
    CHECK(m.visibilityM < 0.25f * 1609.344f + 1.0f);
    CHECK(m.tempC == doctest::Approx(-2.0f));
    CHECK(m.dewPointC == doctest::Approx(-3.0f));
    CHECK(m.altimeterHpa == doctest::Approx(29.85f * 33.8638867f).epsilon(0.0005));
}

TEST_CASE("weather: ICAO style METAR with 9999, Q1013, MPS wind and negative temperatures") {
    const Metar m = parse("EGLL 211020Z 24015G27KT 9999 SCT025 BKN100 M01/M04 Q1013 NOSIG");
    CHECK(m.windKt == doctest::Approx(15.0f));
    CHECK(m.gustKt == doctest::Approx(27.0f));
    CHECK(m.visibilityM == doctest::Approx(10000.0f));
    CHECK(m.visibilityAtLeast);
    CHECK(m.ceilingFt == doctest::Approx(10000.0f));
    CHECK(m.tempC == doctest::Approx(-1.0f));
    CHECK(m.altimeterHpa == doctest::Approx(1013.0f));

    const Metar mps = parse("ULLI 211030Z 25006MPS 4000 BKN010 M10/M12 Q1031");
    CHECK(mps.windKt == doctest::Approx(6.0f * 1.94384f).epsilon(0.001));
    CHECK(mps.visibilityM == doctest::Approx(4000.0f));
    CHECK(mps.altimeterHpa == doctest::Approx(1031.0f));
}

TEST_CASE("weather: CAVOK, A2992 and P6SM") {
    const Metar c = parse("LSZH 211050Z 05004KT CAVOK 22/10 Q1018");
    CHECK(c.cavok);
    CHECK(c.visibilityAtLeast);
    CHECK(c.ceilingFt < 0);
    CHECK(c.altimeterHpa == doctest::Approx(1018.0f));

    const Metar p = parse("KSEA 211053Z 16004KT P6SM CLR 25/12 A2992");
    CHECK(p.visibilityAtLeast);
    CHECK(p.clouds.empty());
    CHECK(p.altimeterHpa == Near(1013.21f, 0.1f));
}

TEST_CASE("weather: METAR with missing fields, calm wind and three digit gusts") {
    const Metar calm = parse("KBLI 211053Z 00000KT 10SM CLR 12/ A3001");
    CHECK(calm.hasWind);
    CHECK(calm.windKt == 0.0f);
    CHECK(calm.hasTemp);
    CHECK_FALSE(calm.hasDewPoint);

    const Metar noTemp = parse("KEUG 211053Z 33010KT 10SM A3000");
    CHECK_FALSE(noTemp.hasTemp);
    CHECK(noTemp.hasAltimeter);

    const Metar noAlt = parse("KEUG 211053Z 33010KT 10SM FEW020 15/10");
    CHECK_FALSE(noAlt.hasAltimeter);

    const Metar strong = parse("KDEN 211053Z 27035G105KT 3SM TS BKN050CB 20/05 A2990");
    CHECK(strong.gustKt == doctest::Approx(105.0f));
    CHECK(strong.ceilingFt == doctest::Approx(5000.0f));
}

TEST_CASE("weather: METAR from a file-style blob with trailing newline and extra lines") {
    const Metar m =
        parse("\nMETAR KPDX 201753Z 36010KT 10SM CLR 20/08 A3010 RMK AO2\n\nKOTH ...\n");
    CHECK(m.station == "KPDX");
    CHECK(m.windFromTrueDeg == Near(0.0f, 0.01f));  // 360 is stored as 0
}

TEST_CASE("weather: garbage is rejected") {
    Metar m;
    CHECK_FALSE(parseMetar("", m));
    CHECK_FALSE(parseMetar("<html>404</html>", m));
    CHECK_FALSE(parseMetar("KPDX no data", m));
}

TEST_CASE("weather: wind profile veers and increases with height") {
    WeatherSetup setup;
    setup.windFromTrueDeg = 270.0f;
    setup.windKt = 10.0f;
    Weather w;
    w.setConditions(resolveWeather(setup, {}, nullptr, 0.0f));
    float from = 0, kt = 0;
    w.meanWindAt(0, from, kt);
    CHECK(from == doctest::Approx(270.0f));
    CHECK(kt == doctest::Approx(10.0f));
    w.meanWindAt(1000, from, kt);
    CHECK(from > 270.0f);
    CHECK(from < 270.0f + Weather::kDefaultVeerDeg);
    CHECK(kt > 10.0f);
    w.meanWindAt(8000, from, kt);
    CHECK(from == doctest::Approx(270.0f + Weather::kDefaultVeerDeg));
    CHECK(kt == doctest::Approx(10.0f * Weather::kDefaultAloftFactor));

    // an explicit aloft wind across north takes the short way round
    setup.windFromTrueDeg = 350.0f;
    setup.aloftFromTrueDeg = 10.0f;
    setup.aloftKt = 30.0f;
    w.setConditions(resolveWeather(setup, {}, nullptr, 0.0f));
    w.meanWindAt(1000, from, kt);
    CHECK(from == Near(0.0f, 0.01f));
    CHECK(kt == doctest::Approx(20.0f));
}

TEST_CASE("weather: gust statistics are bounded, centered near the set wind and deterministic") {
    WeatherSetup setup;
    setup.windFromTrueDeg = 200.0f;
    setup.windKt = 12.0f;
    setup.gustKt = 22.0f;
    setup.seed = 7;
    auto run = [&](std::uint32_t seed, double& meanKt, double& minKt, double& maxKt) {
        setup.seed = seed;
        Weather w;
        w.setConditions(resolveWeather(setup, {}, nullptr, 0.0f));
        w.reset(0);
        double sum = 0;
        minKt = 1e9;
        maxKt = 0;
        const int n = 600 * 120;  // ten minutes at 120 Hz
        for (int i = 0; i < n; ++i) {
            const WindNedFps v = w.step(1.0 / 120.0, 0);
            const double kt = fpsToKt(std::hypot(v.northFps, v.eastFps));
            sum += kt;
            minKt = std::min(minKt, kt);
            maxKt = std::max(maxKt, kt);
        }
        meanKt = sum / n;
    };
    double mean = 0, lo = 0, hi = 0;
    run(7, mean, lo, hi);
    MESSAGE("gust run: mean ", mean, " kt, min ", lo, ", max ", hi);
    CHECK(mean == doctest::Approx(12.0).epsilon(0.04));  // within about 0.5 kt of the set wind
    CHECK(hi <= 22.0 + 0.01);
    CHECK(hi >= 17.0);  // real peaks happen
    CHECK(lo >= 12.0 - 0.6 * 10.0);
    CHECK(lo < 12.0);

    double mean2 = 0, lo2 = 0, hi2 = 0;
    run(7, mean2, lo2, hi2);
    CHECK(mean2 == mean);  // same seed, same wind
    CHECK(hi2 == hi);
    run(8, mean2, lo2, hi2);
    CHECK(mean2 != mean);  // different seed, different gusts
}

TEST_CASE("weather: gusts change smoothly") {
    WeatherSetup setup;
    setup.windFromTrueDeg = 90.0f;
    setup.windKt = 15.0f;
    setup.gustKt = 30.0f;
    setup.turbulence = 1.0f;
    Weather w;
    w.setConditions(resolveWeather(setup, {}, nullptr, 0.0f));
    w.reset(0);
    WindNedFps prev = w.step(1.0 / 120.0, 0);
    double maxJumpFps = 0;
    for (int i = 0; i < 60 * 120; ++i) {
        const WindNedFps v = w.step(1.0 / 120.0, 0);
        maxJumpFps =
            std::max({maxJumpFps, std::fabs(v.northFps - prev.northFps),
                      std::fabs(v.eastFps - prev.eastFps), std::fabs(v.downFps - prev.downFps)});
        prev = v;
    }
    CHECK(maxJumpFps < 0.6);  // ft/s per 1/120 s step
}

TEST_CASE("weather: turbulence is off by default and zero mean when on") {
    WeatherSetup calm;
    calm.windKt = 0.0f;
    Weather w;
    w.setConditions(resolveWeather(calm, {}, nullptr, 0.0f));
    w.reset(0);
    for (int i = 0; i < 1200; ++i) {
        const WindNedFps v = w.step(1.0 / 120.0, 0);
        CHECK(v.northFps == 0.0);
        CHECK(v.downFps == 0.0);
    }
    calm.turbulence = 0.5f;
    w.setConditions(resolveWeather(calm, {}, nullptr, 0.0f));
    w.reset(0);
    double sumD = 0, maxD = 0;
    const int n = 600 * 120;
    for (int i = 0; i < n; ++i) {
        const WindNedFps v = w.step(1.0 / 120.0, 0);
        sumD += v.downFps;
        maxD = std::max(maxD, std::fabs(v.downFps));
    }
    CHECK(std::fabs(sumD / n) < 0.8);
    CHECK(maxD > 0.5);
    CHECK(maxD <= 3.0 + 1e-6);
}

TEST_CASE("weather: sources are combined, scenario wins, METAR fills the rest") {
    const Metar m = parse("KPDX 201653Z 02012G22KT 4SM FEW040 18/09 A3004");
    WeatherSetup none;
    WeatherConditions c = resolveWeather(none, {}, &m, 0.0f);
    CHECK(c.windKt == doctest::Approx(12.0f));
    CHECK(c.gustKt == doctest::Approx(22.0f));
    CHECK(c.hasVisibility);
    CHECK(c.visibilityM == doctest::Approx(4 * 1609.344f).epsilon(0.001));
    CHECK(c.qnhHpa == Near(1017.2f, 0.2f));
    CHECK(c.altimeterSettingHpa == doctest::Approx(c.qnhHpa));
    CHECK(c.source == "metar KPDX");

    WeatherSetup sc;
    sc.windFromTrueDeg = 270.0f;
    sc.windKt = 5.0f;
    c = resolveWeather(sc, {}, &m, 0.0f);
    CHECK(c.windKt == doctest::Approx(5.0f));
    CHECK(c.gustKt == 0.0f);  // the METAR gust is not mixed into a scenario wind
    CHECK(c.visibilityM == doctest::Approx(4 * 1609.344f).epsilon(0.001));  // METAR fills this
    CHECK(c.source == "scenario+metar KPDX");

    WeatherOptions off;
    off.useMetar = false;
    c = resolveWeather(none, off, &m, 0.0f);
    CHECK(c.windKt == 0.0f);
    CHECK_FALSE(c.hasVisibility);
    CHECK(c.source == "default");

    sc.metar = false;
    c = resolveWeather(sc, {}, &m, 0.0f);
    CHECK_FALSE(c.hasVisibility);

    WeatherOptions nog;
    nog.gusts = false;
    c = resolveWeather(none, nog, &m, 0.0f);
    CHECK(c.gustKt == 0.0f);

    const Metar vrb = parse("KXYZ 010000Z VRB03KT 9999 05/05 A3012");
    c = resolveWeather(none, {}, &vrb, 123.0f);
    CHECK(c.windFromTrueDeg == doctest::Approx(123.0f));
    CHECK(fogVisibilityM(c) >= 25000.0f);  // 9999 means clear
}

TEST_CASE("weather: QNH to indicated altitude") {
    // set to the real QNH: indicated = true, whatever the pressure
    for (double qnh : {980.0, 1013.25, 1040.0})
        for (double alt : {0.0, 1500.0, 12000.0})
            CHECK(indicatedAltFt(alt, qnh, qnh) == Near(alt, 1e-6));
    // standard setting, high pressure day: the altimeter reads high ("high to low, look out below")
    const double high = indicatedAltFt(1000.0, 1033.25, kStdQnhHpa);
    CHECK(high < 1000.0);  // QNH higher than the setting -> true altitude above indicated...
    // ... the rule: setting 1 hPa too low reads about 27 ft too low
    CHECK(indicatedAltFt(0.0, 1014.25, 1013.25) == Near(-27.0, 1.5));
    CHECK(indicatedAltFt(0.0, 1013.25, 1014.25) == Near(27.0, 1.5));
    // 1 inHg is roughly 1000 ft (910 ft near sea level)
    const double perInHg = indicatedAltFt(0.0, 1013.25, 1013.25 + kHpaPerInHg);
    CHECK(perInHg > 880.0);
    CHECK(perInHg < 1050.0);
    // A2992 -> 1013.2 hPa, A3012 -> 1020 hPa
    CHECK(inHgToHpa(29.92) == Near(1013.2, 0.1));
    CHECK(hpaToInHg(1013.25) == Near(29.92, 0.01));
    // pressure altitude round trip
    CHECK(pressureAltitudeFt(staticPressureHpa(5000.0, 1013.25), 1013.25) == Near(5000.0, 1e-6));
    CHECK(staticPressureHpa(0.0, 1013.25) == doctest::Approx(1013.25));
    CHECK(staticPressureHpa(5000.0, 1013.25) == Near(843.1, 0.5));
}

TEST_CASE("weather: ISA deviation") {
    CHECK(isaDeviationC(15.0f, 0.0f) == doctest::Approx(0.0f));
    CHECK(isaDeviationC(9.0f, 3000.0f) == Near(0.0f, 0.1f));
}

TEST_CASE("weather: scenario [weather] keys, legacy [wind] and visibility units") {
    Scenario s;
    std::string err;
    REQUIRE(
        parseScenario("[scenario]\naircraft = c172p\n[wind]\nfrom_true_deg = 180\nspeed_kt = 10\n"
                      "[weather]\ngust_kt = 20\nvisibility_km = 3\nqnh_inhg = 29.92\n"
                      "turbulence = 0.3\nseed = 5\nmetar = off\n",
                      s, err));
    REQUIRE(s.weather.windFromTrueDeg);
    CHECK(*s.weather.windFromTrueDeg == 180.0f);
    CHECK(*s.weather.windKt == 10.0f);
    CHECK(*s.weather.gustKt == 20.0f);
    CHECK(*s.weather.visibilityM == doctest::Approx(3000.0f));
    CHECK(*s.weather.qnhHpa == Near(1013.2f, 0.1f));
    CHECK(*s.weather.turbulence == doctest::Approx(0.3f));
    CHECK(s.weather.seed == 5u);
    CHECK(s.weather.metar.has_value());
    CHECK_FALSE(*s.weather.metar);
    CHECK(s.windSpeedKt == 10.0f);

    Scenario plain;
    REQUIRE(parseScenario("[scenario]\naircraft = c172p\n", plain, err));
    CHECK_FALSE(plain.weather.windKt.has_value());
    CHECK_FALSE(plain.weather.visibilityM.has_value());
}

TEST_CASE("weather: loadMetarFor reads <dir>/<ICAO>.txt, also K + 3 letter ident") {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "fsim_x1_weather_test";
    fs::create_directories(dir);
    {
        std::ofstream(dir / "KPDX.txt")
            << "METAR KPDX 201753Z 00000KT 10SM FEW250 14/07 A3003 RMK AO2 SLP168 T01390067 $\n";
    }
    Metar m;
    CHECK(loadMetarFor(dir.string(), "KPDX", m));
    CHECK(m.station == "KPDX");
    CHECK(loadMetarFor(dir.string(), "pdx", m));
    CHECK_FALSE(loadMetarFor(dir.string(), "KOTH", m));
    CHECK_FALSE(loadMetarFor(dir.string(), "", m));
    fs::remove_all(dir);
}
