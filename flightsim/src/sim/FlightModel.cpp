#include "sim/FlightModel.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <vector>

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <initialization/FGTrim.h>
#include <input_output/FGLog.h>
#include <input_output/FGPropertyManager.h>
#include <models/FGAircraft.h>
#include <models/FGGroundReactions.h>
#include <models/FGLGear.h>
#include <models/FGMassBalance.h>
#include <models/FGPropulsion.h>
#include <models/FGPropagate.h>
#include <models/atmosphere/FGWinds.h>
#include <models/propulsion/FGEngine.h>

#include "core/Geo.h"
#include "core/Ini.h"
#include "core/Log.h"
#include "core/Units.h"
#include "sim/CrashRules.h"

namespace fsim {
namespace {

constexpr double kGraceAfterStartS = 1.0;       // no crash verdicts right after a start
constexpr float kStallWarnHysteresisDeg = 1.0f;  // warning clears this far below its threshold
constexpr double kTerrainSlopeProbeM = 60.0;     // distance ahead used to estimate the slope

// Routes JSBSim's log output into ours; banner and info chatter go to debug level.
class JsbsimLogger : public JSBSim::FGLogger {
public:
    void Message(const std::string& message) override { buffer_ += message; }
    void Flush() override {
        while (!buffer_.empty() && (buffer_.back() == '\n' || buffer_.back() == '\r'))
            buffer_.pop_back();
        if (!buffer_.empty()) {
            switch (log_level) {
            case JSBSim::LogLevel::WARN: LOG_WARN("jsbsim: %s", buffer_.c_str()); break;
            case JSBSim::LogLevel::ERROR:
            case JSBSim::LogLevel::FATAL: LOG_ERROR("jsbsim: %s", buffer_.c_str()); break;
            default: LOG_DEBUG("jsbsim: %s", buffer_.c_str()); break;
            }
        }
        buffer_.clear();
    }

private:
    std::string buffer_;
};

// Cached property node: no string lookups in the per-step path.
struct Prop {
    SGPropertyNode* node = nullptr;
    bool valid() const { return node != nullptr; }
    double get(double def = 0.0) const { return node ? node->getDoubleValue() : def; }
    void set(double value) const {
        if (node) node->setDoubleValue(value);
    }
};

float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

std::string replaceAll(std::string s, const std::string& from, const std::string& to) {
    for (size_t pos = s.find(from); pos != std::string::npos; pos = s.find(from, pos + to.size()))
        s.replace(pos, from.size(), to);
    return s;
}

}  // namespace

struct FlightModel::Impl {
    explicit Impl(std::string root) : jsbsimRoot(std::move(root)) {}

    bool start(const AircraftProfile& profile, const StartCondition& sc, AircraftState& state,
               ControlInputs& startControls);
    void applyControls(const ControlInputs& in, const AircraftState& state);
    void refresh(AircraftState& state);
    Prop prop(const std::string& path) const {
        return Prop{fdm->GetPropertyManager()->GetNode(path, false)};
    }
    // Property path for `key`: the profile [properties] override, else `def`; "{n}" is replaced
    // by the engine index.
    std::string pathFor(const char* key, const char* def, int engine = 0) const {
        const auto it = profile.properties.find(key);
        const std::string& base = (it != profile.properties.end() && !it->second.empty())
                                      ? it->second
                                      : std::string(def);
        return replaceAll(base, "{n}", std::to_string(engine));
    }
    double propNumber(const char* key, double def) const {
        const auto it = profile.properties.find(key);
        if (it == profile.properties.end() || it->second.empty()) return def;
        char* end = nullptr;
        const double v = std::strtod(it->second.c_str(), &end);
        return end == it->second.c_str() ? def : v;
    }
    Prop pathProp(const char* key, const char* def, int engine = 0) const {
        return prop(pathFor(key, def, engine));
    }
    void bindProps();
    bool trim();
    float terrainSlopeAlongTrack(const AircraftState& s) const;

    std::string jsbsimRoot;
    AircraftProfile profile;
    StartCondition startCondition;
    bool haveStart = false;
    std::unique_ptr<JSBSim::FGFDMExec> fdm;
    double stepDtS = kSimDtS;
    const TerrainQuery* terrain = nullptr;

    // cached nodes
    Prop pLat, pLon, pAlt, pAgl, pTerrain, pPhi, pTheta, pPsi, pP, pQ, pR;
    Prop pVc, pVt, pMach, pAlpha, pBeta, pHdot, pNz, pVg, pVn, pVe, pVd;
    Prop pWindN, pWindE, pGustN, pGustE, pFlap, pGear, pSpeedbrake, pPitchTrim, pRollTrim;
    Prop pYawTrim, pFuel;
    Prop pLBrake, pRBrake, pCBrake;
    Prop cElevator, cAileron, cRudder, cFlap, cGear, cSpeedbrake, cSpoiler, cSteer;
    struct EngineProps {
        Prop throttleCmd, mixtureCmd, running, n1, n2, rpm, thrust, reverser;
        bool turbine = false;
    };
    EngineProps eng[kMaxEngines];
    int engineCount = 0;

    struct GearInfo {
        std::shared_ptr<JSBSim::FGLGear> unit;
        bool structure = false;       // non-wheel contact point (wing tip, tail, belly...)
        bool retractable = false;
        const char* reason = nullptr;  // crash reason when a structure point touches
    };
    std::vector<GearInfo> gears;
    float gearHeightFt = 0;  // CG height above the wheel contact points
    CrashLimits limits;

    // reverser state: latched on the ground at idle, position slewed over its travel time
    bool reverserLatched = false;
    float reverserPos = 0;  // 0 stowed .. 1 deployed
    double reverserOff = 0, reverserOn = kPi, reverserTravelS = 1.5;

    // ground steering authority fades with ground speed
    float steerFullKt = 15, steerMinKt = 60, steerMinFrac = 0.15f;

    // crash detection / bookkeeping
    float eyeBodyM[3] = {0, 0, 0};
    bool crashed = false;
    char crashReason[sizeof(AircraftState::crashReason)] = {0};
    double touchdownWindowEndS = -1;
    bool stallWarnLatched = false;
    int flapDetent = 0;
    bool parkingBrake = false;
    bool ready = false;

    void declareCrash(AircraftState& s, const char* reason, float vsFpm);
};

void FlightModel::Impl::bindProps() {
    pLat = prop("position/lat-geod-deg");
    pLon = prop("position/long-gc-deg");
    pAlt = prop("position/h-sl-ft");
    pAgl = prop("position/h-agl-ft");
    pTerrain = prop("position/terrain-elevation-asl-ft");
    pPhi = prop("attitude/phi-rad");
    pTheta = prop("attitude/theta-rad");
    pPsi = prop("attitude/psi-rad");
    pP = prop("velocities/p-rad_sec");
    pQ = prop("velocities/q-rad_sec");
    pR = prop("velocities/r-rad_sec");
    pVc = prop("velocities/vc-kts");
    pVt = prop("velocities/vtrue-kts");
    pMach = prop("velocities/mach");
    pAlpha = prop("aero/alpha-deg");
    pBeta = prop("aero/beta-deg");
    pHdot = prop("velocities/h-dot-fps");
    pNz = prop("accelerations/Nz");
    pVg = prop("velocities/vg-fps");
    pVn = prop("velocities/v-north-fps");
    pVe = prop("velocities/v-east-fps");
    pVd = prop("velocities/v-down-fps");
    pWindN = prop("atmosphere/wind-north-fps");
    pWindE = prop("atmosphere/wind-east-fps");
    pGustN = prop("atmosphere/gust-north-fps");
    pGustE = prop("atmosphere/gust-east-fps");
    pFlap = pathProp("flap_pos", "fcs/flap-pos-norm");
    pGear = pathProp("gear_pos", "gear/gear-pos-norm");
    pSpeedbrake = pathProp("speedbrake_pos", "fcs/speedbrake-pos-norm");
    pPitchTrim = pathProp("pitch_trim", "fcs/pitch-trim-cmd-norm");
    pRollTrim = pathProp("roll_trim", "fcs/roll-trim-cmd-norm");
    pYawTrim = pathProp("yaw_trim", "fcs/yaw-trim-cmd-norm");
    pFuel = pathProp("fuel_total", "propulsion/total-fuel-lbs");
    pLBrake = pathProp("brake_left", "fcs/left-brake-cmd-norm");
    pRBrake = pathProp("brake_right", "fcs/right-brake-cmd-norm");
    pCBrake = pathProp("brake_center", "fcs/center-brake-cmd-norm");
    cElevator = prop("fcs/elevator-cmd-norm");
    cAileron = prop("fcs/aileron-cmd-norm");
    cRudder = prop("fcs/rudder-cmd-norm");
    cFlap = pathProp("flap_cmd", "fcs/flap-cmd-norm");
    cGear = pathProp("gear_cmd", "gear/gear-cmd-norm");
    cSpeedbrake = pathProp("speedbrake_cmd", "fcs/speedbrake-cmd-norm");
    cSteer = pathProp("steer_cmd", "fcs/steer-cmd-norm");
    cSpoiler = Prop{};
    if (profile.properties.count("spoiler_cmd")) cSpoiler = pathProp("spoiler_cmd", "");

    auto propulsion = fdm->GetPropulsion();
    const int modelEngines = static_cast<int>(propulsion->GetNumEngines());
    engineCount = std::min({profile.engineCount, kMaxEngines, modelEngines});
    if (engineCount != profile.engineCount)
        LOG_WARN("profile %s lists %d engines but the model has %d", profile.id.c_str(),
                 profile.engineCount, modelEngines);
    for (int i = 0; i < engineCount; ++i) {
        EngineProps& e = eng[i];
        e = EngineProps{};
        e.throttleCmd = pathProp("throttle_cmd", "fcs/throttle-cmd-norm[{n}]", i);
        e.mixtureCmd = pathProp("mixture_cmd", "fcs/mixture-cmd-norm[{n}]", i);
        e.running = pathProp("engine_running", "propulsion/engine[{n}]/set-running", i);
        e.n1 = pathProp("engine_n1", "propulsion/engine[{n}]/n1", i);
        e.n2 = pathProp("engine_n2", "propulsion/engine[{n}]/n2", i);
        e.rpm = pathProp("engine_rpm", "propulsion/engine[{n}]/engine-rpm", i);
        if (!e.rpm.valid()) e.rpm = prop("propulsion/engine[" + std::to_string(i) + "]/propeller-rpm");
        e.thrust = pathProp("engine_thrust", "propulsion/engine[{n}]/thrust-lbs", i);
        if (profile.hasReversers)
            e.reverser = pathProp("reverser_cmd", "propulsion/engine[{n}]/reverser-angle-rad", i);
        const auto type = propulsion->GetEngine(static_cast<unsigned>(i))->GetType();
        e.turbine = type == JSBSim::FGEngine::etTurbine || type == JSBSim::FGEngine::etTurboprop;
    }
    reverserOff = propNumber("reverser_off", 0.0);
    reverserOn = propNumber("reverser_on", kPi);
    reverserTravelS = propNumber("reverser_travel_s", 1.5);
    if (profile.hasReversers && !eng[0].reverser.valid())
        LOG_WARN("profile %s: reverser property '%s' not found in the model", profile.id.c_str(),
                 pathFor("reverser_cmd", "propulsion/engine[{n}]/reverser-angle-rad").c_str());

    // Ground contact points. Wheels are BOGEY contacts; everything else (and any bogey named in
    // the profile's structure_contacts list, which some models use for wing tips and tail) is a
    // structure point whose contact means a crash.
    std::vector<std::string> structureNames;
    if (profile.properties.count("structure_contacts"))
        structureNames = splitTrim(profile.properties.at("structure_contacts"), ',');
    gears.clear();
    gearHeightFt = 0;
    auto gr = fdm->GetGroundReactions();
    for (int i = 0; i < gr->GetNumGearUnits(); ++i) {
        GearInfo gi;
        gi.unit = gr->GetGearUnit(i);
        gi.retractable = gi.unit->GetRetractable();
        const std::string& name = gi.unit->GetName();
        gi.structure = !gi.unit->IsBogey() ||
                       std::find(structureNames.begin(), structureNames.end(), name) !=
                           structureNames.end();
        gi.reason = gi.structure ? structureCrashReason(name) : nullptr;
        if (!gi.structure) {
            // body z is down-positive: the wheel is below the CG by +z
            gearHeightFt = std::max(gearHeightFt, static_cast<float>(gi.unit->GetBodyLocation(3)));
        }
        gears.push_back(std::move(gi));
    }
    limits = crashLimitsFor(profile, gearHeightFt);
    LOG_DEBUG("%s: %d engines, %zu contacts, gear height %.2f ft, belly %.2f ft, hard landing %.0f fpm",
              profile.id.c_str(), engineCount, gears.size(), gearHeightFt, limits.bellyAglFt,
              limits.hardLandingFpm);

    steerFullKt = static_cast<float>(propNumber("ground.steer_full_below_kt", 15));
    steerMinKt = static_cast<float>(propNumber("ground.steer_min_above_kt", 60));
    steerMinFrac = static_cast<float>(propNumber("ground.steer_min_frac", 0.15));
}

bool FlightModel::Impl::trim() {
    // Full trim (pitch, roll, yaw, sideslip) first; fall back to longitudinal only.
    for (int mode : {static_cast<int>(JSBSim::tFull), static_cast<int>(JSBSim::tLongitudinal)}) {
        try {
            fdm->DoTrim(mode);
            return true;
        } catch (const std::exception& e) {
            LOG_WARN("trim mode %d failed (%s)", mode, e.what());
        }
        if (!fdm->RunIC()) return false;
    }
    return false;
}

bool FlightModel::Impl::start(const AircraftProfile& prof, const StartCondition& sc,
                              AircraftState& state, ControlInputs& startControls) {
    ready = false;
    crashed = false;
    crashReason[0] = 0;
    touchdownWindowEndS = -1;
    stallWarnLatched = false;
    reverserLatched = false;
    reverserPos = 0;
    profile = prof;
    startCondition = sc;
    haveStart = true;
    JSBSim::SetLogger(std::make_shared<JsbsimLogger>());

    gears.clear();  // release gear units before the old executive goes away
    fdm.reset();
    fdm = std::make_unique<JSBSim::FGFDMExec>();
    fdm->SetDebugLevel(0);
    fdm->SetRootDir(SGPath(jsbsimRoot));
    fdm->SetAircraftPath(SGPath("aircraft"));
    fdm->SetEnginePath(SGPath("engine"));
    fdm->SetSystemsPath(SGPath("systems"));
    if (!fdm->LoadModel(profile.jsbsimModel)) {
        LOG_ERROR("JSBSim could not load model '%s' from %s", profile.jsbsimModel.c_str(),
                  jsbsimRoot.c_str());
        return false;
    }
    if (profile.flapDetents.empty()) {
        LOG_ERROR("profile %s has no flap detents", profile.id.c_str());
        return false;
    }
    stepDtS = kSimDtS;
    fdm->Setdt(stepDtS);

    // ---- initial condition (geodetic latitude, flat ground at the field elevation) ----
    auto ic = fdm->GetIC();
    ic->SetGeodLatitudeRadIC(degToRad(sc.latDeg));
    ic->SetLongitudeDegIC(sc.lonDeg);
    ic->SetTerrainElevationFtIC(sc.fieldElevFt);
    ic->SetPsiDegIC(sc.headingTrueDeg);
    ic->SetPhiDegIC(0.0);
    ic->SetThetaDegIC(0.0);
    // Wind is NOT put into the initial condition. JSBSim's IC wind keeps the ground velocity and
    // shifts the airspeed, which leaves a crosswind start with a sideslip that cannot be trimmed.
    // Instead the aircraft is trimmed in calm air at the requested IAS, and afterwards the wind is
    // switched on while the body velocity is shifted by the same vector, so the air-relative state
    // (IAS, alpha, beta, trim) is untouched and only the ground track and speed change.
    const bool airborne = sc.kind != StartCondition::Kind::Runway;
    auto applyAirborneIc = [&](double headingTrueDeg) {
        ic->SetPsiDegIC(headingTrueDeg);
        ic->SetAltitudeASLFtIC(sc.altMslFt);
        ic->SetVcalibratedKtsIC(sc.iasKt);
        ic->SetFlightPathAngleDegIC(sc.flightPathDeg);
    };
    if (airborne) {
        applyAirborneIc(sc.headingTrueDeg);
    } else {
        ic->SetAltitudeASLFtIC(sc.fieldElevFt + 5.0);  // settled onto the gear by the ground trim
        ic->SetVgroundKtsIC(0.0);
    }
    if (!fdm->RunIC()) {
        LOG_ERROR("JSBSim RunIC failed");
        return false;
    }
    fdm->GetPropulsion()->InitRunning(-1);  // all engines running
    bindProps();

    // Wind vector (the direction the air moves toward) and the heading that makes the ground
    // track follow the requested direction (a final approach keeps its centerline in a crosswind).
    const double windFromRad = degToRad(sc.windFromTrueDeg);
    const double windNFps = -ktToFps(sc.windSpeedKt) * std::cos(windFromRad);
    const double windEFps = -ktToFps(sc.windSpeedKt) * std::sin(windFromRad);
    if (airborne && sc.kind == StartCondition::Kind::Final && sc.windSpeedKt > 0.1f) {
        const double trackRad = degToRad(sc.headingTrueDeg);
        const double crossToRightFps =
            -windNFps * std::sin(trackRad) + windEFps * std::cos(trackRad);
        const double tasFps = ktToFps(pVt.get());
        if (tasFps > 1.0) {
            const double crab = std::asin(std::clamp(-crossToRightFps / tasFps, -0.7, 0.7));
            applyAirborneIc(sc.headingTrueDeg + radToDeg(crab));
        }
    }

    const int detentIdx =
        std::clamp(sc.flapDetent, 0, static_cast<int>(profile.flapDetents.size()) - 1);
    const float flapCmd = profile.flapDetents[static_cast<size_t>(detentIdx)].cmdNorm;
    flapDetent = detentIdx;
    parkingBrake = false;
    // Gear: fixed gear is always down, whatever the start asks for.
    const bool startGearDown = sc.gearDown || !profile.hasRetractableGear;

    if (airborne) {
        // Let flaps and gear travel to their commanded positions with the aircraft pinned, then
        // re-apply the initial condition and trim.
        cFlap.set(flapCmd);
        cGear.set(startGearDown ? 1.0 : 0.0);
        fdm->SetHoldDown(true);
        for (int i = 0; i < 90 * kSimHz; ++i) {
            fdm->Run();
            if (std::fabs(pFlap.get() - flapCmd) < 0.005 &&
                std::fabs(pGear.get() - (startGearDown ? 1.0 : 0.0)) < 0.005)
                break;
        }
        fdm->SetHoldDown(false);
        if (!fdm->RunIC()) return false;
        // A failed trim must not leave half-solved controls behind: put them back.
        double thr0[kMaxEngines] = {0, 0, 0, 0};
        for (int i = 0; i < engineCount; ++i) thr0[i] = eng[i].throttleCmd.get();
        const bool trimmed = trim();
        if (!trimmed) {
            LOG_WARN("could not trim %s; starting untrimmed", profile.id.c_str());
            for (int i = 0; i < engineCount; ++i) eng[i].throttleCmd.set(thr0[i]);
            cElevator.set(0.0);
            cAileron.set(0.0);
            cRudder.set(0.0);
            pPitchTrim.set(0.0);
            pRollTrim.set(0.0);
            pYawTrim.set(0.0);
            if (!fdm->RunIC()) return false;
        }
        // Move the trim solution into the trim controls (the stick must be centered at start).
        const double elev = cElevator.get(), ail = cAileron.get(), rud = cRudder.get();
        pPitchTrim.set(clampf(static_cast<float>(pPitchTrim.get() + elev), -1.f, 1.f));
        pRollTrim.set(clampf(static_cast<float>(pRollTrim.get() + ail), -1.f, 1.f));
        pYawTrim.set(clampf(static_cast<float>(pYawTrim.get() + rud), -1.f, 1.f));
        cElevator.set(0.0);
        cAileron.set(0.0);
        cRudder.set(0.0);
    } else {
        parkingBrake = true;
        pLBrake.set(1.0);
        pRBrake.set(1.0);
        pCBrake.set(1.0);
        cGear.set(1.0);
        cFlap.set(flapCmd);
        try {
            fdm->DoTrim(JSBSim::tGround);
        } catch (const std::exception& e) {
            LOG_WARN("ground trim failed (%s); settling by simulation", e.what());
        }
        // A short settle so the suspension has compressed before the first frame.
        for (int i = 0; i < 2 * kSimHz; ++i) fdm->Run();
    }
    // Switch the wind on (see the note above). For the parked aircraft the air-relative velocity
    // is simply the wind; for an airborne one the body velocity moves with the air mass.
    if (sc.windSpeedKt > 0.01f) {
        fdm->GetWinds()->SetWindNED(windNFps, windEFps, 0.0);
        if (airborne) {
            auto propagate = fdm->GetPropagate();
            const JSBSim::FGColumnVector3 windBody =
                propagate->GetTl2b() * JSBSim::FGColumnVector3(windNFps, windEFps, 0.0);
            for (unsigned i = 1; i <= 3; ++i)
                propagate->SetUVW(i, propagate->GetUVW(static_cast<int>(i)) + windBody(i));
        }
    }
    fdm->Setsim_time(0.0);

    // Eye point: model EYEPOINT (structural frame) to body axes, else the profile fallback.
    for (float& v : eyeBodyM) v = 0;
    const auto& ep = fdm->GetAircraft()->GetXYZep();
    if (ep(1) != 0.0 || ep(2) != 0.0 || ep(3) != 0.0) {
        const auto body =
            fdm->GetMassBalance()->StructuralToBody(ep);  // feet, x fwd y right z down
        for (int i = 0; i < 3; ++i)
            eyeBodyM[i] = static_cast<float>(ftToM(body(static_cast<unsigned>(i) + 1)));
    } else if (profile.hasEyeOffset) {
        for (int i = 0; i < 3; ++i) eyeBodyM[i] = profile.eyeOffsetBodyM[i];
    }

    // ---- controls matching the start (our sign conventions) ----
    ControlInputs sc2;
    for (int i = 0; i < engineCount; ++i)
        sc2.throttleNorm[i] = clampf(static_cast<float>(eng[i].throttleCmd.get()), 0.f, 1.f);
    sc2.mixtureNorm = 1.0f;
    sc2.elevTrimNorm = -static_cast<float>(pPitchTrim.get());
    sc2.aileronTrimNorm = static_cast<float>(pRollTrim.get());
    sc2.rudderTrimNorm = -static_cast<float>(pYawTrim.get());
    sc2.flapDetent = detentIdx;
    sc2.gearDown = startGearDown;
    sc2.parkingBrake = parkingBrake;
    startControls = sc2;

    state = AircraftState{};
    ready = true;
    refresh(state);
    return true;
}

void FlightModel::Impl::applyControls(const ControlInputs& in, const AircraftState& state) {
    // JSBSim: elevator +1 = nose down, rudder +1 = yaw left. Aileron +1 = roll right (as ours).
    cElevator.set(-clampf(in.elevatorNorm, -1.f, 1.f));
    cAileron.set(clampf(in.aileronNorm, -1.f, 1.f));
    cRudder.set(-clampf(in.rudderNorm, -1.f, 1.f));
    pPitchTrim.set(-clampf(in.elevTrimNorm, -1.f, 1.f));
    pRollTrim.set(clampf(in.aileronTrimNorm, -1.f, 1.f));
    pYawTrim.set(-clampf(in.rudderTrimNorm, -1.f, 1.f));

    float maxThrottle = 0;
    for (int i = 0; i < engineCount; ++i) {
        const float thr = clampf(in.throttleNorm[i], 0.f, 1.f);
        maxThrottle = std::max(maxThrottle, thr);
        eng[i].throttleCmd.set(thr);
        eng[i].mixtureCmd.set(clampf(in.mixtureNorm, 0.f, 1.f));
    }

    const int last = static_cast<int>(profile.flapDetents.size()) - 1;
    flapDetent = std::clamp(in.flapDetent, 0, last);
    cFlap.set(profile.flapDetents[static_cast<size_t>(flapDetent)].cmdNorm);

    // Gear protection: ignore gear-up while any wheel has weight on it; fixed gear stays down.
    const bool gearDown = in.gearDown || !profile.hasRetractableGear || state.wowAny;
    cGear.set(gearDown ? 1.0 : 0.0);

    // Reversers: deploy only on the ground with the throttles at idle; they stow when the pilot
    // releases them or when the wheels leave the ground. Thrust in reverse then follows the
    // throttle (JSBSim multiplies the engine thrust by cos(reverser angle)).
    if (profile.hasReversers) {
        const bool want = in.reverseThrust;
        if (want && !reverserLatched) {
            if (state.wowAny && maxThrottle <= 0.05f) reverserLatched = true;
        } else if (!want || !state.wowAny) {
            reverserLatched = false;
        }
        const float target = reverserLatched ? 1.f : 0.f;
        if (reverserTravelS <= 0.0) {
            reverserPos = target;
        } else {
            const float step = static_cast<float>(stepDtS / reverserTravelS);
            reverserPos += clampf(target - reverserPos, -step, step);
        }
        const double angle = reverserOff + (reverserOn - reverserOff) * reverserPos;
        for (int i = 0; i < engineCount; ++i) eng[i].reverser.set(angle);
    }

    float speedbrake = profile.hasSpeedbrake ? clampf(in.speedbrakeNorm, 0.f, 1.f) : 0.f;
    cSpeedbrake.set(speedbrake);
    // Ground spoilers (models that have them): dump lift on the ground when the speedbrake is
    // out or reverse thrust is selected.
    if (cSpoiler.valid())
        cSpoiler.set(state.wowAny && (speedbrake > 0.5f || reverserPos > 0.5f) ? 1.0 : 0.0);

    parkingBrake = in.parkingBrake;
    const float park = parkingBrake ? 1.f : 0.f;
    const float left = std::max(clampf(in.brakeLeftNorm, 0.f, 1.f), park);
    const float right = std::max(clampf(in.brakeRightNorm, 0.f, 1.f), park);
    pLBrake.set(left);
    pRBrake.set(right);
    pCBrake.set(std::max(park, 0.5f * (left + right)));

    // Nose-wheel steering authority fades with ground speed (full authority at taxi speed, a
    // fraction at takeoff speed) so a pedal-linked input cannot swerve a jet at 100 kt.
    float steerGain = 1.f;
    if (state.gsKt > steerFullKt && steerMinKt > steerFullKt)
        steerGain = std::max(steerMinFrac, 1.f - (1.f - steerMinFrac) * (state.gsKt - steerFullKt) /
                                                     (steerMinKt - steerFullKt));
    cSteer.set(clampf(in.steeringNorm, -1.f, 1.f) * steerGain);
}

float FlightModel::Impl::terrainSlopeAlongTrack(const AircraftState& s) const {
    if (!terrain) return 0.f;
    double lat2 = 0, lon2 = 0;
    destinationPoint(s.latDeg, s.lonDeg, s.trackTrueRad, kTerrainSlopeProbeM, lat2, lon2);
    const double here = terrain->elevationM(s.latDeg, s.lonDeg);
    const double ahead = terrain->elevationM(lat2, lon2);
    return static_cast<float>((ahead - here) / kTerrainSlopeProbeM);  // rise over run, + = uphill
}

void FlightModel::Impl::declareCrash(AircraftState& s, const char* reason, float vsFpm) {
    crashed = true;
    std::strncpy(crashReason, reason, sizeof crashReason - 1);
    crashReason[sizeof crashReason - 1] = 0;
    LOG_WARN("crash: %s (vs %.0f fpm, roll %.1f deg, pitch %.1f deg, ias %.0f kt, agl %.1f ft)",
             reason, vsFpm, radToDeg(s.rollRad), radToDeg(s.pitchRad), s.iasKt, s.altAglFt);
}

void FlightModel::Impl::refresh(AircraftState& s) {
    const float prevVs = s.vsFpm;
    const bool wasWowAny = s.wowAny;

    s.valid = true;
    s.simTimeS = fdm->GetSimTime();
    s.latDeg = pLat.get();
    s.lonDeg = pLon.get();
    s.altMslFt = pAlt.get();
    s.altAglFt = static_cast<float>(pAgl.get());
    s.terrainElevFt = static_cast<float>(pTerrain.get());
    s.rollRad = static_cast<float>(pPhi.get());
    s.pitchRad = static_cast<float>(pTheta.get());
    s.headingTrueRad = static_cast<float>(wrap2Pi(pPsi.get()));
    s.pRadps = static_cast<float>(pP.get());
    s.qRadps = static_cast<float>(pQ.get());
    s.rRadps = static_cast<float>(pR.get());
    s.iasKt = static_cast<float>(pVc.get());
    s.tasKt = static_cast<float>(pVt.get());
    s.mach = static_cast<float>(pMach.get());
    s.alphaDeg = static_cast<float>(pAlpha.get());
    s.betaDeg = static_cast<float>(pBeta.get());
    s.vsFpm = static_cast<float>(pHdot.get() * 60.0);
    s.nzG = static_cast<float>(pNz.get(1.0));
    s.gsKt = static_cast<float>(fpsToKt(pVg.get()));
    s.velNorthMps = static_cast<float>(fpsToMps(pVn.get()));
    s.velEastMps = static_cast<float>(fpsToMps(pVe.get()));
    s.velDownMps = static_cast<float>(fpsToMps(pVd.get()));
    s.trackTrueRad = s.gsKt > 1.0f ? static_cast<float>(wrap2Pi(std::atan2(pVe.get(), pVn.get())))
                                   : s.headingTrueRad;
    // Wind the aircraft flies in: steady wind (scenario) plus gusts. JSBSim wind vectors point
    // where the air moves TO, so "from" is the opposite direction.
    const double windN = pWindN.get() + pGustN.get(), windE = pWindE.get() + pGustE.get();
    s.windKt = static_cast<float>(fpsToKt(std::hypot(windN, windE)));
    s.windFromTrueDeg = s.windKt > 0.01f
                            ? static_cast<float>(wrap360(radToDeg(std::atan2(-windE, -windN))))
                            : 0.f;
    for (int i = 0; i < 3; ++i) s.eyeBodyM[i] = eyeBodyM[i];

    s.flapPosNorm = static_cast<float>(pFlap.get());
    s.flapDetent = flapDetent;
    s.gearPosNorm = static_cast<float>(pGear.get(1.0));
    s.gearRetractable = profile.hasRetractableGear;
    // Weight on wheels counts wheels only; any other contact point touching the ground is a
    // structure strike (crash detection below).
    s.wowAny = false;
    bool anyWheel = false, allWheelsDown = true;
    const GearInfo* structureHit = nullptr;
    for (const GearInfo& g : gears) {
        const bool wow = g.unit->GetWOW();
        if (!g.structure) {
            anyWheel = true;
            s.wowAny = s.wowAny || wow;
            allWheelsDown = allWheelsDown && wow;
        } else if (wow && !structureHit) {
            structureHit = &g;
        }
    }
    s.wowAll = anyWheel && allWheelsDown;
    s.speedbrakePosNorm = static_cast<float>(pSpeedbrake.get());
    s.elevTrimNorm = -static_cast<float>(pPitchTrim.get());
    s.aileronTrimNorm = static_cast<float>(pRollTrim.get());
    s.rudderTrimNorm = -static_cast<float>(pYawTrim.get());
    s.brakeLeftNorm = static_cast<float>(pLBrake.get());
    s.brakeRightNorm = static_cast<float>(pRBrake.get());
    s.parkingBrake = parkingBrake;
    s.engineCount = engineCount;
    for (int i = 0; i < engineCount; ++i) {
        EngineState& e = s.engines[i];
        e.running = eng[i].running.get() > 0.5;
        e.throttleCmdNorm = static_cast<float>(eng[i].throttleCmd.get());
        e.n1Pct = eng[i].turbine ? static_cast<float>(eng[i].n1.get()) : 0.f;
        e.n2Pct = eng[i].turbine ? static_cast<float>(eng[i].n2.get()) : 0.f;
        e.rpm = profile.category == AircraftCategory::Jet ? 0.0f
                                                           : static_cast<float>(eng[i].rpm.get());
        e.thrustLbf = static_cast<float>(eng[i].thrust.get());
        e.reverserDeployed = profile.hasReversers && reverserPos > 0.5f;
    }
    s.fuelTotalLbs = static_cast<float>(pFuel.get());

    // Warnings. The stall warning has hysteresis so it does not chatter at the threshold.
    if (profile.stallWarnAlphaDeg > 0 && !s.wowAny) {
        const float on = profile.stallWarnAlphaDeg;
        stallWarnLatched = s.alphaDeg > on || (stallWarnLatched && s.alphaDeg > on - kStallWarnHysteresisDeg);
    } else {
        stallWarnLatched = false;
    }
    s.stallWarning = stallWarnLatched;
    const float vne = profile.vmoKt > 0 ? profile.vmoKt : profile.vneKt;
    s.overspeed = (vne > 0 && s.iasKt > vne) || (profile.mmo > 0 && s.mach > profile.mmo);

    // ---- crash detection ----------------------------------------------------------------
    const bool touchdown = s.wowAny && !wasWowAny;  // first wheel contact this step
    if (touchdown) touchdownWindowEndS = s.simTimeS + limits.touchdownWindowS;
    if (!crashed && s.simTimeS > kGraceAfterStartS) {
        const char* reason = nullptr;
        const bool gearNotDown = profile.hasRetractableGear && s.gearPosNorm < 0.99f;
        const bool contact = s.wowAny || structureHit;
        if (structureHit) {
            reason = gearNotDown ? "gear-up landing" : structureHit->reason;
        } else if (gearNotDown && !s.wowAny && s.altAglFt < limits.bellyAglFt) {
            // Retracted gear: the model has no belly contact points, so we use the CG height.
            reason = "gear-up landing";
        }
        if (!reason && contact && terrain &&
            terrain->landClass(s.latDeg, s.lonDeg) == LandClass::Water) {
            reason = "water impact";
        }
        if (!reason && touchdown) {
            TouchdownSample td;
            td.sinkFpm = prevVs;
            td.closureFpm = prevVs - static_cast<float>(terrainSlopeAlongTrack(s) * s.gsKt *
                                                        kFpmPerMps * kMpsPerKt);
            td.rollRad = s.rollRad;
            td.pitchRad = s.pitchRad;
            reason = evaluateTouchdownImpact(limits, td);
        } else if (!reason && s.wowAny && s.simTimeS <= touchdownWindowEndS) {
            TouchdownSample td;
            td.rollRad = s.rollRad;
            td.pitchRad = s.pitchRad;
            reason = evaluateTouchdownAttitude(limits, td);
        }
        if (reason) declareCrash(s, reason, prevVs);
    }
    s.crashed = crashed;
    std::memcpy(s.crashReason, crashReason, sizeof s.crashReason);
}

// ------------------------------------------------------------------------------------------------

FlightModel::FlightModel(std::string jsbsimRoot) : impl_(new Impl(std::move(jsbsimRoot))) {}
FlightModel::~FlightModel() = default;

bool FlightModel::started() const {
    return impl_->ready;
}

bool FlightModel::start(const AircraftProfile& profile, const StartCondition& sc) {
    state_ = AircraftState{};
    startControls_ = ControlInputs{};
    try {
        return impl_->start(profile, sc, state_, startControls_);
    } catch (const std::exception& e) {
        LOG_ERROR("flight model start failed: %s", e.what());
    } catch (...) {
        LOG_ERROR("flight model start failed (unknown exception)");
    }
    impl_->ready = false;
    return false;
}

bool FlightModel::restart() {
    if (!impl_->haveStart) return false;
    const AircraftProfile profile = impl_->profile;  // copies: start() reassigns the members
    const StartCondition sc = impl_->startCondition;
    return start(profile, sc);
}

void FlightModel::step(const ControlInputs& in, double dtS) {
    if (!impl_->ready || state_.crashed) return;
    if (std::fabs(dtS - impl_->stepDtS) > 1e-9) {
        impl_->stepDtS = dtS;
        impl_->fdm->Setdt(dtS);
    }
    impl_->applyControls(in, state_);
    impl_->fdm->Run();
    impl_->refresh(state_);
}

void FlightModel::setTerrain(const TerrainQuery* terrain) {
    impl_->terrain = terrain;
}

const AircraftProfile& FlightModel::profile() const {
    return impl_->profile;
}

double FlightModel::getProperty(const std::string& path) const {
    return impl_->fdm ? impl_->fdm->GetPropertyValue(path) : 0.0;
}

void FlightModel::setProperty(const std::string& path, double value) {
    if (impl_->fdm) impl_->fdm->SetPropertyValue(path, value);
}

}  // namespace fsim
