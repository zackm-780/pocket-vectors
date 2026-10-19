#include "sim/FlightModel.h"

#include <algorithm>
#include <cmath>
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

#include "core/Geo.h"
#include "core/Log.h"
#include "core/Units.h"

namespace fsim {
namespace {

// Touchdown / crash thresholds (placeholder values until F1 tunes them per aircraft).
constexpr float kHardLandingFpm = 900.0f;
constexpr float kMaxTouchdownBankRad = 0.55f;  // ~31 degrees
constexpr float kMaxTouchdownPitchUpRad = 0.45f;
constexpr float kMaxTouchdownPitchDownRad = 0.30f;
constexpr double kGraceAfterStartS = 1.0;

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
    double get(double def = 0.0) const { return node ? node->getDoubleValue() : def; }
    void set(double value) const {
        if (node) node->setDoubleValue(value);
    }
};

float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
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
    void bindProps();
    bool trim();

    std::string jsbsimRoot;
    AircraftProfile profile;
    std::unique_ptr<JSBSim::FGFDMExec> fdm;
    double stepDtS = kSimDtS;

    // cached nodes
    Prop pLat, pLon, pAlt, pAgl, pTerrain, pPhi, pTheta, pPsi, pP, pQ, pR;
    Prop pVc, pVt, pMach, pAlpha, pBeta, pHdot, pNz, pVg, pVn, pVe, pVd;
    Prop pWindN, pWindE, pFlap, pGear, pSpeedbrake, pPitchTrim, pRollTrim, pYawTrim, pFuel;
    Prop pLBrake, pRBrake, pCBrake;
    Prop cElevator, cAileron, cRudder, cFlap, cGear, cSpeedbrake, cSteer;
    struct EngineProps {
        Prop throttleCmd, mixtureCmd, running, n1, n2, rpm, thrust;
    };
    EngineProps eng[kMaxEngines];
    std::vector<std::shared_ptr<JSBSim::FGLGear>> gears;

    // crash detection / bookkeeping
    float eyeBodyM[3] = {0, 0, 0};
    bool crashed = false;
    char crashReason[sizeof(AircraftState::crashReason)] = {0};
    int flapDetent = 0;
    bool parkingBrake = false;
    bool ready = false;
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
    pFlap = prop("fcs/flap-pos-norm");
    pGear = prop("gear/gear-pos-norm");
    pSpeedbrake = prop("fcs/speedbrake-pos-norm");
    pPitchTrim = prop("fcs/pitch-trim-cmd-norm");
    pRollTrim = prop("fcs/roll-trim-cmd-norm");
    pYawTrim = prop("fcs/yaw-trim-cmd-norm");
    pFuel = prop("propulsion/total-fuel-lbs");
    pLBrake = prop("fcs/left-brake-cmd-norm");
    pRBrake = prop("fcs/right-brake-cmd-norm");
    pCBrake = prop("fcs/center-brake-cmd-norm");
    cElevator = prop("fcs/elevator-cmd-norm");
    cAileron = prop("fcs/aileron-cmd-norm");
    cRudder = prop("fcs/rudder-cmd-norm");
    cFlap = prop("fcs/flap-cmd-norm");
    cGear = prop("gear/gear-cmd-norm");
    cSpeedbrake = prop("fcs/speedbrake-cmd-norm");
    cSteer = prop("fcs/steer-cmd-norm");

    for (int i = 0; i < profile.engineCount && i < kMaxEngines; ++i) {
        const std::string idx = "[" + std::to_string(i) + "]";
        eng[i].throttleCmd = prop("fcs/throttle-cmd-norm" + idx);
        eng[i].mixtureCmd = prop("fcs/mixture-cmd-norm" + idx);
        eng[i].running = prop("propulsion/engine" + idx + "/set-running");
        eng[i].n1 = prop("propulsion/engine" + idx + "/n1");
        eng[i].n2 = prop("propulsion/engine" + idx + "/n2");
        eng[i].rpm = prop("propulsion/engine" + idx + "/engine-rpm");
        eng[i].thrust = prop("propulsion/engine" + idx + "/thrust-lbs");
    }

    gears.clear();
    auto gr = fdm->GetGroundReactions();
    for (int i = 0; i < gr->GetNumGearUnits(); ++i) gears.push_back(gr->GetGearUnit(i));
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
    profile = prof;
    JSBSim::SetLogger(std::make_shared<JsbsimLogger>());

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
    // Wind: the IC wind vector is the air-mass velocity (the direction the air moves toward), so a
    // wind FROM 270 is a vector pointing east (verified in tests/sim: headwind raises IAS over GS).
    // It must be applied after the speed, and it shifts the IC airspeed by about the wind speed, so
    // airborne starts nudge the IC speed after trim until the trimmed IAS matches (below).
    const double windFromRad = degToRad(sc.windFromTrueDeg);
    const double windFps = ktToFps(sc.windSpeedKt);
    auto setWind = [&] {
        ic->SetWindNEDFpsIC(-windFps * std::cos(windFromRad), -windFps * std::sin(windFromRad),
                            0.0);
    };
    const bool airborne = sc.kind != StartCondition::Kind::Runway;
    double icIasKt = sc.iasKt;
    auto applyAirborneIc = [&](double icIasKt) {
        ic->SetAltitudeASLFtIC(sc.altMslFt);
        ic->SetVcalibratedKtsIC(icIasKt);  // speed first: the airspeed vector must follow the nose
        setWind();                         // then wind (ground velocity is kept, airspeed shifts)
        ic->SetFlightPathAngleDegIC(sc.flightPathDeg);
    };
    if (airborne) {
        applyAirborneIc(sc.iasKt);
    } else {
        ic->SetAltitudeASLFtIC(sc.fieldElevFt + 5.0);  // settled onto the gear by the ground trim
        ic->SetVgroundKtsIC(0.0);
        setWind();
    }
    if (!fdm->RunIC()) {
        LOG_ERROR("JSBSim RunIC failed");
        return false;
    }
    fdm->GetPropulsion()->InitRunning(-1);  // all engines running
    bindProps();

    const int detentIdx =
        std::clamp(sc.flapDetent, 0, static_cast<int>(profile.flapDetents.size()) - 1);
    const float flapCmd = profile.flapDetents[static_cast<size_t>(detentIdx)].cmdNorm;
    flapDetent = detentIdx;
    parkingBrake = false;

    if (airborne) {
        // Let flaps and gear travel to their commanded positions with the aircraft pinned, then
        // re-apply the initial condition and trim.
        cFlap.set(flapCmd);
        cGear.set(sc.gearDown ? 1.0 : 0.0);
        fdm->SetHoldDown(true);
        for (int i = 0; i < 90 * kSimHz; ++i) {
            fdm->Run();
            if (std::fabs(pFlap.get() - flapCmd) < 0.005 &&
                std::fabs(pGear.get() - (sc.gearDown ? 1.0 : 0.0)) < 0.005)
                break;
        }
        fdm->SetHoldDown(false);
        if (!fdm->RunIC()) return false;
        bool trimmed = trim();
        // With wind the IC speed ends up a few knots off; nudge it until the trimmed IAS matches.
        for (int pass = 0;
             pass < 3 && trimmed && sc.windSpeedKt > 0.5f && std::fabs(pVc.get() - sc.iasKt) > 0.3;
             ++pass) {
            icIasKt += sc.iasKt - pVc.get();
            applyAirborneIc(icIasKt);
            if (!fdm->RunIC()) return false;
            trimmed = trim();
        }
        if (!trimmed) LOG_WARN("could not trim %s; starting untrimmed", profile.id.c_str());
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
    fdm->Setsim_time(0.0);

    // Eye point: model EYEPOINT (structural frame) to body axes, else the profile fallback.
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
    for (int i = 0; i < profile.engineCount && i < kMaxEngines; ++i)
        sc2.throttleNorm[i] = clampf(static_cast<float>(eng[i].throttleCmd.get()), 0.f, 1.f);
    sc2.mixtureNorm = 1.0f;
    sc2.elevTrimNorm = -static_cast<float>(pPitchTrim.get());
    sc2.aileronTrimNorm = static_cast<float>(pRollTrim.get());
    sc2.rudderTrimNorm = -static_cast<float>(pYawTrim.get());
    sc2.flapDetent = detentIdx;
    sc2.gearDown = sc.gearDown;
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

    for (int i = 0; i < profile.engineCount && i < kMaxEngines; ++i) {
        eng[i].throttleCmd.set(clampf(in.throttleNorm[i], 0.f, 1.f));
        eng[i].mixtureCmd.set(clampf(in.mixtureNorm, 0.f, 1.f));
    }

    const int last = static_cast<int>(profile.flapDetents.size()) - 1;
    flapDetent = std::clamp(in.flapDetent, 0, last);
    cFlap.set(profile.flapDetents[static_cast<size_t>(flapDetent)].cmdNorm);

    // Gear protection: ignore gear-up while any wheel has weight on it; fixed gear stays down.
    const bool gearDown = in.gearDown || !profile.hasRetractableGear || state.wowAny;
    cGear.set(gearDown ? 1.0 : 0.0);

    cSpeedbrake.set(clampf(in.speedbrakeNorm, 0.f, 1.f));
    parkingBrake = in.parkingBrake;
    const float park = parkingBrake ? 1.f : 0.f;
    pLBrake.set(std::max(clampf(in.brakeLeftNorm, 0.f, 1.f), park));
    pRBrake.set(std::max(clampf(in.brakeRightNorm, 0.f, 1.f), park));
    pCBrake.set(park);
    cSteer.set(clampf(in.steeringNorm, -1.f, 1.f));
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
    const double windN = pWindN.get(), windE = pWindE.get();
    s.windKt = static_cast<float>(fpsToKt(std::hypot(windN, windE)));
    s.windFromTrueDeg = static_cast<float>(wrap360(radToDeg(std::atan2(-windE, -windN))));
    for (int i = 0; i < 3; ++i) s.eyeBodyM[i] = eyeBodyM[i];

    s.flapPosNorm = static_cast<float>(pFlap.get());
    s.flapDetent = flapDetent;
    s.gearPosNorm = static_cast<float>(pGear.get(1.0));
    s.gearRetractable = profile.hasRetractableGear;
    // Weight on wheels counts wheels (bogeys) only; any other contact point touching the ground
    // is a structure strike (crash detection below).
    s.wowAny = false;
    bool anyBogey = false, allBogeysDown = true, structureContact = false;
    for (const auto& g : gears) {
        const bool wow = g->GetWOW();
        if (g->IsBogey()) {
            anyBogey = true;
            s.wowAny = s.wowAny || wow;
            allBogeysDown = allBogeysDown && wow;
        } else if (wow) {
            structureContact = true;
        }
    }
    s.wowAll = anyBogey && allBogeysDown;
    s.speedbrakePosNorm = static_cast<float>(pSpeedbrake.get());
    s.elevTrimNorm = -static_cast<float>(pPitchTrim.get());
    s.aileronTrimNorm = static_cast<float>(pRollTrim.get());
    s.rudderTrimNorm = -static_cast<float>(pYawTrim.get());
    s.brakeLeftNorm = static_cast<float>(pLBrake.get());
    s.brakeRightNorm = static_cast<float>(pRBrake.get());
    s.parkingBrake = parkingBrake;
    s.engineCount = profile.engineCount;
    for (int i = 0; i < profile.engineCount && i < kMaxEngines; ++i) {
        EngineState& e = s.engines[i];
        e.running = eng[i].running.get() > 0.5;
        e.throttleCmdNorm = static_cast<float>(eng[i].throttleCmd.get());
        e.n1Pct = static_cast<float>(eng[i].n1.get());
        e.n2Pct = static_cast<float>(eng[i].n2.get());
        e.rpm =
            profile.category == AircraftCategory::Jet ? 0.0f : static_cast<float>(eng[i].rpm.get());
        e.thrustLbf = static_cast<float>(eng[i].thrust.get());
    }
    s.fuelTotalLbs = static_cast<float>(pFuel.get());

    s.stallWarning =
        profile.stallWarnAlphaDeg > 0 && !s.wowAny && s.alphaDeg > profile.stallWarnAlphaDeg;
    const float vne = profile.vmoKt > 0 ? profile.vmoKt : profile.vneKt;
    s.overspeed = (vne > 0 && s.iasKt > vne) || (profile.mmo > 0 && s.mach > profile.mmo);

    // ---- crash detection (placeholder rules; F1 refines per aircraft) ----
    if (!crashed && s.simTimeS > kGraceAfterStartS) {
        const char* reason = nullptr;
        if (structureContact) {
            reason = "terrain strike";
        } else if (s.wowAny && !wasWowAny) {  // touchdown this step
            if (prevVs < -kHardLandingFpm)
                reason = "hard landing";
            else if (std::fabs(s.rollRad) > kMaxTouchdownBankRad)
                reason = "wing strike";
            else if (s.pitchRad > kMaxTouchdownPitchUpRad)
                reason = "tail strike";
            else if (s.pitchRad < -kMaxTouchdownPitchDownRad)
                reason = "nose strike";
        }
        if (reason) {
            crashed = true;
            std::strncpy(crashReason, reason, sizeof crashReason - 1);
            LOG_WARN("crash: %s (vs %.0f fpm, roll %.1f deg, pitch %.1f deg)", reason, prevVs,
                     radToDeg(s.rollRad), radToDeg(s.pitchRad));
        }
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

double FlightModel::getProperty(const std::string& path) const {
    return impl_->fdm ? impl_->fdm->GetPropertyValue(path) : 0.0;
}

void FlightModel::setProperty(const std::string& path, double value) {
    if (impl_->fdm) impl_->fdm->SetPropertyValue(path, value);
}

}  // namespace fsim
