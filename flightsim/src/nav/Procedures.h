#pragma once
// Real instrument procedures (WP N4): loader for the `procedures/{ICAO}.proc` files that
// tools/nav/cifp.py builds from the FAA CIFP (format: DATA.md section 7).
//
//   std::vector<Approach> aps = loadApproaches(navDir, "KOTH");
//   const Approach* ap = findApproach(aps, "I05");           // or "I05-Z", "R05-Y", "VOR-B"
//   std::vector<Leg> legs = assembleApproachLegs(*ap, "RARES");
//   insertApproach(plan, ap->id, ap->kind, ap->runwayIdent, legs);   // nav/FlightPlan.h
//
// Conventions of the produced `Approach` objects (core/NavTypes.h):
//  - `id` is the full CIFP ident without "RW": "I05-Z", "R28L-Y", "H05-Z", "L13", "VOR-B". The
//    "-Y"/"-Z" part is the FAA multiple-procedure indicator ("ILS Z RWY 5"). findApproach() also
//    accepts the short form ("I05") and returns the first procedure of that name (sorted by id).
//  - `runwayIdent` is DB style ("05", "28L"); empty for circling-only approaches ("VOR-B").
//  - `finalLegs` run from the initial fix of the final segment up to and including the MAP leg
//    (the runway threshold for runway-aligned approaches). `missedLegs` follow the MAP and all have
//    `missedApproach = true`.
//  - `transitions` follow N5's ApproachBuilder convention: transitions[0] is "vectors" (empty
//    name, no legs), the IAF routes come after it. Each ends at (or just before) the first fix of
//    the final segment: a trailing direct leg to that fix is dropped (the final segment's IF
//    carries it); holds and CF legs stay, so an IF may repeat the fix of the previous leg.
//  - Hold legs (HM/HA/HF): `courseTrueDeg` is the INBOUND course and `arcRight` is the turn
//    direction (true = right turns). RF legs: arcCenter*/arcRadiusNm/arcRight. `Between` altitude
//    constraints store the lower limit in altFt and the upper limit in alt2Ft.
//  - `minimumsFt` is 0 (CIFP carries no minimums); N5 falls back to its synthesized values.
//  - Waypoint kinds: runway fixes ("RW05") are `Runway`, VOR/NDB fixes `Vor`/`Ndb`, computed
//    fixes of approximated legs `Synthetic`, everything else `Fix`.
//
// Data not representable in `Leg` (FAF/IAF/MAP roles, hold leg length, published vertical angle,
// the ARINC type an approximated leg came from) is available in ApproachInfo.

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "core/NavTypes.h"
#include "nav/Fms.h"

namespace fsim {

// Extra per-leg data, parallel to the leg vectors of the Approach it belongs to.
struct ProcLegExtra {
    bool iaf = false, faf = false, map = false;
    float vertAngleDeg = 0;   // published vertical angle on this leg (negative = descent), 0 = none
    float holdLegTimeS = 0;   // hold legs: outbound leg time (0 = not specified)
    float holdLegDistNm = 0;  // hold legs: outbound leg length (0 = not specified)
    std::string
        approximatedFrom;  // ARINC 424 path terminator this leg replaces ("AF", "PI", ...); empty = native
};

struct ApproachInfo {
    std::string title;                                        // "ILS Z RWY 05"
    std::vector<std::string> levelsOfService;                 // RNAV: "LPV", "LNAV/VNAV", "LNAV"
    std::vector<std::vector<ProcLegExtra>> transitionExtras;  // [transition][leg]
    std::vector<ProcLegExtra> finalExtras;                    // parallel to Approach::finalLegs
    std::vector<ProcLegExtra> missedExtras;                   // parallel to Approach::missedLegs
    int fafFinalLeg = -1;  // index into finalLegs of the FAF, -1 if the procedure has none
    int mapFinalLeg = -1;  // index into finalLegs of the MAP (always the last final leg), -1 = none
};

// N8: a SID or STAR. A SID is flown runway transition + common route + enroute transition, a
// STAR enroute transition + common route + runway transition (see assembleSidLegs /
// assembleStarLegs). Runway transitions are named DB style: "28L", "10B" (both 10s), "ALL".
struct ProcRoute {
    std::string name;
    std::vector<Leg> legs;
};
struct TerminalProcedure {
    std::string id;     // "MINNE5", "HHOOD6"
    std::string title;  // the rest of the SID / STAR line
    bool isSid = true;  // false = STAR
    std::vector<ProcRoute> runwayTransitions;
    std::vector<Leg> commonLegs;
    std::vector<ProcRoute> enrouteTransitions;
};

struct ProcedureSet {
    std::string airportIdent;
    std::string cycle;                 // FAA cycle "2610"
    std::vector<Approach> approaches;  // sorted by id
    std::vector<ApproachInfo> info;    // parallel to approaches
    std::vector<std::string> warnings;
    std::vector<TerminalProcedure> sids, stars;  // N8, sorted by id
};

// Parses the text of a .proc file. Returns false with a readable message for an unknown format
// version or a malformed line; unknown directives and KEY=value options are skipped.
bool parseProcedures(std::string_view text, ProcedureSet& out, std::string& error);

// Loads {navDir}/procedures/{icao}.proc (also tried: {navDir}/nav/procedures/, so a data root
// works too). The parsed set is cached per (directory, ICAO), including "no file"; the result is
// null when the airport has no procedures or the file is unreadable (logged once).
std::shared_ptr<const ProcedureSet> loadProcedureSet(const std::string& navDir,
                                                     std::string_view icao);
void clearProcedureCache();

// What N5/H5 pass as the realProvider. Empty when the airport has no real procedures.
std::vector<Approach> loadApproaches(const std::string& dataDir, std::string_view icao);

// The provider N5 expects (nav/ApproachBuilder.h `RealApproachProvider`, the same std::function
// type): the real approaches that land on `runway` of `airport`, ordered by id, empty when the
// airport has no procedure file. Circling-only approaches (no runway) are not offered.
//   navigator.setRealApproachProvider(makeCifpApproachProvider(paths.data("nav")));
using CifpApproachProvider =
    std::function<std::vector<Approach>(const Airport& airport, const Runway& runway)>;
CifpApproachProvider makeCifpApproachProvider(const std::string& navDir);

// Exact (case-insensitive) match of `id` first; then the short form without the multiple-procedure
// indicator ("I05" finds "I05-Y"/"I05-Z", the first by id wins; "I16R" finds "I16RZ", but "R28"
// does not find "R28L"). nullptr if nothing matches.
const Approach* findApproach(const std::vector<Approach>& approaches, std::string_view id);

// All approaches that land on `runwayIdent` (DB style, "05", "28L"), ordered by id.
std::vector<const Approach*> approachesForRunway(const std::vector<Approach>& approaches,
                                                 std::string_view runwayIdent);

// The legs to hand to insertApproach(): transition `transitionName` (empty or unknown = none,
// i.e. vectors to the final segment), then the final legs, then the missed legs. When the transition
// already ends at the first fix of the final segment (the usual case) the duplicate IF leg of the
// final segment is dropped and its altitude constraint is kept if the transition leg has none.
// `transitionFound` (optional) reports whether the named transition exists. A transition that
// joins the final segment later than its first fix (a procedure turn at the FAF, 5% of the US
// transitions) skips the final legs before the join, see transitionJoinLeg().
std::vector<Leg> assembleApproachLegs(const Approach& approach, std::string_view transitionName,
                                      bool* transitionFound = nullptr);

// Index into finalLegs of the fix where transition `transitionIndex` meets the final segment:
// 0 for the usual case (and for vectors), k > 0 when the transition ends at a later final fix.
// N5's approachPlanLegs() concatenates transition + ALL final legs; for those transitions
// assembleApproachLegs() is the better source of plan legs.
int transitionJoinLeg(const Approach& approach, size_t transitionIndex);

// Like ApproachInfo lookup by id for a loaded set; nullptr if absent.
const ApproachInfo* findApproachInfo(const ProcedureSet& set, std::string_view approachId);

// ---- N8: SIDs and STARs --------------------------------------------------------------------

// True when runway transition `transitionName` serves `runwayIdent`: the same name, "ALL", or
// "10B" for 10L / 10C / 10R.
bool runwayTransitionMatches(std::string_view transitionName, std::string_view runwayIdent);
// Case-insensitive id match; nullptr if absent.
const TerminalProcedure* findSid(const ProcedureSet& set, std::string_view id);
const TerminalProcedure* findStar(const ProcedureSet& set, std::string_view id);
// The SIDs usable from `runwayIdent` (a matching runway transition, or no runway transitions at
// all) and the STARs that serve it (or have no runway transitions), ordered by id. An empty
// runway lists them all.
std::vector<const TerminalProcedure*> sidsForRunway(const ProcedureSet& set,
                                                    std::string_view runwayIdent);
std::vector<const TerminalProcedure*> starsForRunway(const ProcedureSet& set,
                                                     std::string_view runwayIdent);
// Plan legs of a SID from `runwayIdent` into enroute transition `transition` ("" = none), or of
// a STAR from `transition` to `runwayIdent`. A leading IF that repeats the previous fix is dropped
// (its altitude constraint is kept if the previous leg has none); an IF in mid route becomes a TF.
// Every leg is tagged LegSource::Sid / Star. `missing` (optional) receives a readable note when the
// runway or the transition was not found (the rest is still assembled).
std::vector<Leg> assembleSidLegs(const TerminalProcedure& sid, std::string_view runwayIdent,
                                 std::string_view transition, std::string* missing = nullptr);
std::vector<Leg> assembleStarLegs(const TerminalProcedure& star, std::string_view transition,
                                  std::string_view runwayIdent, std::string* missing = nullptr);
// The procedures an .fms file names: inserts its SID (from the plan's origin and departure runway)
// and STAR (to the destination and arrival runway) from {navDir}/procedures, and keeps its
// approach transition in plan.approachTransition. Returns a note for each name not found.
std::vector<std::string> applyFmsProcedures(FlightPlan& plan, const FmsFile& file,
                                            const std::string& navDir);

}  // namespace fsim
