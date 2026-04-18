import { useState, useEffect, useRef, useMemo } from "react";

// ════════════════════════════════════════════════════════════
// OAHU SFR DIGITAL TWIN — v2.3
// JavaScript port of oahu_sfr_model_v2.py
//
// Key modelling decisions (v2.1+ revisions):
//  1. Virtual-inertia accounting — GFM virtual inertia is realised
//     only as a power injection (dp_inertia in GFM respond()), NOT
//     added to H_eff.  Adding it to both would double-count.
//  2. Solar-ramp offset targeting — ibr_power_scale_offset is
//     applied only to PV (non-GFM) fleet output so that BESS
//     responds independently of cloud cover.
//  3. Spinning-reserve / headroom — the governor clips its valve
//     reference to available headroom (cap − baseLoad) and p_sync
//     is clamped to [0, syncOnlineMW].
//  4. Era dispatch sizing — syncMW covers base load + N-1 reserve.
//     Pre-BESS eras carry full ~208 MW reserve on sync; BESS eras
//     offload most reserve to the BESS, allowing sync ≈ 50-60 MW
//     above base loading.
// ════════════════════════════════════════════════════════════
const F0 = 60.0, SB = 1800.0, HS = 4.0, DROOP_SYNC = 0.05;
const GOV_DEADBAND = 0.036;
const F_HP = 0.30, T_GOV = 0.3, T_REHEAT = 7.0;
const RAMP_RATE_MW_S = 5.0;
const D_LOAD = 1.5;
const MOTOR_FRAC = 0.30, RESIST_FRAC = 0.70;
const UFLS = [
  { f: 59.3, shed: 0.05, delay: 0.10 },
  { f: 58.9, shed: 0.05, delay: 0.10 },
  { f: 58.5, shed: 0.05, delay: 0.10 },
];
const AGC_CYCLE = 2.0, AGC_KP = 50.0, AGC_KI = 20.0;
const AGC_DEADBAND = 0.02, AGC_BESS_PART = 0.70, AGC_SYNC_PART = 0.30;

// ════════════════════════════════════════════════════════════
// INVERTER CLASSES (matching v2 Python model)
// ════════════════════════════════════════════════════════════
function mkLeg(c) {
  return {
    t: "leg", c, tripped: false, power: 0, timer: 0,
    init(d = 0.7) { this.tripped = false; this.timer = 0; this.power = c * d; return this.power; },
    respond(f, dfdt, dt) {
      if (this.tripped) return 0;
      if (f <= 59.3) {
        const depth = Math.max(59.3 - f, 0.01);
        const thresh = Math.max(0.16 * 0.1 / depth, 0.05);
        this.timer += dt;
        if (this.timer >= thresh) { this.tripped = true; return 0; }
      } else { this.timer = Math.max(0, this.timer - dt); }
      return this.power;
    },
    setAgc() {}
  };
}
function mkRT(c) {
  return {
    t: "rt", c, tripped: false, power: 0,
    init(d = 0.7) { this.tripped = false; this.power = c * d; return this.power; },
    respond(f) { if (this.tripped) return 0; if (f <= 57) { this.tripped = true; return 0; } return this.power; },
    setAgc() {}
  };
}
function mkSm(c) {
  return {
    t: "sm", c, tripped: false, power: 0, pCmd: 0, pBase: 0,
    init(d = 0.7) { this.tripped = false; const p = c * d; this.power = p; this.pCmd = p; this.pBase = p; return p; },
    respond(f, dfdt, dt) {
      if (this.tripped) return 0; if (f <= 57) { this.tripped = true; return 0; }
      const dF = f - F0; let tgt;
      if (Math.abs(dF) <= 0.036) tgt = this.pBase;
      else { const sg = dF > 0 ? 1 : -1; tgt = this.pBase + (-c * (dF - sg * 0.036) / (0.04 * F0)); tgt = Math.max(0, Math.min(tgt, this.pBase + 0.05 * c)); }
      this.pCmd += (dt / (0.5 + dt)) * (tgt - this.pCmd); this.power = this.pCmd; return this.power;
    },
    setAgc() {}
  };
}
// GFM BESS — virtual synchronous machine.  Virtual inertia (VH = 5 s) is
// realised as a power injection (dpI term in respond()), NOT added to H_eff
// in the swing equation.  This avoids double-counting the inertial effect.
function mkGFM(c, soc = 0.80) {
  return {
    t: "gfm", c, VH: 5.0, tripped: false, power: 0, pCmd: 0, pBase: 0,
    soc, energyMWh: c * 4.0, agcSetpoint: 0,
    init(d = 0.0) { this.tripped = false; this.power = c * d; this.pBase = this.power; this.pCmd = this.power; this.agcSetpoint = 0; return this.power; },
    setAgc(mw) { this.agcSetpoint = Math.max(-c, Math.min(c, mw)); },
    respond(f, dfdt, dt) {
      if (this.tripped) return 0; if (f <= 56) { this.tripped = true; return 0; }
      const dF = f - F0; let dpDroop = 0;
      if (Math.abs(dF) > 0.017) { const sg = dF > 0 ? 1 : -1; dpDroop = -c * (dF - sg * 0.017) / (0.04 * F0); }
      const dpI = -2.0 * this.VH * c * dfdt / F0;
      let tgt = this.pBase + dpDroop + dpI + this.agcSetpoint;
      if (this.soc <= 0.10 && tgt > 0) tgt = 0; if (this.soc >= 0.95 && tgt < 0) tgt = 0;
      tgt = Math.max(-c, Math.min(c, tgt));
      this.pCmd += (dt / (0.05 + dt)) * (tgt - this.pCmd); this.power = this.pCmd;
      const eMWh = this.power * (dt / 3600);
      if (this.energyMWh > 0) { this.soc -= eMWh / this.energyMWh; this.soc = Math.max(0, Math.min(1, this.soc)); }
      return this.power;
    }
  };
}

// ════════════════════════════════════════════════════════════
// REHEAT GOVERNOR
// ════════════════════════════════════════════════════════════
function makeGovernor(capMW) {
  return {
    cap: capMW, pValve: 0, pReheat: 0, pSetAdj: 0, pMechPrev: 0, baseLoad: 0,
    init() { this.pValve = 0; this.pReheat = 0; this.pSetAdj = 0; this.pMechPrev = 0; this.baseLoad = 0; },
    setAgc(mw) { this.pSetAdj = mw; },
    step(freq, dt) {
      let dF = freq - F0;
      let dFeff = Math.abs(dF) <= GOV_DEADBAND ? 0 : dF - Math.sign(dF) * GOV_DEADBAND;
      // Headroom: can't exceed online capacity or go below zero
      const hUp = Math.max(this.cap - this.baseLoad, 0);
      const hDn = this.baseLoad;
      let pRef = -(1 / DROOP_SYNC) * (dFeff / F0) * this.cap + this.pSetAdj;
      pRef = Math.max(-Math.min(0.3 * this.cap, hDn), Math.min(Math.min(0.3 * this.cap, hUp), pRef));
      this.pValve += (pRef - this.pValve) / T_GOV * dt;
      const pHP = F_HP * this.pValve;
      const pRehRef = (1 - F_HP) * this.pValve;
      this.pReheat += (pRehRef - this.pReheat) / T_REHEAT * dt;
      let pMech = pHP + this.pReheat;
      // Clamp to headroom (belt-and-suspenders)
      pMech = Math.max(-hDn, Math.min(hUp, pMech));
      this.pMechPrev = pMech; return pMech;
    }
  };
}

// ════════════════════════════════════════════════════════════
// CONTINGENCY FRAMEWORK
// ════════════════════════════════════════════════════════════
function makeSyncTrip(tripMW, tripTime = 1.0) {
  let applied = false;
  return { type: "sync_trip", tripTime,
    apply(t, dt, state) {
      if (t < tripTime || applied) return; applied = true;
      state.pSyncBase -= tripMW; const ns = state.syncOnlineMW - tripMW;
      state.syncOnlineMW = ns; state.HSys = Math.max(HS * ns / SB, 0.05); state.governor.cap = ns;
    }
  };
}
function makeSolarRamp(rampMW, rampDur = 45, tripTime = 1.0) {
  let done = 0;
  return { type: "solar_ramp", tripTime,
    apply(t, dt, state) {
      if (t < tripTime) return;
      const el = t - tripTime;
      if (el >= rampDur) { const r = rampMW - done; if (r > 0.01) { state.ibrOffset -= r; done = rampMW; } return; }
      const x = el / rampDur, k = 8, sig = 1 / (1 + Math.exp(-k * (x - 0.5)));
      const s0 = 1 / (1 + Math.exp(k * 0.5)), s1 = 1 / (1 + Math.exp(-k * 0.5));
      const tgt = rampMW * (sig - s0) / (s1 - s0), d = tgt - done;
      if (d > 0) { state.ibrOffset -= d; done = tgt; }
    }
  };
}
function makeIBRTrip(tripMW, tripTime = 1.0, rampS = 0.5) {
  let done = 0;
  return { type: "ibr_trip", tripTime,
    apply(t, dt, state) {
      if (t < tripTime) return;
      const el = t - tripTime;
      if (el >= rampS) { const r = tripMW - done; if (r > 0.01) { state.ibrOffset -= r; done = tripMW; } return; }
      const tgt = tripMW * el / rampS, d = tgt - done;
      if (d > 0) { state.ibrOffset -= d; done = tgt; }
    }
  };
}
function makeBESSOutage(tripTime = 1.0) {
  let applied = false;
  return { type: "bess_outage", tripTime,
    apply(t, dt, state) {
      if (t < tripTime || applied) return; applied = true;
      for (const fl of state.fleets) { if (fl.t === "gfm") { fl.tripped = true; fl.power = 0; fl.pCmd = 0; } }
    }
  };
}
function makeCompound(list) {
  return { type: "compound", tripTime: Math.min(...list.map(c => c.tripTime)),
    apply(t, dt, state) { for (const c of list) c.apply(t, dt, state); }
  };
}

// ════════════════════════════════════════════════════════════
// SIMULATION ENGINE v2
// H_eff = physical sync inertia only (GFM virtual inertia is a
//         power injection, not added to H_eff — see note above).
// Solar/IBR offsets hit PV output only; BESS responds via controls.
// Governor is headroom-aware (capped to cap − baseLoad).
// ════════════════════════════════════════════════════════════
function simulate({ syncMW, fleets, pLoad, contingency, tEnd = 60, dt = 0.002 }) {
  const N = Math.floor(tEnd / dt);
  const tA = new Float64Array(N), fA = new Float64Array(N), rA = new Float64Array(N);
  const psA = new Float64Array(N), piA = new Float64Array(N);
  const socA = new Float64Array(N), pLoadA = new Float64Array(N);

  let HSys = HS * syncMW / SB;
  const governor = makeGovernor(syncMW); governor.init();
  let agcIntegral = 0, agcLastDispatch = -999, agcTotal = 0;

  let totalIbrBase = 0;
  for (const fl of fleets) totalIbrBase += fl.init(fl.t === "gfm" ? 0 : 0.7);
  let pSyncBase = Math.max(0, Math.min(syncMW, pLoad - totalIbrBase));
  const gfmFleet = fleets.find(f => f.t === "gfm") || null;

  let freq = F0, dfdt = 0, uflsShed = 0;
  const uT = UFLS.map(() => false), uTm = UFLS.map(() => 0);

  const st = { pSyncBase, syncOnlineMW: syncMW, HSys, governor, fleets, pLoad, ibrOffset: 0 };

  for (let i = 0; i < N; i++) {
    const t = i * dt; tA[i] = t;
    contingency.apply(t, dt, st);
    pSyncBase = st.pSyncBase; HSys = st.HSys;

    let bessAgc = 0, syncAgc = 0;
    if (t > contingency.tripTime + 2.0) {
      const dF = freq - F0;
      if (Math.abs(dF) > AGC_DEADBAND) agcIntegral += dF * dt;
      agcIntegral = Math.max(-50, Math.min(50, agcIntegral));
      if (t - agcLastDispatch >= AGC_CYCLE) {
        agcLastDispatch = t;
        agcTotal = Math.abs(dF) <= AGC_DEADBAND ? agcTotal * 0.95 : AGC_KP * (-dF) + AGC_KI * (-agcIntegral);
        agcTotal = Math.max(-300, Math.min(300, agcTotal));
      }
      bessAgc = agcTotal * AGC_BESS_PART; syncAgc = agcTotal * AGC_SYNC_PART;
      governor.setAgc(syncAgc);
      for (const fl of fleets) { if (fl.t === "gfm") fl.setAgc(bessAgc); }
    }

    governor.baseLoad = pSyncBase;
    const dpMech = governor.step(freq, dt);
    const pSync = Math.max(0, Math.min(pSyncBase + dpMech, st.syncOnlineMW));
    // Separate PV (non-GFM) and GFM so solar ramp offsets only hit PV.
    let pPV = 0, pGFM = 0;
    for (const fl of fleets) { const p = fl.respond(freq, dfdt, dt); if (fl.t === "gfm") pGFM += p; else pPV += p; }
    pPV = Math.max(0, pPV + st.ibrOffset);
    let pIbr = pPV + pGFM;

    for (let j = 0; j < UFLS.length; j++) {
      if (!uT[j]) {
        if (freq <= UFLS[j].f) { uTm[j] += dt; if (uTm[j] >= UFLS[j].delay) { uT[j] = true; uflsShed += UFLS[j].shed; } }
        else { uTm[j] = Math.max(0, uTm[j] - dt); }
      }
    }
    const pLB = pLoad * (1 - uflsShed), fPu = freq / F0;
    const pLA = pLB * MOTOR_FRAC * fPu * fPu + pLB * RESIST_FRAC;

    // H_eff uses only physical (synchronous) inertia.  GFM virtual
    // inertia is realised as an explicit power injection inside the
    // GFM respond() (dp_inertia term) — adding it here would double-count.
    let Heff = st.HSys;
    const pDamp = D_LOAD * ((freq - F0) / F0) * SB;
    if (Heff > 0) dfdt = ((pSync + pIbr - pLA - pDamp) / SB) * F0 / (2 * Heff); else dfdt = 0;
    freq += dfdt * dt; freq = Math.max(freq, 50);

    fA[i] = freq; rA[i] = dfdt; psA[i] = pSync; piA[i] = pIbr;
    socA[i] = gfmFleet ? gfmFleet.soc : 0; pLoadA[i] = pLA;
  }
  let nadir = F0, nT = 0, mR = 0;
  for (let i = 0; i < N; i++) { if (fA[i] < nadir) { nadir = fA[i]; nT = tA[i]; } if (tA[i] > 1 && tA[i] < 5 && Math.abs(rA[i]) > mR) mR = Math.abs(rA[i]); }
  return { tA, fA, rA, psA, piA, socA, pLoadA, N, nadir, nT, mR, ss: fA[N - 1] };
}

// ════════════════════════════════════════════════════════════
// HECO 138 kV TOPOLOGY
// ════════════════════════════════════════════════════════════
const SUBS = {
  "KAHE 1": { x: 126.5, y: 258.0, plant: "kahe" }, "KAHE 2": { x: 142.2, y: 247.4, plant: "kahe" },
  "CEIP": { x: 148.4, y: 281.2, plant: "campbell" }, "AES": { x: 145.1, y: 290.3, plant: "kalaeloa" },
  "KALAELOA": { x: 149.2, y: 296.0, plant: "kalaeloa" }, "EWA NUI": { x: 187.0, y: 246.5 },
  "WAHIAWA": { x: 202.6, y: 161.1 }, "WAIAU 1": { x: 249.7, y: 234.3, plant: "waiau" },
  "MAKALAPA": { x: 274.1, y: 256.8 }, "KEWALO": { x: 318.2, y: 284.8 },
  "HAIAWA": { x: 306.9, y: 241.2 }, "KOOLAU": { x: 348, y: 230 }, "PUKELE": { x: 377.4, y: 288.4 },
};
const TX = [
  { s1: "HAIAWA", s2: "MAKALAPA", pts: [[274.1, 256.8], [275.8, 256.1], [278.3, 254.3], [280.2, 252.1], [283.5, 251.4], [283.9, 247.1], [285.0, 244.7], [287.7, 241.2], [291.2, 239.0], [297.3, 244.0], [300.6, 243.1], [303.7, 242.1], [306.3, 240.9], [306.9, 241.2]] },
  { s1: "HAIAWA", s2: "KEWALO", pts: [[318.2, 284.8], [318.1, 284.2], [312.2, 280.9], [311.5, 276.2], [311.6, 272.7], [314.5, 270.3], [312.2, 266.4], [316.5, 263.7], [321.7, 254.5], [311.4, 241.3], [306.9, 242.1], [306.9, 241.2]] },
  { s1: "KOOLAU", s2: "PUKELE", pts: [[348, 230], [372.1, 242.3], [375.8, 246.8], [386.3, 254.5], [390.2, 269.0], [382.4, 282.4], [377.1, 287.8], [377.4, 288.4]] },
  { s1: "KALAELOA", s2: "EWA NUI", pts: [[149.2, 296.0], [150.6, 293.5], [151.7, 288.9], [150.7, 286.0], [153.7, 284.1], [154.9, 283.0], [155.4, 280.3], [158.2, 279.8], [165.3, 281.3], [169.3, 280.3], [181.0, 276.5], [184.3, 274.2], [181.6, 269.2], [181.5, 266.9], [183.7, 264.2], [184.4, 261.7], [184.3, 259.0], [183.1, 256.3], [180.1, 253.6], [181.0, 252.0], [182.3, 250.0], [184.6, 248.5], [187.3, 246.7], [187.0, 246.5]] },
  { s1: "EWA NUI", s2: "CEIP", pts: [[187.0, 246.5], [182.2, 250.0], [181.0, 252.0], [180.1, 253.6], [183.5, 257.2], [184.4, 259.9], [184.2, 262.7], [181.9, 266.3], [181.3, 268.5], [184.8, 275.1], [168.5, 280.5], [165.3, 281.2], [159.5, 280.1], [152.4, 278.4], [149.4, 278.0]] },
  { s1: "AES", s2: "CEIP", pts: [[149.4, 278.0], [145.9, 279.1], [143.6, 286.0], [144.2, 288.3], [144.8, 293.2], [144.4, 294.0]] },
  { s1: "AES", s2: "CEIP", pts: [[144.4, 294.0], [143.9, 293.2], [143.1, 288.2], [141.9, 282.4], [145.4, 278.8], [148.5, 277.4], [149.4, 278.0]] },
  { s1: "CEIP", s2: "KAHE 1", pts: [[149.4, 278.0], [151.1, 274.8], [153.2, 271.4], [149.6, 269.6], [144.0, 268.0], [139.3, 266.6], [141.5, 258.8], [134.4, 258.4], [127.3, 258.4], [126.7, 258.2]] },
  { s1: "CEIP", s2: "KAHE 1", pts: [[149.4, 278.0], [151.2, 274.8], [153.3, 271.4], [149.7, 269.5], [144.0, 268.0], [139.4, 266.6], [141.6, 258.7], [134.4, 258.4], [127.3, 258.3], [126.7, 258.2]] },
  { s1: "KAHE 1", s2: "KAHE 2", pts: [[126.7, 258.2], [126.8, 258.0], [126.6, 257.6], [126.3, 257.3]] },
  { s1: "MAKALAPA", s2: "KEWALO", pts: [[318.2, 284.8], [316.3, 283.3], [313.0, 282.5], [310.6, 282.7], [308.1, 277.4], [303.9, 274.5], [302.7, 273.1], [302.6, 270.9], [300.3, 269.5], [294.8, 267.3], [294.0, 269.5], [292.4, 271.7], [285.5, 271.2], [279.5, 270.2], [273.1, 269.0], [270.8, 265.5], [270.1, 262.8], [271.0, 259.1], [272.0, 258.9], [274.1, 256.8]] },
  { s1: "WAIAU 1", s2: "KOOLAU", pts: [[348, 230], [367.7, 238.0], [353.1, 233.0], [337.1, 234.5], [317.6, 241.3], [309.1, 239.2], [305.9, 241.5], [296.0, 231.7], [286.8, 225.7], [268.7, 218.0], [256.6, 220.5], [250.3, 229.1], [249.5, 233.5], [249.7, 234.3]] },
  { s1: "KOOLAU", s2: "PUKELE", pts: [[377.4, 288.4], [379.4, 289.4], [385.5, 280.0], [390.7, 269.3], [392.4, 265.6], [384.1, 248.9], [374.9, 242.9], [373.6, 238.8], [348, 230]] },
  { s1: "KAHE 2", s2: "WAIAU 1", pts: [[126.3, 257.3], [138.1, 255.4], [145.9, 250.6], [160.9, 238.9], [165.2, 230.5], [180.7, 220.7], [195.0, 212.0], [203.5, 208.6], [213.1, 207.7], [220.1, 198.2], [233.5, 192.8], [241.6, 193.6], [257.0, 204.4], [266.0, 205.1], [262.9, 215.9], [256.6, 220.4], [250.3, 229.1], [249.4, 233.5], [249.7, 234.3]] },
  { s1: "KALAELOA", s2: "AES", pts: [[149.2, 296.0], [145.4, 296.1], [144.5, 294.4], [144.4, 294.0]] },
  { s1: "KAHE 2", s2: "HAIAWA", pts: [[126.3, 257.3], [138.3, 254.9], [148.2, 248.1], [153.4, 244.1], [160.7, 238.6], [171.9, 225.8], [186.2, 217.1], [196.9, 210.6], [211.5, 206.9], [218.4, 199.7], [232.4, 192.5], [244.7, 197.4], [252.3, 197.4], [261.4, 199.4], [275.5, 208.6], [287.7, 214.7], [300.5, 228.3], [306.9, 241.2]] },
  { s1: "EWA NUI", s2: "WAIAU 1", pts: [[187.0, 246.5], [185.8, 245.0], [185.5, 243.2], [191.1, 241.1], [194.7, 238.9], [204.2, 234.0], [208.6, 232.4], [212.0, 230.1], [215.5, 227.9], [220.2, 226.8], [225.9, 227.3], [228.7, 225.2], [234.3, 228.3], [244.9, 232.0], [248.1, 233.3], [247.5, 234.4], [249.7, 234.3]] },
  { s1: "WAIAU 1", s2: "MAKALAPA", pts: [[249.7, 234.3], [253.5, 235.6], [260.0, 238.5], [266.8, 241.5], [268.8, 242.9], [269.9, 244.3], [269.1, 246.8], [266.4, 251.6], [266.4, 253.9], [268.2, 260.0], [270.6, 260.4], [271.3, 258.2], [273.2, 257.1], [274.1, 256.8]] },
  { s1: "WAIAU 1", s2: "KOOLAU", pts: [[348, 230], [358.4, 235.0], [344.8, 235.1], [334.2, 235.9], [316.2, 243.0], [309.1, 239.5], [305.9, 241.6], [295.7, 231.8], [273.6, 219.7], [262.2, 219.9], [256.3, 233.1], [250.1, 233.6], [249.7, 234.3]] },
  { s1: "WAHIAWA", s2: "WAIAU 1", pts: [[249.7, 234.3], [254.6, 232.9], [259.4, 221.3], [269.8, 218.4], [265.7, 202.0], [255.6, 188.0], [248.2, 175.5], [240.8, 168.4], [234.8, 165.9], [217.9, 161.5], [202.9, 161.4], [202.6, 161.1]] },
  { s1: "KAHE 2", s2: "HAIAWA", pts: [[126.3, 257.3], [138.1, 255.4], [145.9, 250.6], [160.9, 238.9], [165.2, 230.5], [180.7, 220.7], [195.0, 212.0], [203.5, 208.6], [213.1, 207.7], [220.1, 198.2], [233.5, 192.8], [241.6, 193.6], [257.0, 204.4], [268.9, 204.5], [282.3, 213.7], [290.4, 220.7], [299.5, 228.9], [306.9, 241.2]] },
  { s1: "HAIAWA", s2: "KOOLAU", pts: [[306.9, 241.2], [308.8, 238.8], [313.5, 233.2], [330.4, 225.0], [334.0, 218.0], [339.9, 220.0], [345.6, 225.0], [348, 230]] },
];
const OAHU = "M 25.2,147.2 L 33.7,144.0 L 41.6,141.8 L 54.7,143.4 L 67.8,142.9 L 80.9,141.3 L 94.0,139.7 L 107.1,138.0 L 118.8,136.4 L 128.7,134.3 L 136.5,132.1 L 141.8,127.8 L 146.3,123.5 L 156.8,113.2 L 164.7,108.3 L 176.4,100.8 L 187.6,93.2 L 198.7,87.8 L 209.2,82.4 L 220.3,77.0 L 231.4,73.8 L 243.2,71.6 L 255.0,73.8 L 264.2,78.1 L 272.7,83.5 L 279.2,91.6 L 285.8,100.8 L 290.3,110.5 L 294.3,121.3 L 297.5,132.1 L 302.1,144.0 L 305.4,154.8 L 308.7,165.6 L 311.9,175.3 L 315.2,183.4 L 318.5,191.5 L 323.1,198.0 L 328.3,205.0 L 333.5,210.4 L 340.1,213.1 L 347.9,214.2 L 354.5,215.8 L 359.7,219.6 L 357.1,223.9 L 351.2,226.6 L 355.8,232.0 L 362.3,239.0 L 368.9,245.5 L 374.1,252.0 L 387.2,261.7 L 397.0,269.8 L 406.9,279.0 L 414.7,286.0 L 423.2,291.4 L 429.8,295.2 L 433.0,299.5 L 431.7,304.9 L 426.5,309.2 L 418.6,311.9 L 410.1,313.5 L 400.3,314.6 L 390.5,315.1 L 382.0,314.1 L 374.1,313.0 L 364.3,313.5 L 354.5,314.1 L 344.7,314.6 L 334.9,315.7 L 328.3,317.8 L 323.1,318.9 L 319.1,316.8 L 315.2,313.5 L 310.0,310.3 L 303.4,307.6 L 295.6,306.0 L 285.8,303.8 L 275.9,301.6 L 266.1,300.0 L 256.3,298.4 L 246.5,296.2 L 238.0,294.1 L 231.4,291.9 L 224.9,289.8 L 217.0,287.6 L 209.2,284.4 L 202.6,280.6 L 197.4,276.8 L 192.2,272.5 L 189.5,268.2 L 187.6,264.4 L 190.8,268.2 L 196.1,273.6 L 189.5,279.0 L 179.1,282.2 L 167.9,284.4 L 156.8,286.5 L 145.0,288.1 L 133.2,288.7 L 122.1,288.1 L 113.6,286.5 L 107.1,288.7 L 102.5,292.5 L 99.2,297.9 L 95.9,302.7 L 91.3,304.9 L 84.8,302.7 L 78.3,299.5 L 71.7,295.2 L 65.2,289.8 L 61.2,283.3 L 58.6,275.2 L 54.7,264.4 L 52.1,253.6 L 48.1,242.8 L 44.2,232.0 L 40.3,221.2 L 36.4,210.4 L 33.7,199.6 L 31.1,188.8 L 28.5,178.0 L 25.9,167.2 L 23.3,156.4 L 22.0,149.4 L 23.9,147.2 L 25.2,147.2 Z";
const PLANTS = [
  { id: "kahe", name: "Kahe", fuel: "Oil/Steam", mw: 650, x: 120, y: 258, col: "#e8854a" },
  { id: "waiau", name: "Waiau", fuel: "Oil/Steam", mw: 402, x: 250, y: 228, col: "#e8854a" },
  { id: "campbell", name: "Campbell Ind.", fuel: "Oil/Gas CT", mw: 130, x: 145, y: 275, col: "#d4a04a" },
  { id: "kalaeloa", name: "Kalaeloa", fuel: "CC", mw: 208, x: 147, y: 298, col: "#c98540" },
  { id: "schofield", name: "Schofield", fuel: "Biofuel", mw: 50, x: 195, y: 172, col: "#8bab5e" },
];
const BESS_SITES = [
  { id: "kes", name: "Kapolei ES", mw: 185, x: 155, y: 290, col: "#4facfe" },
  { id: "waiawa", name: "Waiawa", mw: 75, x: 280, y: 225, col: "#4facfe" },
  { id: "mililani", name: "Mililani I", mw: 39, x: 185, y: 165, col: "#43e97b" },
  { id: "hoohana", name: "Ho'ohana", mw: 36, x: 210, y: 155, col: "#43e97b" },
];
const LOAD_CENTERS = [
  { name: "Honolulu", x: 310, y: 278, mw: 320 }, { name: "Pearl City", x: 260, y: 248, mw: 170 },
  { name: "Kailua", x: 370, y: 262, mw: 95 }, { name: "Kapolei", x: 170, y: 260, mw: 130 },
  { name: "Mililani", x: 210, y: 188, mw: 105 }, { name: "Kaneohe", x: 330, y: 215, mw: 85 },
  { name: "Wahiawa", x: 205, y: 148, mw: 75 }, { name: "North Shore", x: 200, y: 105, mw: 55 },
];

// ════════════════════════════════════════════════════════════
// ERAS + SCENARIOS
// ════════════════════════════════════════════════════════════
// syncMW sized to cover base load + spinning reserve (N-1 criterion).
// Pre-BESS eras carry the full Kalaeloa 208 MW reserve on sync.
// Eras 4-5: BESS covers most of the N-1 reserve, so sync reserve ≈ 50 MW.
const ERAS = [
  { label: "Era 1: Pre-Rule 14H (≤2001)", desc: "All synchronous, no customer PV", syncMW: 1400, pLoad: 1150, build: () => [] },
  { label: "Era 2: 1547-2003 (2001–12)", desc: "300 MW legacy PV — cascading trip risk", syncMW: 1150, pLoad: 1150, build: () => [mkLeg(300)] },
  { label: "Era 3: Ride-Through (2012–17)", desc: "Mixed legacy/RT/smart fleet", syncMW: 1050, pLoad: 1150, build: () => [mkLeg(150), mkRT(200), mkSm(150)] },
  { label: "Era 4: 1547-2018 + KES (2018–26)", desc: "Smart inverters + 185 MW GFM BESS", syncMW: 750, pLoad: 1150, build: () => [mkLeg(40), mkRT(200), mkSm(422), mkGFM(185)] },
  { label: "Era 5: GFM-Forward (2027+)", desc: "Expanded GFM, reduced sync reliance", syncMW: 650, pLoad: 1150, build: () => [mkSm(800), mkGFM(370)] },
];

const SCENARIOS = [
  { id: "s_cip",      cat: "sync",  label: "CIP CT-1 trip",         sub: "30 MW combustion turbine",                  plant: "campbell", icon: "\u26A1", color: "#ff6b35", tEnd: 60,  make: () => makeSyncTrip(30) },
  { id: "s_waiau",    cat: "sync",  label: "Waiau unit trip",       sub: "50 MW steam unit",                          plant: "waiau",    icon: "\u26A1", color: "#ff6b35", tEnd: 60,  make: () => makeSyncTrip(50) },
  { id: "s_kahe",     cat: "sync",  label: "Kahe unit trip",        sub: "90 MW oil/steam unit",                      plant: "kahe",     icon: "\u26A1", color: "#ff6b35", tEnd: 60,  make: () => makeSyncTrip(90) },
  { id: "s_kalaeloa", cat: "sync",  label: "Kalaeloa full trip",    sub: "208 MW combined cycle \u2014 largest N-1",  plant: "kalaeloa", icon: "\u26A1", color: "#ff6b35", tEnd: 60,  make: () => makeSyncTrip(208) },
  { id: "r_small",    cat: "solar", label: "Scattered clouds",      sub: "30 MW ramp over 60 s",                      icon: "\u2601", color: "#ffc400", tEnd: 120, make: () => makeSolarRamp(30, 60) },
  { id: "r_med",      cat: "solar", label: "Broad cloud front",     sub: "60 MW ramp over 45 s",                      icon: "\u2601", color: "#ffc400", tEnd: 120, make: () => makeSolarRamp(60, 45) },
  { id: "r_large",    cat: "solar", label: "Fast storm front",      sub: "100 MW ramp over 30 s",                     icon: "\u2601", color: "#ffc400", tEnd: 120, make: () => makeSolarRamp(100, 30) },
  { id: "r_worst",    cat: "solar", label: "Severe worst-case",     sub: "150 MW ramp over 20 s",                     icon: "\u2601", color: "#ffc400", tEnd: 120, make: () => makeSolarRamp(150, 20) },
  { id: "i_small",    cat: "ibr",   label: "Small DER dropout",     sub: "20 MW IBR protection trip",                 icon: "\uD83D\uDD0C", color: "#d62728", tEnd: 60,  make: () => makeIBRTrip(20) },
  { id: "i_med",      cat: "ibr",   label: "Moderate DER dropout",  sub: "50 MW IBR sympathetic trip",                icon: "\uD83D\uDD0C", color: "#d62728", tEnd: 60,  make: () => makeIBRTrip(50) },
  { id: "i_large",    cat: "ibr",   label: "Large DER dropout",     sub: "100 MW (Blue Cut Fire-scale)",              icon: "\uD83D\uDD0C", color: "#d62728", tEnd: 90,  make: () => makeIBRTrip(100) },
  { id: "b_out",      cat: "bess",  label: "BESS forced outage",    sub: "GFM BESS trips \u2014 loss of virtual inertia", icon: "\uD83D\uDD0B", color: "#4facfe", tEnd: 90,  make: () => makeBESSOutage() },
  { id: "c_mod",      cat: "compound", label: "Cloud \u2192 Kahe trip",     sub: "60 MW ramp then 90 MW sync trip at 30 s",   icon: "\u26D3", color: "#bb86fc", tEnd: 120, make: () => makeCompound([makeSolarRamp(60, 30, 1), makeSyncTrip(90, 30)]) },
  { id: "c_sev",      cat: "compound", label: "Storm \u2192 Kalaeloa trip", sub: "100 MW ramp then 208 MW trip at 25 s",      icon: "\u26D3", color: "#bb86fc", tEnd: 120, make: () => makeCompound([makeSolarRamp(100, 25, 1), makeSyncTrip(208, 25)]) },
];

const CATS = [
  { id: "sync",     label: "Sync Trip",       icon: "\u26A1", color: "#ff6b35" },
  { id: "solar",    label: "Solar Ramp",       icon: "\u2601", color: "#ffc400" },
  { id: "ibr",      label: "IBR Trip",         icon: "\uD83D\uDD0C", color: "#d62728" },
  { id: "bess",     label: "BESS Outage",      icon: "\uD83D\uDD0B", color: "#4facfe" },
  { id: "compound", label: "Compound",         icon: "\u26D3", color: "#bb86fc" },
];

// ════════════════════════════════════════════════════════════
// CHARTS
// ════════════════════════════════════════════════════════════
function FreqChart({ r, h = 220 }) {
  const ref = useRef(null);
  useEffect(() => {
    const c = ref.current; if (!c || !r) return;
    const ctx = c.getContext("2d"), W = c.width, H = c.height;
    const m = { t: 22, r: 12, b: 28, l: 44 }, pw = W - m.l - m.r, ph = H - m.t - m.b;
    ctx.clearRect(0, 0, W, H);
    const { fA, tA, N, nadir, nT } = r;
    let fMin = 60, fMax = 60;
    for (let i = 0; i < N; i++) { if (fA[i] < fMin) fMin = fA[i]; if (fA[i] > fMax) fMax = fA[i]; }
    fMin = Math.floor((fMin - 0.15) * 5) / 5; fMax = Math.ceil((fMax + 0.15) * 5) / 5;
    const tMax = tA[N - 1];
    const xS = t => m.l + (t / tMax) * pw, yS = f => m.t + (1 - (f - fMin) / (fMax - fMin)) * ph;
    ctx.strokeStyle = "rgba(255,255,255,0.04)"; ctx.lineWidth = 1;
    for (let f = Math.ceil(fMin * 5) / 5; f <= fMax; f += 0.2) { ctx.beginPath(); ctx.moveTo(m.l, yS(f)); ctx.lineTo(W - m.r, yS(f)); ctx.stroke(); }
    [[60, "rgba(255,255,255,0.1)"], [59.3, "rgba(255,170,0,0.35)"], [58.9, "rgba(255,120,0,0.25)"], [58.5, "rgba(255,60,0,0.25)"], [57, "rgba(255,23,68,0.3)"]].forEach(([fv, col]) => {
      if (fv >= fMin && fv <= fMax) { ctx.strokeStyle = col; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(m.l, yS(fv)); ctx.lineTo(W - m.r, yS(fv)); ctx.stroke(); ctx.setLineDash([]); }
    });
    const grad = ctx.createLinearGradient(0, yS(fMax), 0, yS(fMin));
    grad.addColorStop(0, "#00e676"); grad.addColorStop(0.4, "#c6ff00"); grad.addColorStop(0.7, "#ffc400"); grad.addColorStop(1, "#ff1744");
    ctx.strokeStyle = grad; ctx.lineWidth = 2; ctx.beginPath();
    const sk = Math.max(1, Math.floor(N / 600));
    for (let i = 0; i < N; i += sk) { i === 0 ? ctx.moveTo(xS(tA[i]), yS(fA[i])) : ctx.lineTo(xS(tA[i]), yS(fA[i])); } ctx.stroke();
    ctx.strokeStyle = "rgba(0,230,118,0.06)"; ctx.lineWidth = 5; ctx.beginPath();
    for (let i = 0; i < N; i += sk) { i === 0 ? ctx.moveTo(xS(tA[i]), yS(fA[i])) : ctx.lineTo(xS(tA[i]), yS(fA[i])); } ctx.stroke();
    ctx.fillStyle = "#ff1744"; ctx.beginPath(); ctx.arc(xS(nT), yS(nadir), 3.5, 0, Math.PI * 2); ctx.fill();
    ctx.font = "bold 9px monospace"; ctx.textAlign = "left"; ctx.fillText(nadir.toFixed(2) + " Hz", xS(nT) + 6, yS(nadir) + 3);
    ctx.fillStyle = "rgba(255,255,255,0.3)"; ctx.font = "9px monospace"; ctx.textAlign = "right";
    for (let f = Math.ceil(fMin * 5) / 5; f <= fMax; f += 0.2) ctx.fillText(f.toFixed(1), m.l - 4, yS(f) + 3);
    ctx.textAlign = "center"; const tStep = tMax <= 30 ? 5 : tMax <= 60 ? 10 : 20;
    for (let t = 0; t <= tMax; t += tStep) ctx.fillText(t + "s", xS(t), H - m.b + 12);
  }, [r]);
  return <canvas ref={ref} width={520} height={h} style={{ width: "100%", height: "auto", display: "block" }} />;
}
function PowerChart({ r, h = 140 }) {
  const ref = useRef(null);
  useEffect(() => {
    const c = ref.current; if (!c || !r) return;
    const ctx = c.getContext("2d"), W = c.width, H = c.height;
    const m = { t: 22, r: 12, b: 28, l: 44 }, pw = W - m.l - m.r, ph = H - m.t - m.b;
    ctx.clearRect(0, 0, W, H);
    const { tA, psA, piA, pLoadA, N } = r; const tMax = tA[N - 1];
    let pMax = 0;
    for (let i = 0; i < N; i++) { const s = psA[i] + piA[i]; if (s > pMax) pMax = s; if (pLoadA[i] > pMax) pMax = pLoadA[i]; }
    pMax = Math.ceil(pMax / 100) * 100 + 50;
    const xS = t => m.l + (t / tMax) * pw, yS = p => m.t + (1 - p / pMax) * ph;
    const sk = Math.max(1, Math.floor(N / 500));
    ctx.fillStyle = "rgba(139,90,43,0.12)"; ctx.beginPath(); ctx.moveTo(xS(tA[0]), yS(0));
    for (let i = 0; i < N; i += sk) ctx.lineTo(xS(tA[i]), yS(Math.max(0, psA[i]))); ctx.lineTo(xS(tA[N - 1]), yS(0)); ctx.fill();
    [[psA, "#8c564b", 1.5], [piA, "#2ca02c", 1.5], [pLoadA, "#82b1ff", 1]].forEach(([arr, col, lw]) => {
      ctx.strokeStyle = col; ctx.lineWidth = lw; ctx.beginPath();
      for (let i = 0; i < N; i += sk) { i === 0 ? ctx.moveTo(xS(tA[i]), yS(arr[i])) : ctx.lineTo(xS(tA[i]), yS(arr[i])); } ctx.stroke();
    });
    [[m.l + 5, "#8c564b", "Sync"], [m.l + 42, "#2ca02c", "IBR"], [m.l + 76, "#82b1ff", "Load"]].forEach(([lx, col, lbl]) => {
      ctx.fillStyle = col; ctx.fillRect(lx, 5, 8, 3); ctx.fillStyle = "rgba(255,255,255,0.4)"; ctx.font = "8px monospace"; ctx.textAlign = "left"; ctx.fillText(lbl, lx + 11, 10);
    });
    ctx.fillStyle = "rgba(255,255,255,0.3)"; ctx.font = "9px monospace"; ctx.textAlign = "right";
    for (let v = 0; v <= pMax; v += 200) ctx.fillText(v + "", m.l - 4, yS(v) + 3);
    ctx.textAlign = "center"; const tStep = tMax <= 30 ? 5 : tMax <= 60 ? 10 : 20;
    for (let t = 0; t <= tMax; t += tStep) ctx.fillText(t + "s", xS(t), H - m.b + 12);
  }, [r]);
  return <canvas ref={ref} width={520} height={h} style={{ width: "100%", height: "auto", display: "block" }} />;
}
function SOCChart({ r, h = 90 }) {
  const ref = useRef(null);
  useEffect(() => {
    const c = ref.current; if (!c || !r) return;
    const ctx = c.getContext("2d"), W = c.width, H = c.height;
    const m = { t: 16, r: 12, b: 24, l: 44 }, pw = W - m.l - m.r, ph = H - m.t - m.b;
    ctx.clearRect(0, 0, W, H);
    const { tA, socA, N } = r; const tMax = tA[N - 1];
    if (!socA || socA[0] === 0) { ctx.fillStyle = "rgba(255,255,255,0.15)"; ctx.font = "10px monospace"; ctx.textAlign = "center"; ctx.fillText("No BESS in fleet", W / 2, H / 2); return; }
    let sMin = 1, sMax = 0;
    for (let i = 0; i < N; i++) { if (socA[i] < sMin) sMin = socA[i]; if (socA[i] > sMax) sMax = socA[i]; }
    sMin = Math.max(0, Math.floor(sMin * 20) / 20 - 0.05); sMax = Math.min(1, Math.ceil(sMax * 20) / 20 + 0.05);
    const xS = t => m.l + (t / tMax) * pw, yS = s => m.t + (1 - (s - sMin) / (sMax - sMin)) * ph;
    const sk = Math.max(1, Math.floor(N / 400));
    ctx.strokeStyle = "#4facfe"; ctx.lineWidth = 1.5; ctx.beginPath();
    for (let i = 0; i < N; i += sk) { i === 0 ? ctx.moveTo(xS(tA[i]), yS(socA[i])) : ctx.lineTo(xS(tA[i]), yS(socA[i])); } ctx.stroke();
    ctx.fillStyle = "rgba(79,172,254,0.06)"; ctx.beginPath(); ctx.moveTo(xS(tA[0]), yS(sMin));
    for (let i = 0; i < N; i += sk) ctx.lineTo(xS(tA[i]), yS(socA[i])); ctx.lineTo(xS(tA[N - 1]), yS(sMin)); ctx.fill();
    ctx.fillStyle = "rgba(255,255,255,0.3)"; ctx.font = "8px monospace"; ctx.textAlign = "right";
    for (let s = Math.ceil(sMin * 10) / 10; s <= sMax; s += 0.1) ctx.fillText((s * 100).toFixed(0) + "%", m.l - 4, yS(s) + 3);
    ctx.fillStyle = "#4facfe"; ctx.font = "8px monospace"; ctx.textAlign = "left"; ctx.fillText("BESS SOC", m.l + 2, m.t - 4);
  }, [r]);
  return <canvas ref={ref} width={520} height={h} style={{ width: "100%", height: "auto", display: "block" }} />;
}

// ════════════════════════════════════════════════════════════
// MAIN APP
// ════════════════════════════════════════════════════════════
const BX = { background: "rgba(255,255,255,0.02)", border: "1px solid rgba(255,255,255,0.06)", borderRadius: 10, padding: 14 };
const LB = { fontSize: 9, letterSpacing: 2, color: "rgba(255,255,255,0.24)", marginBottom: 8 };

export default function App() {
  const [era, setEra] = useState(3);
  const [selectedCat, setSelectedCat] = useState("sync");
  const [selectedScenario, setSelectedScenario] = useState("s_kahe");
  const [runKey, setRunKey] = useState(null);
  const [animT, setAnimT] = useState(null);

  const e = ERAS[era];
  const scenario = SCENARIOS.find(s => s.id === selectedScenario);
  const catScenarios = SCENARIOS.filter(s => s.cat === selectedCat);

  const result = useMemo(() => {
    if (!runKey || !scenario) return null;
    return simulate({ syncMW: e.syncMW, fleets: e.build(), pLoad: e.pLoad, contingency: scenario.make(), tEnd: scenario.tEnd, dt: 0.002 });
  }, [runKey]);

  useEffect(() => {
    if (!runKey || !scenario) { setAnimT(null); return; }
    setAnimT(0); const t0 = Date.now(); const tEnd = scenario.tEnd;
    const speed = tEnd > 60 ? 4 : 2;
    const iv = setInterval(() => { const t = Math.min((Date.now() - t0) / 1000 * speed, tEnd); setAnimT(t); if (t >= tEnd) clearInterval(iv); }, 50);
    return () => clearInterval(iv);
  }, [runKey]);

  const curFreq = useMemo(() => {
    if (!result || animT === null) return F0;
    return result.fA[Math.min(Math.floor(animT / 0.002), result.N - 1)];
  }, [result, animT]);

  const fCol = Math.abs(curFreq - F0) < 0.3 ? "#00e676" : Math.abs(curFreq - F0) < 0.7 ? "#ffc400" : Math.abs(curFreq - F0) < 1.2 ? "#ff9100" : "#ff1744";
  const isRunning = runKey !== null;
  const highlightPlant = isRunning && scenario?.plant || null;

  const handleRun = () => setRunKey(Date.now());
  const handleSelectCat = (cat) => { setSelectedCat(cat); setSelectedScenario(SCENARIOS.find(s => s.cat === cat)?.id || null); setRunKey(null); };
  const handleSelectScenario = (id) => { setSelectedScenario(id); setRunKey(null); };
  const handleEra = (i) => { setEra(i); setRunKey(null); };
  const handlePlantClick = (id) => {
    const sc = SCENARIOS.find(s => s.cat === "sync" && s.plant === id);
    if (sc) { setSelectedCat("sync"); setSelectedScenario(sc.id); setRunKey(Date.now()); }
  };

  const lineGlow = isRunning ? (curFreq < 59.3 ? "rgba(255,23,68,0.5)" : curFreq < 59.5 ? "rgba(255,196,0,0.4)" : "rgba(0,230,118,0.3)") : "rgba(255,255,255,0.14)";

  return (
    <div style={{ minHeight: "100vh", background: "linear-gradient(145deg,#060a11 0%,#0a1018 40%,#080d13 100%)", color: "#e0e0e0", fontFamily: "'JetBrains Mono','Fira Code',monospace", padding: 16, boxSizing: "border-box" }}>
      <style>{`
        @keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
        @keyframes fadeUp{from{opacity:0;transform:translateY(6px)}to{opacity:1;transform:translateY(0)}}
        @keyframes tripFlash{0%{opacity:1}30%{opacity:.15}60%{opacity:.7}100%{opacity:.5}}
      `}</style>

      {/* HEADER */}
      <div style={{ display: "flex", alignItems: "center", gap: 14, marginBottom: 16, borderBottom: "1px solid rgba(0,230,118,0.1)", paddingBottom: 12 }}>
        <div style={{ width: 9, height: 9, borderRadius: "50%", background: isRunning ? "#ff1744" : "#00e676", boxShadow: `0 0 10px ${isRunning ? "#ff1744" : "#00e676"}`, animation: "pulse 2s infinite" }} />
        <div>
          <h1 style={{ margin: 0, fontSize: 16, fontWeight: 700, letterSpacing: 2, color: "#fff" }}>OAHU TRANSMISSION DIGITAL TWIN</h1>
          <div style={{ fontSize: 8, color: "rgba(255,255,255,0.22)", letterSpacing: 3, marginTop: 1 }}>HECO 138 kV · CONTINGENCY FRAMEWORK v2.1 · ELEN 4510</div>
        </div>
        <div style={{ marginLeft: "auto", textAlign: "right" }}>
          <div style={{ fontSize: 26, fontWeight: 700, color: fCol, fontVariantNumeric: "tabular-nums", lineHeight: 1 }}>{curFreq.toFixed(2)}</div>
          <div style={{ fontSize: 8, color: "rgba(255,255,255,0.25)", letterSpacing: 1 }}>SYSTEM Hz</div>
        </div>
      </div>

      <div style={{ display: "grid", gridTemplateColumns: "1fr 360px", gap: 14, maxWidth: 1200 }}>
        {/* LEFT COLUMN */}
        <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
          <div style={{ ...BX, padding: 16 }}>
            <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 8 }}>
              <div style={LB}>138 kV ONE-LINE — OAHU (HIFLD DATA)</div>
              {isRunning && scenario && <div style={{ fontSize: 9, color: scenario.color, animation: "pulse 1.5s infinite" }}>{scenario.icon} {scenario.label.toUpperCase()}</div>}
            </div>
            <svg viewBox="0 0 500 340" style={{ width: "100%", background: "rgba(0,10,20,0.4)", borderRadius: 8 }}>
              <defs>
                <radialGradient id="oc" cx="50%" cy="50%" r="55%"><stop offset="0%" stopColor="rgba(0,80,140,0.06)" /><stop offset="100%" stopColor="transparent" /></radialGradient>
                <filter id="pg"><feGaussianBlur stdDeviation="2" result="b" /><feMerge><feMergeNode in="b" /><feMergeNode in="SourceGraphic" /></feMerge></filter>
                <filter id="bg"><feGaussianBlur stdDeviation="1.2" result="b" /><feMerge><feMergeNode in="b" /><feMergeNode in="SourceGraphic" /></feMerge></filter>
                <filter id="tg"><feGaussianBlur stdDeviation="0.8" result="b" /><feMerge><feMergeNode in="b" /><feMergeNode in="SourceGraphic" /></feMerge></filter>
              </defs>
              <rect width="500" height="340" fill="url(#oc)" />
              <path d={OAHU} fill="rgba(255,255,255,0.025)" stroke="rgba(255,255,255,0.08)" strokeWidth="0.8" />
              {[["Leeward", 80, 242], ["Ewa\nPlain", 170, 230], ["Central", 215, 200], ["North Shore", 205, 78], ["Windward", 410, 195], ["Honolulu", 330, 300]].map(([l, x, y]) => (
                <text key={l} x={x} y={y} textAnchor="middle" fill="rgba(255,255,255,0.055)" fontSize="11" fontFamily="monospace" fontWeight="bold">{l}</text>
              ))}
              {TX.map((ln, i) => {
                const d = "M " + ln.pts.map(p => p[0] + "," + p[1]).join(" L ");
                const s1 = SUBS[ln.s1], s2 = SUBS[ln.s2];
                const atTripped = highlightPlant && ((s1?.plant === highlightPlant) || (s2?.plant === highlightPlant));
                return <path key={i} d={d} fill="none" stroke={atTripped ? "rgba(255,23,68,0.4)" : lineGlow} strokeWidth={atTripped ? 1.2 : 1.0} strokeLinecap="round" style={atTripped ? { animation: "tripFlash 1s ease-in-out" } : {}} filter="url(#tg)" />;
              })}
              {Object.entries(SUBS).map(([name, s]) => {
                const atTripped = highlightPlant && s.plant === highlightPlant;
                return (<g key={name}>
                  <circle cx={s.x} cy={s.y} r={3.5} fill={atTripped ? "rgba(255,23,68,0.15)" : "rgba(255,255,255,0.06)"} stroke={atTripped ? "#ff1744" : "rgba(255,255,255,0.15)"} strokeWidth={0.5} />
                  <circle cx={s.x} cy={s.y} r={1.5} fill={atTripped ? "#ff1744" : "rgba(255,255,255,0.25)"} />
                  <text x={s.x} y={s.y - 6} textAnchor="middle" fill="rgba(255,255,255,0.3)" fontSize="5" fontFamily="monospace">{name}</text>
                </g>);
              })}
              {LOAD_CENTERS.map(lc => {
                const r = Math.sqrt(lc.mw) / 3.2;
                return (<g key={lc.name}>
                  <circle cx={lc.x} cy={lc.y} r={r} fill="rgba(130,177,255,0.08)" stroke="rgba(130,177,255,0.2)" strokeWidth={0.4} />
                  <text x={lc.x} y={lc.y + r + 7} textAnchor="middle" fill="rgba(130,177,255,0.3)" fontSize="5" fontFamily="monospace">{lc.name}</text>
                  <text x={lc.x} y={lc.y + 2} textAnchor="middle" fill="rgba(130,177,255,0.35)" fontSize="4.5" fontFamily="monospace">{lc.mw}</text>
                </g>);
              })}
              {BESS_SITES.map(b => (
                <g key={b.id}>
                  <rect x={b.x - 6} y={b.y - 4.5} width={12} height={9} rx={2} fill={b.col + "15"} stroke={b.col} strokeWidth="0.5" filter="url(#bg)" />
                  <text x={b.x} y={b.y + 1.5} textAnchor="middle" fill={b.col} fontSize="5" fontFamily="monospace" fontWeight="bold">{b.mw}</text>
                  <text x={b.x} y={b.y - 6.5} textAnchor="middle" fill="rgba(255,255,255,0.3)" fontSize="4.5" fontFamily="monospace">{b.name}</text>
                </g>
              ))}
              {PLANTS.map(p => {
                const isTr = highlightPlant === p.id;
                const r = Math.sqrt(p.mw) / 4;
                return (<g key={p.id} onClick={() => handlePlantClick(p.id)} style={{ cursor: "pointer" }}>
                  <circle cx={p.x} cy={p.y} r={r + 6} fill="transparent" />
                  <circle cx={p.x} cy={p.y} r={r + 2} fill="none" stroke={isTr ? "#ff1744" : p.col} strokeWidth="0.6" opacity={isTr ? .8 : .4} style={isTr ? { animation: "tripFlash .8s ease-in-out" } : {}} />
                  <circle cx={p.x} cy={p.y} r={r} filter="url(#pg)" fill={isTr ? "#ff174430" : p.col} stroke={isTr ? "#ff1744" : "none"} strokeWidth={isTr ? .8 : 0} opacity={isTr ? .5 : .85} />
                  {isTr && <><line x1={p.x - r * .6} y1={p.y - r * .6} x2={p.x + r * .6} y2={p.y + r * .6} stroke="#ff1744" strokeWidth="1.2" />
                    <line x1={p.x + r * .6} y1={p.y - r * .6} x2={p.x - r * .6} y2={p.y + r * .6} stroke="#ff1744" strokeWidth="1.2" /></>}
                  <text x={p.x} y={p.y - r - 4} textAnchor="middle" fill={isTr ? "#ff1744" : "rgba(255,255,255,0.5)"} fontSize="6" fontFamily="monospace" fontWeight="bold">{p.name}</text>
                  <text x={p.x} y={p.y + 2} textAnchor="middle" fill={isTr ? "#ff174488" : "#fff"} fontSize="5.5" fontFamily="monospace" fontWeight="bold">{p.mw} MW</text>
                </g>);
              })}
              <g transform="translate(8,6)">
                {[["#e8854a", "Thermal", 0], ["#4facfe", "BESS", 10], ["#43e97b", "Solar+BESS", 20], ["rgba(130,177,255,0.5)", "Load (MW)", 30]].map(([c, l, y]) => (
                  <g key={l}><circle cx={4} cy={y + 3} r={2.5} fill={c} opacity={.6} /><text x={10} y={y + 5} fill="rgba(255,255,255,0.3)" fontSize="5" fontFamily="monospace">{l}</text></g>))}
                <line x1={1} y1={45} x2={8} y2={45} stroke="rgba(255,255,255,0.14)" strokeWidth="1" /><text x={10} y={47} fill="rgba(255,255,255,0.3)" fontSize="5" fontFamily="monospace">138 kV</text>
              </g>
            </svg>
            {!isRunning && <div style={{ textAlign: "center", marginTop: 8, fontSize: 10, color: "rgba(255,255,255,0.2)", fontStyle: "italic" }}>Click a plant to trip it, or pick a scenario on the right</div>}
          </div>
        </div>

        {/* RIGHT COLUMN */}
        <div style={{ display: "flex", flexDirection: "column", gap: 12 }}>
          <div style={BX}>
            <div style={LB}>REGULATORY ERA</div>
            {ERAS.map((er, i) => (
              <div key={i} onClick={() => handleEra(i)}
                style={{ padding: "5px 9px", marginBottom: 2, borderRadius: 5, cursor: "pointer", fontSize: 9,
                  background: era === i ? "rgba(0,230,118,0.07)" : "transparent",
                  border: era === i ? "1px solid rgba(0,230,118,0.2)" : "1px solid transparent",
                  color: era === i ? "#00e676" : "rgba(255,255,255,0.28)", transition: "all 0.2s" }}>
                {er.label}
              </div>))}
            <div style={{ fontSize: 8, color: "rgba(255,255,255,0.18)", marginTop: 4, padding: "2px 9px" }}>{e.desc}</div>
          </div>

          {/* CONTINGENCY PICKER */}
          <div style={BX}>
            <div style={LB}>CONTINGENCY SCENARIO</div>
            <div style={{ display: "flex", gap: 3, marginBottom: 10, flexWrap: "wrap" }}>
              {CATS.map(cat => (
                <div key={cat.id} onClick={() => handleSelectCat(cat.id)}
                  style={{ padding: "4px 8px", borderRadius: 4, cursor: "pointer", fontSize: 8, transition: "all 0.2s", whiteSpace: "nowrap",
                    background: selectedCat === cat.id ? cat.color + "18" : "transparent",
                    border: selectedCat === cat.id ? "1px solid " + cat.color + "40" : "1px solid rgba(255,255,255,0.06)",
                    color: selectedCat === cat.id ? cat.color : "rgba(255,255,255,0.3)" }}>
                  {cat.icon} {cat.label}
                </div>
              ))}
            </div>
            <div style={{ display: "flex", flexDirection: "column", gap: 4 }}>
              {catScenarios.map(sc => {
                const sel = selectedScenario === sc.id;
                return (
                  <div key={sc.id} onClick={() => handleSelectScenario(sc.id)}
                    style={{ padding: "7px 10px", borderRadius: 6, cursor: "pointer", transition: "all 0.15s",
                      background: sel ? sc.color + "12" : "rgba(255,255,255,0.015)",
                      border: sel ? "1px solid " + sc.color + "40" : "1px solid rgba(255,255,255,0.04)" }}>
                    <div style={{ fontSize: 10, fontWeight: 600, color: sel ? sc.color : "rgba(255,255,255,0.5)" }}>
                      {sc.icon} {sc.label}
                    </div>
                    <div style={{ fontSize: 8, color: "rgba(255,255,255,0.22)", marginTop: 2 }}>{sc.sub}</div>
                  </div>
                );
              })}
            </div>
            {selectedCat === "bess" && era < 3 && (
              <div style={{ fontSize: 8, color: "#ffc400", marginTop: 6, padding: "4px 6px", background: "rgba(255,196,0,0.06)", borderRadius: 4 }}>
                Current era has no GFM BESS — select Era 4 or 5
              </div>
            )}
            <button onClick={handleRun}
              style={{ width: "100%", marginTop: 10, padding: "9px 0", borderRadius: 6, border: "1px solid " + (scenario?.color || "#00e676") + "60",
                background: (scenario?.color || "#00e676") + "15", color: scenario?.color || "#00e676", fontFamily: "inherit", fontSize: 10,
                fontWeight: 700, letterSpacing: 2, cursor: "pointer", transition: "all 0.2s" }}>
              SIMULATE
            </button>
          </div>

          {/* RESULTS */}
          {result ? (
            <div style={{ ...BX, animation: "fadeUp 0.3s ease" }}>
              <div style={LB}>SYSTEM FREQUENCY RESPONSE</div>
              <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr 1fr", gap: 5, marginBottom: 10 }}>
                {[
                  ["NADIR", result.nadir.toFixed(2) + " Hz", result.nadir < 59.3 ? "#ff1744" : result.nadir < 59.5 ? "#ffc400" : "#00e676"],
                  ["ROCOF", result.mR.toFixed(2) + " Hz/s", result.mR > 2 ? "#ff1744" : result.mR > 1 ? "#ffc400" : "#00e676"],
                  ["SS FREQ", result.ss.toFixed(2) + " Hz", Math.abs(result.ss - 60) < 0.05 ? "#00e676" : "#82b1ff"],
                ].map(([l, v, c]) => (
                  <div key={l} style={{ textAlign: "center", padding: "6px 2px", background: "rgba(0,0,0,0.2)", borderRadius: 5 }}>
                    <div style={{ fontSize: 7, color: "rgba(255,255,255,0.25)", letterSpacing: 1 }}>{l}</div>
                    <div style={{ fontSize: 13, fontWeight: 700, color: c, marginTop: 2 }}>{v}</div>
                  </div>))}
              </div>
              <FreqChart r={result} />
              <div style={{ marginTop: 8 }}><PowerChart r={result} /></div>
              <div style={{ marginTop: 6 }}><SOCChart r={result} /></div>
              <div style={{
                marginTop: 10, padding: 8, borderRadius: 5, fontSize: 9, lineHeight: 1.5,
                background: result.nadir < 59.3 ? "rgba(255,23,68,0.07)" : result.nadir < 59.5 ? "rgba(255,196,0,0.05)" : "rgba(0,230,118,0.04)",
                border: "1px solid " + (result.nadir < 59.3 ? "rgba(255,23,68,0.18)" : result.nadir < 59.5 ? "rgba(255,196,0,0.12)" : "rgba(0,230,118,0.1)")
              }}>
                <span style={{ fontWeight: 700, color: result.nadir < 59.3 ? "#ff1744" : result.nadir < 59.5 ? "#ffc400" : "#00e676" }}>
                  {result.nadir < 59.3 ? "UFLS ACTIVATED" : result.nadir < 59.5 ? "MARGINAL" : "STABLE"}
                </span>
                <span style={{ color: "rgba(255,255,255,0.35)" }}>
                  {" \u2014 "}{result.nadir < 59.3 ? "Nadir breaches 59.3 Hz. Load shedding initiated." : result.nadir < 59.5 ? "Nadir approaches UFLS threshold. Limited margin." : "Frequency contained above 59.5 Hz. Adequate reserves."}
                  {scenario && " " + scenario.icon + " " + scenario.label + " \u2014 " + scenario.sub}
                </span>
              </div>
            </div>
          ) : (
            <div style={{ ...BX, display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center", minHeight: 200, gap: 10 }}>
              <div style={{ width: 44, height: 44, borderRadius: "50%", border: "2px solid rgba(255,255,255,0.06)", display: "flex", alignItems: "center", justifyContent: "center" }}>
                <span style={{ fontSize: 22 }}>{"\u26A1"}</span></div>
              <div style={{ fontSize: 11, color: "rgba(255,255,255,0.2)", textAlign: "center" }}>Pick a scenario and hit SIMULATE</div>
            </div>
          )}

          {/* FLEET */}
          <div style={BX}>
            <div style={LB}>FLEET COMPOSITION</div>
            {(() => {
              const nm = { leg: "Legacy (pre-2012)", rt: "Ride-through", sm: "Smart (Rule 14H)", gfm: "Grid-forming BESS" };
              const cl = { leg: "#d62728", rt: "#ff7f0e", sm: "#2ca02c", gfm: "#1f77b4" };
              const built = e.build();
              const rows = [["Synchronous", e.syncMW, "#8c564b"], ...built.map(f => [nm[f.t], f.c, cl[f.t]]), ["System load", e.pLoad, "#82b1ff"]];
              return rows.map(([n, v, c], i) => (
                <div key={i} style={{ display: "flex", justifyContent: "space-between", padding: "2px 0", borderBottom: "1px solid rgba(255,255,255,0.03)", fontSize: 9, color: "rgba(255,255,255,0.35)" }}>
                  <span>{n}</span><span style={{ color: c, fontWeight: 600 }}>{v} MW</span></div>));
            })()}
          </div>

          {/* PARAMS */}
          <div style={BX}>
            <div style={LB}>SYSTEM PARAMETERS</div>
            <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 2, fontSize: 9, color: "rgba(255,255,255,0.35)" }}>
              {[["S_BASE", "1800 MVA"], ["H_SYNC", "4.0 s"], ["R_SYNC", "5%"], ["T_GOV", "0.3 s"],
                ["T_REHEAT", "7.0 s"], ["F_HP", "0.30"], ["D_LOAD", "1.5"], ["UFLS", "59.3/58.9/58.5"],
                ["AGC_KP", "50"], ["AGC_KI", "20"]].map(([k, v]) => (
                <div key={k} style={{ display: "flex", justifyContent: "space-between", padding: "2px 0", borderBottom: "1px solid rgba(255,255,255,0.03)" }}>
                  <span>{k}</span><span style={{ color: "#fff", fontWeight: 600 }}>{v}</span></div>))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
