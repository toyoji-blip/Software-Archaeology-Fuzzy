/*
 This program reproduces a control system design originally implemented in 1987.

 This program simulates four process models (G1–G4) and compares the closed-loop
 responses of three PID tuning methods: 
 - Fuzzy-based tuning (1987 specification)
 - Takahashi–Chan Method (TCM)
 - IAE-based numerical optimization
 
 For each process model, the controller gains (Kp, Ki, Kd) obtained from the
 three methods are applied to an I-PD control loop. The resulting waveforms are
 evaluated in two scenarios:
 
  1. Setpoint Tracking (SV step response)
  2. Disturbance Rejection (load disturbance step)
 
 The purpose of this program is to reproduce, visualize, and compare the dynamic
 characteristics of the three tuning methods across all four process models.
*/


import React, { useState, useEffect, useCallback } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from 'recharts';

/**
 * PID制御 特性比較シミュレーター V0.5
 * 改良点: 内部計算dt(idt)と描画サンプリングdt(dt)を分離
 * ロジック: I-PD制御, 無駄時間(リングバッファ), 零点対応プロセスモデル
 */

// --- 定数・プリセットデータ ---
const PRESETS = {
  third_order: {
    Fuzzy: { kp: 3.394, ki: 0.377, kd: 7.358 },
    IAE:   { kp: 15.2508, ki: 1.3034, kd: 107.3539 },
    TCM:   { kp: 4.7381, ki: 0.7456, kd: 7.4449 },
    params: {
      K: 5,
      Ts: [2, 5, 8],
      Tz: 0,
      L: 0,
      dt: 1.0,   // 描画サンプリング周期（修論記載）
      idt: 0.2   // 内部計算dt（修論のステップ応答取得Δt）
    }
  },
  fourth_order: {
    Fuzzy: { kp: 2.885, ki: 0.234, kd: 9.192 },
    IAE:   { kp: 2.6988, ki: 0.3222, kd: 6.6462 },
    TCM:   { kp: 4.0775, ki: 0.3824, kd: 9.3105 },
    params: {
      K: 3,
      Ts: [1, 1, 3, 5],
      Tz: 0,
      L: 0,
      dt: 0.5,
      idt: 0.2
    }
  },
  second_order_zero: {
    Fuzzy: { kp: 2.881, ki: 0.331, kd: 6.905 },
    IAE:   { kp: 2.1278, ki: 0.3196, kd: 3.3198 },
    TCM:   { kp: 3.6937, ki: 0.4687, kd: 3 },
    params: {
      K: 3,
      Ts: [2, 5],
      Tz: 1,
      L: 2,
      dt: 0.5,
      idt: 0.25
    }
  },
  third_order_zero: {
    Fuzzy: { kp: 4.002, ki: 0.453, kd: 7.752 },
    IAE:   { kp: 4.9962, ki: 0.3701, kd: 15.8567 },
    TCM:   { kp: 5.5913, ki: 1.0276, kd: 7.9499 },
    params: {
      K: 5,
      Ts: [5, 9, 12],
      Tz: 11,
      L: 1,
      dt: 1.0,
      idt: 0.5
    }
  }
};

const SIM_DURATION = 50;

// --- シミュレーションコアロジック ---

function createProcess(p, idt) {
  const Ts = p.Ts || [];
  const Tz = p.Tz || 0;
  let x = Ts.map(() => 0);
  let yBasePrev = 0;

  return function(u) {
    let currentIn = u;
    for (let i = 0; i < Ts.length; i++) {
      const alpha = Math.exp(-idt / Ts[i]);
      x[i] = alpha * x[i] + (1 - alpha) * currentIn;
      currentIn = x[i];
    }
    const y_base = x[x.length - 1];
    let out;
    if (Tz !== 0) {
      const dy_base = (y_base - yBasePrev) / idt;
      out = y_base + Tz * dy_base;
    } else {
      out = y_base;
    }
    yBasePrev = y_base;
    return isFinite(out) ? out : 0;
  };
}

const runSimulation = (methodGains, p, mode, useInternalDt) => {
  // useInternalDt: true=内部dt分離モード, false=従来モード（dt統一）
  const outputDt = p.dt;
  const internalDt = useInternalDt ? p.idt : p.dt;
  const stepsPerOutput = Math.max(1, Math.round(outputDt / internalDt));

  const process = createProcess(p, internalDt);
  const { kp, ki, kd } = methodGains;

  const L = p.L || 0;
  const N = Math.round(L / internalDt);
  const useBuf = N > 0;
  let ubuf = useBuf ? Array(N).fill(0) : [];
  let bufIndex = 0;

  let y = 0, yPrev = 0, integralError = 0;
  let results = [];
  let outputCount = 0;
  const totalSteps = Math.round(SIM_DURATION / internalDt);

  for (let step = 0; step <= totalSteps; step++) {
    const t = parseFloat((step * internalDt).toFixed(6));
    const sv = (mode === 'tracking') ? 1.0 : 0;
    const dist = (mode === 'disturbance') ? 1.0 : 0;

    const error = sv - y;
    integralError += error * internalDt;
    const dy = (y - yPrev) / internalDt;

    // I-PD 制御
    const mv = (ki * integralError) - (kp * y) - (kd * dy);

    // 外乱はプロセスゲイン(外乱振幅K)を乗算
    const uIn = mv + (dist * p.K);

    let uDelayed;
    if (useBuf) {
      uDelayed = ubuf[bufIndex];
      ubuf[bufIndex] = uIn;
      bufIndex = (bufIndex + 1) % N;
    } else {
      uDelayed = uIn;
    }

    yPrev = y;
    y = process(uDelayed);

    if (Math.abs(y) > 10) break; // 発散ガード

    // 描画サンプリング: stepsPerOutputごとに記録
    if (step % stepsPerOutput === 0) {
      results.push({ t: parseFloat(t.toFixed(2)), y });
      outputCount++;
    }
  }
  return results;
};

// --- UI コンポーネント ---

const MethodTag = ({ type, children }) => {
  const styles = {
    fuzzy: "bg-emerald-100 text-emerald-700",
    iae:   "bg-red-100 text-red-700",
    tcm:   "bg-sky-100 text-sky-700"
  };
  return (
    <span className={`text-[10px] px-2 py-0.5 rounded font-bold uppercase ${styles[type]}`}>
      {children}
    </span>
  );
};

const InfoBadge = ({ label, value, highlight }) => (
  <div className={`flex flex-col items-center px-3 py-1.5 rounded-lg ${highlight ? 'bg-amber-50 border border-amber-200' : 'bg-slate-100'}`}>
    <span className="text-[9px] uppercase font-bold text-slate-400">{label}</span>
    <span className={`text-sm font-black font-mono ${highlight ? 'text-amber-700' : 'text-slate-700'}`}>{value}</span>
  </div>
);

export default function App() {
  const [selectedKey, setSelectedKey] = useState('third_order');
  const [useInternalDt, setUseInternalDt] = useState(false);
  const [trackingData, setTrackingData] = useState([]);
  const [disturbanceData, setDisturbanceData] = useState([]);

  const currentPreset = PRESETS[selectedKey];
  const p = currentPreset.params;

  const updateSim = useCallback(() => {
    const methods = ['Fuzzy', 'IAE', 'TCM'];
    const modes = ['tracking', 'disturbance'];
    const results = {};

    modes.forEach(mode => {
      const modeData = [];
      const simResults = methods.map(m =>
        runSimulation(currentPreset[m], p, mode, useInternalDt)
      );
      const length = simResults[0].length;
      for (let i = 0; i < length; i++) {
        modeData.push({
          t: simResults[0][i].t,
          fuzzy: simResults[0][i].y,
          iae:   simResults[1][i]?.y,
          tcm:   simResults[2][i]?.y,
          ref:   mode === 'tracking' ? 1.0 : 0
        });
      }
      results[mode] = modeData;
    });

    setTrackingData(results.tracking);
    setDisturbanceData(results.disturbance);
  }, [selectedKey, useInternalDt, currentPreset, p]);

  useEffect(() => {
    updateSim();
  }, [updateSim]);

  return (
    <div className="p-4 md:p-8 bg-slate-50 min-h-screen text-slate-900 font-sans">
      <div className="max-w-6xl mx-auto">

        {/* Header */}
        <header className="flex flex-col md:flex-row md:items-end justify-between mb-8 gap-4 border-b border-slate-200 pb-6">
          <div>
            <h1 className="text-3xl font-black tracking-tight text-slate-900">PID制御 特性比較</h1>
            <p className="text-slate-500 font-medium text-sm">I-PD制御 追値・外乱応答解析</p>
          </div>
          <div className="flex flex-wrap items-center gap-3">
            {/* Process selector */}
            <div className="bg-white p-3 rounded-xl shadow-sm border border-slate-200">
              <div className="flex flex-col">
                <span className="text-[10px] uppercase font-bold text-slate-400 ml-1">Process Model</span>
                <select
                  value={selectedKey}
                  onChange={(e) => setSelectedKey(e.target.value)}
                  className="bg-transparent border-none text-sm font-bold text-slate-700 focus:ring-0 cursor-pointer outline-none"
                >
                  <option value="third_order">G1: 三次遅れ (T=2,5,8)</option>
                  <option value="fourth_order">G2: 四次遅れ (T=1,1,3,5)</option>
                  <option value="second_order_zero">G3: 二次遅れ零点 (T=2,5, Tz=1, L=2)</option>
                  <option value="third_order_zero">G4: 三次遅れ零点 (T=5,9,12, Tz=11, L=1)</option>
                </select>
              </div>
            </div>

            {/* dt mode toggle */}
            <div className="bg-white p-3 rounded-xl shadow-sm border border-slate-200 flex flex-col gap-1">
              <span className="text-[10px] uppercase font-bold text-slate-400">Simulation Mode</span>
              <div className="flex gap-2">
                <button
                  onClick={() => setUseInternalDt(false)}
                  className={`px-3 py-1.5 rounded-lg text-xs font-bold transition-all ${
                    !useInternalDt
                      ? 'bg-indigo-600 text-white shadow-md'
                      : 'bg-slate-100 text-slate-500 hover:bg-slate-200'
                  }`}
                >
                  従来モード (dt統一)
                </button>
                <button
                  onClick={() => setUseInternalDt(true)}
                  className={`px-3 py-1.5 rounded-lg text-xs font-bold transition-all ${
                    useInternalDt
                      ? 'bg-amber-500 text-white shadow-md'
                      : 'bg-slate-100 text-slate-500 hover:bg-slate-200'
                  }`}
                >
                  内部dt分離モード
                </button>
              </div>
            </div>

            <button
              onClick={updateSim}
              className="bg-indigo-600 text-white px-6 py-2 rounded-lg text-sm font-bold hover:bg-indigo-700 transition-all shadow-md active:scale-95"
            >
              再計算
            </button>
          </div>
        </header>

        <div className="grid grid-cols-1 lg:grid-cols-4 gap-8">

          {/* Left panel */}
          <div className="lg:col-span-1 space-y-4">

            {/* dt info */}
            <div className={`p-4 rounded-2xl shadow-sm border-2 ${useInternalDt ? 'bg-amber-50 border-amber-200' : 'bg-white border-slate-200'}`}>
              <h3 className="text-xs font-black text-slate-400 mb-3 uppercase tracking-widest">
                {useInternalDt ? '🔬 内部dt分離モード' : '📊 従来モード'}
              </h3>
              <div className="flex gap-2 flex-wrap">
                <InfoBadge label="描画Δt" value={`${p.dt}s`} />
                <InfoBadge label="内部Δt" value={`${p.idt}s`} highlight={useInternalDt} />
                <InfoBadge label="外乱振幅" value={`${p.K}`} />
                {p.L > 0 && <InfoBadge label="むだ時間" value={`${p.L}s`} />}
              </div>
              {useInternalDt && (
                <p className="text-[10px] text-amber-700 mt-2 leading-relaxed">
                  内部計算をidt={p.idt}sで実行し、dt={p.dt}sごとに描画データを出力します。
                  修論のステップ応答取得Δtを内部dtとして使用。
                </p>
              )}
              {!useInternalDt && (
                <p className="text-[10px] text-slate-500 mt-2 leading-relaxed">
                  内部計算・描画ともにdt={p.dt}sを使用（V0.4と同等）。
                </p>
              )}
            </div>

            {/* Gains */}
            <div className="bg-white p-5 rounded-2xl shadow-sm border border-slate-200">
              <h3 className="text-xs font-black text-slate-400 mb-4 uppercase tracking-widest">Gain Parameters</h3>
              <div className="space-y-4">
                {['Fuzzy', 'IAE', 'TCM'].map(m => (
                  <div key={m} className="space-y-1.5">
                    <MethodTag type={m.toLowerCase()}>{m === 'Fuzzy' ? 'Present (Fuzzy)' : m}</MethodTag>
                    <div className={`grid grid-cols-3 gap-1 text-[10px] font-mono p-2 rounded ${
                      m === 'Fuzzy' ? 'bg-emerald-50 text-emerald-800' :
                      m === 'TCM'   ? 'bg-sky-50 text-sky-800' : 'bg-slate-50'
                    }`}>
                      <div>Kp:<br/><span className="font-bold">{currentPreset[m].kp.toFixed(3)}</span></div>
                      <div>Ki:<br/><span className="font-bold">{currentPreset[m].ki.toFixed(3)}</span></div>
                      <div>Kd:<br/><span className="font-bold">{currentPreset[m].kd.toFixed(3)}</span></div>
                    </div>
                  </div>
                ))}
              </div>
            </div>

            {/* Logic note */}
            <div className="bg-white p-4 rounded-2xl shadow-sm border border-slate-200">
              <h3 className="text-xs font-black text-slate-400 mb-2 uppercase tracking-widest">Logic</h3>
              <div className="text-[10px] text-slate-500 space-y-1.5 leading-relaxed">
                <p><b>I-PD制御:</b> MV = Ki∫e·dt − Kp·y − Kd·ẏ</p>
                <p><b>追値:</b> SV=1, 外乱なし</p>
                <p><b>外乱:</b> SV=0, d=K (t=0から)</p>
                <p><b>離散化:</b> α=exp(−idt/T)</p>
                {p.L > 0 && <p><b>むだ時間:</b> リングバッファ N={Math.round(p.L / (useInternalDt ? p.idt : p.dt))}ステップ</p>}
              </div>
            </div>
          </div>

          {/* Charts */}
          <div className="lg:col-span-3 space-y-6">

            {/* Tracking */}
            <div className="bg-white p-6 rounded-2xl shadow-sm border border-slate-200">
              <h2 className="text-sm font-bold text-slate-800 mb-4 flex items-center gap-2">
                <div className="w-2 h-6 bg-blue-500 rounded-full"></div>
                Setpoint Tracking Response (SV 0 → 1)
              </h2>
              <div className="h-[260px] w-full">
                <ResponsiveContainer>
                  <LineChart data={trackingData}>
                    <CartesianGrid strokeDasharray="3 3" vertical={false} stroke="#f1f5f9" />
                    <XAxis
                      dataKey="t"
                      fontSize={10}
                      axisLine={{ stroke: '#94a3b8' }}
                      tick={{ fill: '#64748b' }}
                      label={{ value: 'Time (s)', position: 'insideBottomRight', offset: -5, fontSize: 10 }}
                    />
                    <YAxis fontSize={10} domain={[0, 1.5]} />
                    <Tooltip formatter={(v) => v?.toFixed(3)} />
                    <Legend
                      verticalAlign="top"
                      align="right"
                      iconType="circle"
                      wrapperStyle={{ fontSize: '12px' }}
                      payload={[
                        { value: 'Present (Fuzzy)', type: 'line', color: '#22c55e' },
                        { value: 'IAE',             type: 'line', color: '#ef4444' },
                        { value: 'TCM',             type: 'line', color: '#0ea5e9' },
                      ]}
                    />
                    <Line name="Present (Fuzzy)" type="monotone" dataKey="fuzzy" stroke="#22c55e" strokeWidth={3} dot={false} isAnimationActive={false} />
                    <Line name="IAE"             type="monotone" dataKey="iae"   stroke="#ef4444" dot={false} isAnimationActive={false} />
                    <Line name="TCM"             type="monotone" dataKey="tcm"   stroke="#0ea5e9" dot={false} isAnimationActive={false} />
                    <Line name="Ref"             type="step"     dataKey="ref"   stroke="#94a3b8" strokeDasharray="5 5" dot={false} isAnimationActive={false} legendType="none" />
                  </LineChart>
                </ResponsiveContainer>
              </div>
            </div>

            {/* Disturbance */}
            <div className="bg-white p-6 rounded-2xl shadow-sm border-2 border-indigo-50">
              <h2 className="text-sm font-bold text-slate-800 mb-4 flex items-center gap-2">
                <div className="w-2 h-6 bg-amber-500 rounded-full"></div>
                Disturbance Response (d = +{p.K}, t = 0)
              </h2>
              <div className="h-[260px] w-full">
                <ResponsiveContainer>
                  <LineChart data={disturbanceData}>
                    <CartesianGrid strokeDasharray="3 3" vertical={false} stroke="#f1f5f9" />
                    <XAxis
                      dataKey="t"
                      fontSize={10}
                      axisLine={{ stroke: '#94a3b8' }}
                      tick={{ fill: '#64748b' }}
                      label={{ value: 'Time (s)', position: 'insideBottomRight', offset: -5, fontSize: 10 }}
                    />
                    <YAxis fontSize={10} />
                    <Tooltip formatter={(v) => v?.toFixed(3)} />
                    <Legend
                      verticalAlign="top"
                      align="right"
                      iconType="circle"
                      wrapperStyle={{ fontSize: '12px' }}
                      payload={[
                        { value: 'Present (Fuzzy)', type: 'line', color: '#22c55e' },
                        { value: 'IAE',             type: 'line', color: '#ef4444' },
                        { value: 'TCM',             type: 'line', color: '#0ea5e9' },
                      ]}
                    />
                    <Line name="Present (Fuzzy)" type="monotone" dataKey="fuzzy" stroke="#22c55e" strokeWidth={3} dot={false} isAnimationActive={false} />
                    <Line name="IAE"             type="monotone" dataKey="iae"   stroke="#ef4444" dot={false} isAnimationActive={false} />
                    <Line name="TCM"             type="monotone" dataKey="tcm"   stroke="#0ea5e9" dot={false} isAnimationActive={false} />
                    <Line
                      name="Zero"
                      type="step"
                      dataKey={() => 0}
                      stroke="#94a3b8"
                      strokeDasharray="5 5"
                      dot={false}
                      isAnimationActive={false}
                      legendType="none"
                    />
                  </LineChart>
                </ResponsiveContainer>
              </div>
            </div>

            {/* Mode comparison note */}
            <div className={`p-4 rounded-xl text-xs leading-relaxed ${
              useInternalDt ? 'bg-amber-50 border border-amber-200 text-amber-800' : 'bg-slate-100 text-slate-500'
            }`}>
              {useInternalDt ? (
                <>
                  <b>🔬 内部dt分離モード有効:</b> 制御計算・プロセス離散化をidt={p.idt}sで実行。
                  描画は{p.dt}sごとにサンプリング。
                  無駄時間バッファ N={Math.round(p.L / p.idt)}ステップ (={p.L}s)。
                  修論の1987年実装に近い挙動を推定再現。
                </>
              ) : (
                <>
                  <b>📊 従来モード:</b> 全計算をdt={p.dt}sで統一（V0.4と同等）。
                  内部dtと描画dtを分離するには「内部dt分離モード」を選択してください。
                </>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}