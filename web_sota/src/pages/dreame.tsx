import {
  AlertCircle,
  Battery,
  Droplets,
  Home,
  Map as MapIcon,
  Play,
  Square,
  Wind,
} from "lucide-react";

export default function DreamePage() {
  return (
    <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
            <Wind className="w-8 h-8 text-sky-400" />
            Dreame D20 Pro
          </h1>
          <p className="text-gray-400">
            Autonomous floor cleaning, mop array configuration, and MIoT
            mappings.
          </p>
        </div>
        <div className="flex items-center gap-3 bg-blue-500/10 text-blue-400 border border-blue-500/20 px-4 py-2 rounded-full">
          <div className="w-2 h-2 rounded-full bg-blue-500 animate-pulse" />
          <span className="text-sm font-semibold">
            Status: Charging / Docked
          </span>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Status Column */}
        <div className="flex flex-col gap-6">
          <div className="glass-card p-6 border border-white/10 rounded-xl relative overflow-hidden group">
            <h3 className="text-lg font-semibold mb-6 flex items-center gap-2">
              <Battery className="w-5 h-5 text-emerald-400" /> Battery Level
            </h3>
            <div className="relative w-32 h-32 mx-auto">
              <svg className="w-full h-full transform -rotate-90">
                <circle
                  cx="64"
                  cy="64"
                  r="56"
                  className="stroke-slate-800"
                  strokeWidth="12"
                  fill="none"
                />
                <circle
                  cx="64"
                  cy="64"
                  r="56"
                  className="stroke-emerald-500"
                  strokeWidth="12"
                  fill="none"
                  strokeDasharray="351.85"
                  strokeDashoffset="35.185"
                />
              </svg>
              <div className="absolute inset-0 flex items-center justify-center">
                <span className="text-3xl font-bold">90%</span>
              </div>
            </div>
          </div>

          <div className="glass-card p-6 border border-white/10 rounded-xl">
            <h3 className="text-lg font-semibold mb-4">Consumables</h3>
            <div className="space-y-4">
              <div>
                <div className="flex justify-between text-sm mb-1 text-slate-300">
                  <span>Main Brush</span>
                  <span>72%</span>
                </div>
                <div className="h-1.5 bg-slate-800 rounded-full">
                  <div className="h-full bg-sky-400 w-[72%] rounded-full" />
                </div>
              </div>
              <div>
                <div className="flex justify-between text-sm mb-1 text-slate-300">
                  <span>Side Brush</span>
                  <span>45%</span>
                </div>
                <div className="h-1.5 bg-slate-800 rounded-full">
                  <div className="h-full bg-amber-400 w-[45%] rounded-full" />
                </div>
              </div>
              <div>
                <div className="flex justify-between text-sm mb-1 text-slate-300">
                  <span>Filter</span>
                  <span>
                    9%{" "}
                    <AlertCircle className="inline w-3 h-3 text-red-400 ml-1" />
                  </span>
                </div>
                <div className="h-1.5 bg-slate-800 rounded-full">
                  <div className="h-full bg-red-400 w-[9%] rounded-full" />
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Map Interface */}
        <div className="lg:col-span-2 glass-card border border-white/10 rounded-xl overflow-hidden flex flex-col">
          <div className="p-4 border-b border-white/10 bg-slate-900/50 flex justify-between">
            <h3 className="font-semibold flex items-center gap-2">
              <MapIcon className="w-4 h-4 text-sky-400" /> Floorplan Vector Map
            </h3>
            <span className="text-xs text-slate-400 font-mono">
              MAP ID: GROUND_FLOOR_B
            </span>
          </div>
          <div className="flex-1 relative bg-[#0f172a] min-h-[400px]">
            {/* Placeholder Map SVG / Grid */}
            <div
              className="absolute inset-0"
              style={{
                backgroundImage:
                  "radial-gradient(circle at 2px 2px, rgba(255,255,255,0.05) 1px, transparent 0)",
                backgroundSize: "32px 32px",
              }}
            />
            <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-48 h-48 border border-sky-500/30 bg-sky-500/5 rounded p-4">
              <p className="text-sky-500/50 text-xs text-center mt-16 font-mono">
                Kitchen (Zone 2)
              </p>
            </div>
            <div className="absolute top-1/2 left-1/4 -translate-y-1/2 w-32 h-64 border border-purple-500/30 bg-purple-500/5 rounded p-4">
              <p className="text-purple-500/50 text-xs text-center mt-24 font-mono">
                Hallway
              </p>
            </div>

            {/* Robot Icon */}
            <div className="absolute top-[45%] left-[60%] w-6 h-6 bg-sky-400 rounded-full shadow-[0_0_15px_rgba(56,189,248,0.5)] border-2 border-white flex items-center justify-center">
              <div className="w-1 h-3 bg-white rounded-full -mt-2 opacity-50" />
            </div>
          </div>
        </div>

        {/* Commands */}
        <div className="flex flex-col gap-6">
          <div className="glass-card p-6 border border-white/10 rounded-xl">
            <h3 className="text-lg font-semibold mb-4">Command Deck</h3>
            <div className="grid grid-cols-2 gap-3">
              <button className="flex flex-col items-center gap-2 bg-emerald-500/10 hover:bg-emerald-500/20 text-emerald-400 border border-emerald-500/20 p-4 rounded-xl transition-colors">
                <Play className="w-6 h-6" />
                <span className="text-sm font-medium">Start</span>
              </button>
              <button className="flex flex-col items-center gap-2 bg-rose-500/10 hover:bg-rose-500/20 text-rose-400 border border-rose-500/20 p-4 rounded-xl transition-colors">
                <Square className="w-6 h-6" />
                <span className="text-sm font-medium">Stop</span>
              </button>
              <button className="col-span-2 flex flex-col items-center gap-2 bg-sky-500/10 hover:bg-sky-500/20 text-sky-400 border border-sky-500/20 p-4 rounded-xl transition-colors">
                <Home className="w-6 h-6" />
                <span className="text-sm font-medium">Return to Base</span>
              </button>
            </div>
          </div>

          <div className="glass-card p-6 border border-white/10 rounded-xl flex-1">
            <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
              <Droplets className="w-5 h-5 text-blue-400" /> Mopping Parameters
            </h3>
            <div className="space-y-4">
              <div>
                <label className="text-xs text-slate-400 block mb-2">
                  Water Volume
                </label>
                <div className="flex gap-2">
                  {["Low", "Medium", "High"].map((level) => (
                    <button
                      key={level}
                      className={`flex-1 py-1 text-xs rounded border ${level === "Medium" ? "bg-blue-500 text-white border-blue-500" : "bg-transparent text-slate-300 border-white/10 hover:bg-white/5"}`}
                    >
                      {level}
                    </button>
                  ))}
                </div>
              </div>
              <div>
                <label className="text-xs text-slate-400 block mb-2">
                  Suction Power
                </label>
                <div className="flex gap-2">
                  {["Quiet", "Standard", "Turbo"].map((level) => (
                    <button
                      key={level}
                      className={`flex-1 py-1 text-xs rounded border ${level === "Turbo" ? "bg-orange-500 text-white border-orange-500" : "bg-transparent text-slate-300 border-white/10 hover:bg-white/5"}`}
                    >
                      {level}
                    </button>
                  ))}
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
