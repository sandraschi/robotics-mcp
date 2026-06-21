import { Layers, Map, Maximize, Radio, Target } from "lucide-react";
import "./map.css";

export default function MapPage() {
  return (
    <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen">
      <div className="flex justify-between items-center mb-6">
        <div>
          <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
            <Map className="w-8 h-8 text-emerald-400" />
            Map & Telemetry
          </h1>
          <p className="text-gray-400">
            Live SLAM rendering, Nav2 costmaps, and hardware pointcloud
            injection.
          </p>
        </div>

        <div className="flex gap-4 items-center">
          <div className="bg-slate-900 border border-slate-800 rounded-lg p-1 flex">
            <button className="px-4 py-1.5 text-sm bg-slate-800 text-white rounded-md font-medium">
              2D Grid
            </button>
            <button className="px-4 py-1.5 text-sm text-slate-400 hover:text-white transition-colors">
              3D Pointcloud
            </button>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Tools Sidebar */}
        <div className="flex flex-col gap-6">
          <div className="glass-card p-4 border border-white/10 rounded-xl space-y-2">
            <h3 className="text-xs font-semibold text-slate-500 uppercase tracking-wider mb-4">
              View Layers
            </h3>
            {[
              { name: "Occupancy Grid (Map)", active: true, icon: Map },
              { name: "Global Costmap", active: true, icon: Layers },
              { name: "Local Costmap", active: false, icon: Layers },
              { name: "LiDAR Scans (/scan)", active: true, icon: Radio },
              { name: "Robot Odometry (/odom)", active: true, icon: Target },
            ].map((layer, i) => (
              <button
                key={i}
                className={`flex items-center gap-3 w-full p-2 rounded-lg transition-colors ${layer.active ? "bg-emerald-500/10 text-emerald-400 border border-emerald-500/20" : "text-slate-400 hover:bg-white/5 border border-transparent"}`}
                title={`Toggle ${layer.name}`}
              >
                <layer.icon className="w-4 h-4" />
                <span className="text-sm font-medium flex-1 text-left">
                  {layer.name}
                </span>
                <div
                  className={`w-3 h-3 rounded-full border ${layer.active ? "bg-emerald-500 border-emerald-500" : "border-slate-600"}`}
                />
              </button>
            ))}
          </div>

          <div className="glass-card p-4 border border-white/10 rounded-xl">
            <h3 className="text-xs font-semibold text-slate-500 uppercase tracking-wider mb-4">
              SLAM Statistics
            </h3>
            <div className="space-y-3 font-mono text-sm">
              <div className="flex justify-between items-center text-slate-300">
                <span>Resolution</span>
                <span className="text-emerald-400">0.05m/px</span>
              </div>
              <div className="flex justify-between items-center text-slate-300">
                <span>Dimensions</span>
                <span className="text-emerald-400">1024x1024</span>
              </div>
              <div className="flex justify-between items-center text-slate-300">
                <span>Origin</span>
                <span className="text-emerald-400">[-10.0, -10.0, 0.0]</span>
              </div>
              <div className="pt-3 border-t border-white/10">
                <div className="flex justify-between items-center text-slate-300">
                  <span>Loop Closures</span>
                  <span className="text-blue-400">14</span>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Map Interface */}
        <div className="lg:col-span-3 glass-card border border-white/10 rounded-xl overflow-hidden relative flex flex-col min-h-[600px]">
          {/* Toolbar overlay */}
          <div className="absolute top-4 right-4 z-10 flex flex-col gap-2">
            <button
              title="Maximize View"
              aria-label="Maximize Map View"
              className="p-2 bg-slate-900 border border-slate-700 rounded shadow-lg text-slate-300 hover:text-white transition-colors"
            >
              <Maximize className="w-5 h-5" />
            </button>
            <button
              title="Center on Robot"
              aria-label="Center Map on Robot Position"
              className="p-2 bg-slate-900 border border-slate-700 rounded shadow-lg text-slate-300 hover:text-white transition-colors"
            >
              <Target className="w-5 h-5" />
            </button>
          </div>

          <div className="flex-1 bg-[#0a0f18] relative overflow-hidden group border-b border-white/10">
            {/* Fake Occupancy Grid Pattern + Sweep */}
            <div className="absolute inset-0 map-grid-pattern" />

            {/* Random Map walls pseudo-SVG */}
            <svg className="absolute inset-0 w-full h-full opacity-60">
              <path
                d="M 200 100 L 200 400 L 600 400 L 600 250 L 800 250 L 800 100 Z"
                fill="none"
                stroke="currentColor"
                strokeWidth="4"
                className="text-slate-600"
              />
              <path
                d="M 200 100 L 200 400 L 600 400 L 600 250 L 800 250 L 800 100 Z"
                fill="rgba(148,163,184,0.1)"
                stroke="none"
              />
              {/* Inner obstacle */}
              <rect
                x="300"
                y="200"
                width="100"
                height="80"
                className="fill-slate-800 stroke-slate-600"
                strokeWidth="2"
              />
              {/* Nav2 Costmap Gradient Effect (fake) */}
              <path
                d="M 180 80 L 180 420 L 620 420 L 620 270 L 820 270 L 820 80 Z"
                fill="none"
                stroke="rgba(239,68,68,0.2)"
                strokeWidth="16"
              />
            </svg>

            {/* LiDAR sweep animation */}
            <div
              aria-hidden="true"
              className="absolute top-[280px] left-[450px] w-[500px] h-[500px] rounded-full border border-emerald-500/10 lidar-sweep -translate-x-1/2 -translate-y-1/2 animate-[spin_3s_linear_infinite]"
            />

            {/* Pointcloud dots (fake) */}
            <div aria-hidden="true">
              <div className="absolute top-[160px] left-[380px] w-1 h-1 bg-red-500 rounded-full pointcloud-dot" />
              <div className="absolute top-[180px] left-[360px] w-1 h-1 bg-red-500 rounded-full pointcloud-dot" />
              <div className="absolute top-[200px] left-[340px] w-1 h-1 bg-red-500 rounded-full pointcloud-dot" />
            </div>

            {/* Robot Center Indicator */}
            <div className="absolute top-[280px] left-[450px] w-6 h-6 bg-emerald-500 rounded-full border-2 border-white flex items-center justify-center -translate-x-1/2 -translate-y-1/2 robot-indicator-glow z-20">
              <div className="w-1 h-3 bg-white rounded-full -mt-2 shadow-sm" />
            </div>

            {/* Trajectory path */}
            <svg className="absolute inset-0 w-full h-full pointer-events-none">
              <path
                d="M 450 280 C 450 350, 300 350, 250 300"
                fill="none"
                stroke="rgba(56,189,248,0.5)"
                strokeWidth="3"
                strokeDasharray="6 6"
              />
            </svg>
            <div className="absolute top-[300px] left-[250px] w-4 h-4 rounded-full border-2 border-sky-400 -translate-x-1/2 -translate-y-1/2 bg-sky-400/20" />
          </div>
        </div>
      </div>
    </div>
  );
}
