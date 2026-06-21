import { Activity, Battery, Cpu, HardDrive, Server, Wifi } from "lucide-react";

export default function StatusPage() {
  return (
    <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen">
      <div className="mb-8">
        <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
          <Activity className="w-8 h-8 text-blue-400" />
          Fleet Status
        </h1>
        <p className="text-gray-400">
          Comprehensive live telemetry across all hardware and virtual nodes.
        </p>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
        {[
          {
            label: "Overall System Health",
            value: "99.9%",
            icon: Activity,
            color: "text-emerald-400",
            bg: "from-emerald-500/10",
          },
          {
            label: "Active Nodes",
            value: "12",
            icon: Server,
            color: "text-blue-400",
            bg: "from-blue-500/10",
          },
          {
            label: "Network Latency",
            value: "14ms",
            icon: Wifi,
            color: "text-purple-400",
            bg: "from-purple-500/10",
          },
          {
            label: "Fleet Power",
            value: "85%",
            icon: Battery,
            color: "text-amber-400",
            bg: "from-amber-500/10",
          },
        ].map((stat, i) => (
          <div
            key={i}
            className="glass-card p-6 border border-white/10 rounded-xl relative overflow-hidden group"
          >
            <div
              className={`absolute inset-0 bg-gradient-to-br ${stat.bg} to-transparent opacity-0 group-hover:opacity-100 transition-opacity`}
            />
            <div className="flex justify-between items-start mb-4">
              <stat.icon className={`w-6 h-6 ${stat.color}`} />
            </div>
            <h3 className="text-3xl font-bold mb-1">{stat.value}</h3>
            <p className="text-gray-400 text-sm">{stat.label}</p>
          </div>
        ))}
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* CPU/Memory Metrics */}
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <h3 className="text-lg font-semibold mb-6 flex items-center gap-2">
            <Cpu className="w-5 h-5 text-indigo-400" /> Compute Resources
          </h3>
          <div className="space-y-6">
            <div>
              <div className="flex justify-between text-sm mb-2">
                <span className="text-gray-400">
                  Main SBC CPU (Raspberry Pi 5)
                </span>
                <span className="font-mono">42%</span>
              </div>
              <div className="h-2 bg-slate-800 rounded-full overflow-hidden">
                <div className="h-full bg-indigo-500 w-[42%] rounded-full relative">
                  <div className="absolute inset-0 bg-white/20 animate-pulse" />
                </div>
              </div>
            </div>
            <div>
              <div className="flex justify-between text-sm mb-2">
                <span className="text-gray-400">Memory (16GB RAM)</span>
                <span className="font-mono">8.4 GB</span>
              </div>
              <div className="h-2 bg-slate-800 rounded-full overflow-hidden">
                <div className="h-full bg-blue-500 w-[52%] rounded-full relative" />
              </div>
            </div>
            <div>
              <div className="flex justify-between text-sm mb-2">
                <span className="text-gray-400">
                  NPU Accelerator (Ollama/Tensor)
                </span>
                <span className="font-mono">89%</span>
              </div>
              <div className="h-2 bg-slate-800 rounded-full overflow-hidden">
                <div className="h-full bg-amber-500 w-[89%] rounded-full relative" />
              </div>
            </div>
          </div>
        </div>

        {/* Robot Sub-Systems */}
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <h3 className="text-lg font-semibold mb-6 flex items-center gap-2">
            <HardDrive className="w-5 h-5 text-emerald-400" /> Subsystems Matrix
          </h3>
          <div className="space-y-4">
            {[
              { name: "ROS 2 Navigation Stack", status: "Active", ping: "2ms" },
              {
                name: "LiDAR Pointcloud Stream",
                status: "Active",
                ping: "5ms",
              },
              {
                name: "Dreame Miot API Bridge",
                status: "Polling",
                ping: "45ms",
              },
              {
                name: "Local LLM FastMCP Inference",
                status: "Active",
                ping: "120ms",
              },
              { name: "VRChat OSC Bridge", status: "Offline", ping: "-" },
            ].map((sys, i) => (
              <div
                key={i}
                className="flex items-center justify-between p-3 rounded-lg bg-white/5 border border-white/5 hover:border-white/10 transition-colors"
              >
                <span className="text-sm font-medium text-slate-200">
                  {sys.name}
                </span>
                <div className="flex items-center gap-4">
                  <span className="text-xs font-mono text-slate-400">
                    {sys.ping}
                  </span>
                  <span
                    className={`text-xs px-2 py-1 rounded-full ${
                      sys.status === "Active"
                        ? "bg-emerald-500/20 text-emerald-400 border border-emerald-500/30"
                        : sys.status === "Polling"
                          ? "bg-blue-500/20 text-blue-400 border border-blue-500/30"
                          : "bg-slate-800 text-slate-400 border border-slate-700"
                    }`}
                  >
                    {sys.status}
                  </span>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}
