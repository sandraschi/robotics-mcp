import {
  ArrowRight,
  Box,
  Brain,
  Gamepad2,
  Glasses,
  Image,
  Link,
  Video,
} from "lucide-react";

export default function VbotEcosystem() {
  return (
    <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
            <Box className="w-8 h-8 text-cyan-400" />
            Virtual Robotics Ecosystem
          </h1>
          <p className="text-gray-400">
            Bridging local cognitive architectures with diverse virtual
            embodiments running via inter-MCP orchestration.
          </p>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 xl:grid-cols-4 gap-6 mb-8">
        {[
          {
            title: "Unity 3D / Gazebo",
            url: "http://localhost:10712",
            badge: "PORT 10712",
            desc: "Rigid-body ROS simulation engine & synthetic training datasets.",
            icon: Gamepad2,
            color: "text-emerald-400",
            border: "hover:border-emerald-500/50",
            bg: "hover:bg-emerald-500/10",
          },
          {
            title: "Resonite (OSC)",
            url: "http://localhost:10728",
            badge: "PORT 10728",
            desc: "Native ProtoFlux manipulation & spatial computing playground.",
            icon: Glasses,
            color: "text-yellow-400",
            border: "hover:border-yellow-500/50",
            bg: "hover:bg-yellow-500/10",
          },
          {
            title: "VRChat / Avatar",
            url: "http://localhost:10732",
            badge: "PORT 10732",
            desc: "Social embodiment, IK rigging hooks, and expressive locomotion.",
            icon: Video,
            color: "text-rose-400",
            border: "hover:border-rose-500/50",
            bg: "hover:bg-rose-500/10",
          },
          {
            title: "World Labs (API)",
            url: "http://localhost:10748",
            badge: "PORT 10748",
            desc: "Generative spatial layout intelligence and nav-mesh creation.",
            icon: Image,
            color: "text-indigo-400",
            border: "hover:border-indigo-500/50",
            bg: "hover:bg-indigo-500/10",
          },
        ].map((node, i) => (
          <a
            key={i}
            href={node.url}
            target="_blank"
            rel="noreferrer"
            className={`glass-card p-6 border border-white/10 rounded-xl flex flex-col group transition-all duration-300 ${node.border} ${node.bg}`}
          >
            <div className="flex justify-between items-start mb-4">
              <node.icon className={`w-8 h-8 ${node.color}`} />
              <span className="text-xs font-mono px-2 py-1 bg-slate-800 text-slate-400 rounded border border-slate-700">
                {node.badge}
              </span>
            </div>
            <h3 className="text-lg font-bold mb-2 group-hover:text-white transition-colors">
              {node.title}
            </h3>
            <p className="text-sm text-slate-400 mb-4 flex-1">{node.desc}</p>
            <div className="flex items-center text-sm font-medium text-slate-300 group-hover:text-white transition-colors">
              Launch Interface{" "}
              <ArrowRight className="ml-2 w-4 h-4 transition-transform group-hover:translate-x-1" />
            </div>
          </a>
        ))}
      </div>

      {/* Architecture Visualization Panel */}
      <div className="glass-card border border-white/10 rounded-xl p-8 relative overflow-hidden">
        <div className="absolute inset-0 bg-gradient-to-br from-cyan-900/20 to-transparent pointer-events-none" />

        <h3 className="text-xl font-bold mb-8 flex items-center gap-2 relative z-10">
          <Link className="w-6 h-6 text-cyan-400" />
          VBot Component Graph Architecture
        </h3>

        <div className="relative h-[400px] w-full bg-slate-900/50 rounded-lg border border-slate-800 flex items-center justify-center">
          {/* SVG Connections */}
          <svg className="absolute inset-0 w-full h-full pointer-events-none z-0">
            <line
              x1="25%"
              y1="50%"
              x2="50%"
              y2="50%"
              stroke="rgba(34,211,238,0.3)"
              strokeWidth="2"
              strokeDasharray="5,5"
              className="animate-pulse"
            />
            <line
              x1="50%"
              y1="50%"
              x2="75%"
              y2="30%"
              stroke="rgba(34,211,238,0.3)"
              strokeWidth="2"
              strokeDasharray="5,5"
            />
            <line
              x1="50%"
              y1="50%"
              x2="75%"
              y2="50%"
              stroke="rgba(34,211,238,0.3)"
              strokeWidth="2"
              strokeDasharray="5,5"
            />
            <line
              x1="50%"
              y1="50%"
              x2="75%"
              y2="70%"
              stroke="rgba(34,211,238,0.3)"
              strokeWidth="2"
              strokeDasharray="5,5"
            />
          </svg>

          {/* Left Node: Local Intelligence */}
          <div className="absolute left-[25%] -translate-x-1/2 flex flex-col items-center gap-3 z-10">
            <div className="w-20 h-20 rounded-2xl bg-gradient-to-br from-purple-500 to-indigo-600 p-1 shadow-lg shadow-purple-500/20">
              <div className="w-full h-full bg-slate-900 rounded-xl flex items-center justify-center">
                <Brain className="w-10 h-10 text-purple-400" />
              </div>
            </div>
            <span className="font-mono text-xs bg-slate-800 px-3 py-1 rounded-full text-purple-300 border border-purple-500/30">
              LLM Cognition
            </span>
          </div>

          {/* Center Node: Robotics MCP Orchestrator */}
          <div className="absolute left-[50%] -translate-x-1/2 flex flex-col items-center gap-3 z-10">
            <div className="w-24 h-24 rounded-full bg-cyan-500/20 border-2 border-cyan-400 flex items-center justify-center shadow-[0_0_30px_rgba(34,211,238,0.3)]">
              <Box className="w-10 h-10 text-cyan-400" />
            </div>
            <span className="font-mono text-sm font-bold text-cyan-400 bg-cyan-950/50 px-4 py-1.5 rounded-full border border-cyan-500/50">
              Robotics MCP
            </span>
          </div>

          {/* Right Nodes: Endpoints */}
          <div className="absolute left-[75%] -translate-x-1/2 top-[30%] -translate-y-1/2 flex items-center gap-4 z-10">
            <div className="w-12 h-12 rounded-lg bg-emerald-500/10 border border-emerald-500/50 flex items-center justify-center text-emerald-400 shadow-lg shadow-emerald-500/20">
              <Gamepad2 className="w-6 h-6" />
            </div>
            <span className="font-mono text-xs text-emerald-300">Unity</span>
          </div>

          <div className="absolute left-[75%] -translate-x-1/2 top-[50%] -translate-y-1/2 flex items-center gap-4 z-10">
            <div className="w-12 h-12 rounded-lg bg-yellow-500/10 border border-yellow-500/50 flex items-center justify-center text-yellow-400 shadow-lg shadow-yellow-500/20">
              <Glasses className="w-6 h-6" />
            </div>
            <span className="font-mono text-xs text-yellow-300">Resonite</span>
          </div>

          <div className="absolute left-[75%] -translate-x-1/2 top-[70%] -translate-y-1/2 flex items-center gap-4 z-10">
            <div className="w-12 h-12 rounded-lg bg-rose-500/10 border border-rose-500/50 flex items-center justify-center text-rose-400 shadow-lg shadow-rose-500/20">
              <Video className="w-6 h-6" />
            </div>
            <span className="font-mono text-xs text-rose-300">VRChat</span>
          </div>
        </div>
      </div>
    </div>
  );
}
