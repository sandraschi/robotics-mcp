import {
  Bot,
  Box,
  Brain,
  Cpu,
  Database,
  ExternalLink,
  Grid,
  Network,
} from "lucide-react";

export default function AppsHub() {
  return (
    <div className="p-8 pb-32 animate-fade-in text-white min-h-screen">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
            <Grid className="w-8 h-8 text-sky-400" />
            Apps Hub
          </h1>
          <p className="text-gray-400">
            Central directory bridging external Fleet web applications and
            Sub-Agents.
          </p>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-6">
        {[
          {
            id: "fleet",
            name: "Fleet Command",
            desc: "Global orchestration for local LLMs and port assignments.",
            port: 10702,
            icon: Network,
            color: "text-indigo-400",
          },
          {
            id: "meta",
            name: "MetaMCP",
            desc: "Central system registry management and runtime config.",
            port: 10704,
            icon: Database,
            color: "text-yellow-400",
          },
          {
            id: "robotics",
            name: "Robotics Control",
            desc: "Currently active: Hardware/ROS routing and teleop.",
            port: 10706,
            icon: Bot,
            color: "text-emerald-400",
            active: true,
          },
          {
            id: "memory",
            name: "Advanced Memory",
            desc: "Semantic persistence map and relational DB.",
            port: 10710,
            icon: Brain,
            color: "text-purple-400",
          },
          {
            id: "yahboom",
            name: "Yahboom MCP",
            desc: "Yahboom ROSmaster / ROS2 robot control webapp. Start via webapp/start.ps1.",
            port: 10792,
            icon: Bot,
            color: "text-amber-400",
          },
          {
            id: "devices",
            name: "Home Devices",
            desc: "Tapo cameras, Philips Hue, embedded IoT routing.",
            port: 10718,
            icon: Cpu,
            color: "text-orange-400",
          },
          {
            id: "blender",
            name: "Blender Studio",
            desc: "Generative 3D modeling and automated scripting.",
            port: 10722,
            icon: Box,
            color: "text-sky-400",
          },
        ].map((app) => (
          <a
            key={app.id}
            href={app.active ? "#" : `http://localhost:${app.port}`}
            target={app.active ? "_self" : "_blank"}
            rel="noreferrer"
            className={`glass-card p-6 border rounded-xl flex flex-col group transition-all duration-300 ${app.active ? "border-emerald-500/50 bg-emerald-500/5 cursor-default shadow-[0_0_15px_rgba(16,185,129,0.1)]" : "border-white/10 hover:border-sky-500/50 hover:bg-sky-500/5 cursor-pointer"}`}
          >
            <div className="flex justify-between items-start mb-4">
              <app.icon
                className={`w-8 h-8 ${app.color} transition-transform group-hover:scale-110`}
              />
              {!app.active && (
                <ExternalLink className="w-4 h-4 text-slate-500 group-hover:text-sky-400 opacity-0 group-hover:opacity-100 transition-opacity" />
              )}
              {app.active && (
                <span className="text-xs bg-emerald-500/20 text-emerald-400 px-2 py-1 rounded-full border border-emerald-500/30">
                  CURRENT
                </span>
              )}
            </div>
            <h3 className="text-lg font-bold mb-2 group-hover:text-sky-300 transition-colors">
              {app.name}
            </h3>
            <p className="text-slate-400 text-sm flex-1">{app.desc}</p>
            <div className="mt-4 pt-4 border-t border-white/5 opacity-50 font-mono text-xs flex justify-between">
              <span>localhost</span>
              <span>:{app.port}</span>
            </div>
          </a>
        ))}
      </div>
    </div>
  );
}
