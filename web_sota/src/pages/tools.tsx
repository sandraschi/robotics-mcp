import { PenTool, Search } from "lucide-react";

export default function ToolsExplorer() {
  return (
    <div className="p-8 pb-32 animate-fade-in text-white min-h-screen">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
            <PenTool className="w-8 h-8 text-indigo-400" />
            MCP Tools Explorer
          </h1>
          <p className="text-gray-400">
            Inspect registered robotics automation and hardware capabilities on
            the active FastMCP session.
          </p>
        </div>
      </div>

      <div className="glass-card border border-white/10 rounded-xl overflow-hidden mb-6">
        <div className="p-4 border-b border-white/10 bg-slate-900/50 flex gap-4">
          <div className="relative flex-1">
            <Search className="w-5 h-5 absolute left-3 top-1/2 -translate-y-1/2 text-slate-400" />
            <input
              type="text"
              title="Search Tools"
              placeholder="Search registered MCP tools..."
              className="w-full bg-black/50 border border-slate-700 rounded-lg pl-10 pr-4 py-2 text-sm focus:outline-none focus:border-indigo-500 transition-colors"
            />
          </div>
        </div>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-0 divide-y md:divide-y-0 md:divide-x divide-white/5">
          {[
            {
              tool: "robotics_agentic_workflow",
              dev: "Context sampling for orchestrating multi-robot workflows.",
              args: ["thought_process", "objective"],
            },
            {
              tool: "dreame_vacuum_control",
              dev: "Execute cleaning maneuvers and modify water/suction presets.",
              args: ["action", "params"],
            },
            {
              tool: "dreame_vacuum_status",
              dev: "Read live MIoT telemetry from Dreame D20 Pro.",
              args: [],
            },
            {
              tool: "robot_control_movement",
              dev: "Raw WASD/Gamepad teleop vector injector via Rosbridge.",
              args: ["robot_id", "velocities"],
            },
            {
              tool: "gazebo_fuel_models",
              dev: "Query and pull Gazebo simulation models.",
              args: ["query", "author"],
            },
            {
              tool: "vbot_crud",
              dev: "Initialize virtual bodies inside Resonite/Unity.",
              args: ["robot_id", "platform", "model"],
            },
          ].map((t, idx) => (
            <div
              key={idx}
              className="p-6 hover:bg-white/5 transition-colors border-b border-white/5"
            >
              <div className="flex justify-between items-start mb-3">
                <h3 className="font-mono text-indigo-300 font-semibold">
                  {t.tool}
                </h3>
              </div>
              <p className="text-sm text-slate-400 mb-4">{t.dev}</p>
              <div className="flex flex-wrap gap-2 mt-auto">
                {t.args.length === 0 ? (
                  <span className="text-xs font-mono px-2 py-0.5 rounded bg-slate-800 text-slate-500">
                    No Arguments
                  </span>
                ) : (
                  t.args.map((a, i) => (
                    <span
                      key={i}
                      className="text-xs font-mono px-2 py-0.5 rounded bg-indigo-500/20 text-indigo-300 border border-indigo-500/30"
                    >
                      {a}
                    </span>
                  ))
                )}
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
