import { Bot, Map as MapIcon } from "lucide-react";
import { Link as RouterLink } from "react-router-dom";
import { RobotStatusDisplay } from "@/components/robot-status";

export default function Dashboard() {
  return (
    <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
            <Bot className="w-8 h-8 text-indigo-400" />
            Robotics Dashboard
          </h1>
          <p className="text-gray-400">
            Unified telemetry and control platform for connected robotics
            ecosystems.
          </p>
        </div>
      </div>

      <section className="mb-8">
        <h2 className="text-lg font-semibold mb-4 flex items-center gap-2">
          <Bot className="w-5 h-5 text-indigo-400" /> Robot status
        </h2>
        <RobotStatusDisplay />
      </section>

      <section>
        <h2 className="text-lg font-semibold mb-4">Quick links</h2>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
          <RouterLink
            to="/map"
            className="border border-white/10 rounded-xl p-6 relative overflow-hidden group block hover:border-emerald-500/50 transition-colors bg-slate-900/50 backdrop-blur"
          >
            <div className="absolute inset-0 bg-gradient-to-br from-emerald-500/10 to-transparent opacity-0 group-hover:opacity-100 transition-opacity" />
            <h3 className="text-lg font-semibold mb-2 flex items-center gap-2">
              <MapIcon className="w-5 h-5 text-emerald-400" /> Cartography
            </h3>
            <p className="text-gray-400 text-sm">
              View spatial floorplans from local appliances
            </p>
          </RouterLink>
        </div>
      </section>
    </div>
  );
}
