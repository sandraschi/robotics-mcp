import { Gamepad2, Settings } from "lucide-react";

export default function Control() {
  return (
    <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
            <Gamepad2 className="w-8 h-8 text-fuchsia-400" />
            Direct Control
          </h1>
          <p className="text-gray-400">
            Dispatch raw commands and manage configurations for embodied agents.
          </p>
        </div>
      </div>

      <div className="glass-card border border-white/10 p-6 rounded-xl flex items-center justify-center min-h-[400px]">
        <div className="text-center text-gray-500">
          <Settings className="w-12 h-12 mx-auto mb-4 opacity-50" />
          <p>No active robots connected for manual steering.</p>
        </div>
      </div>
    </div>
  );
}
