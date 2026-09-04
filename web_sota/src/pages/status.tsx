import {
  Activity,
  Battery,
  Cpu,
  HardDrive,
  Loader2,
  Server,
  Wifi,
} from "lucide-react";
import { useEffect, useState } from "react";

interface StatusData {
  version: string;
  status: string;
  robots: Array<{
    robot_id: string;
    robot_type: string;
    is_virtual: boolean;
    platform?: string | null;
  }>;
  mounted_servers: string[];
  http_enabled?: boolean;
  tool_count?: number;
}

export default function StatusPage() {
  const [status, setStatus] = useState<StatusData | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        const res = await fetch("/api/v1/status");
        if (!res.ok) throw new Error(`Status ${res.status}`);
        const data = await res.json();
        if (!cancelled) setStatus(data);
      } catch (e) {
        if (!cancelled)
          setError(e instanceof Error ? e.message : "Failed to fetch status");
      } finally {
        if (!cancelled) setLoading(false);
      }
    })();
    return () => {
      cancelled = true;
    };
  }, []);

  if (loading) {
    return (
      <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen flex items-center gap-3">
        <Loader2 className="h-6 w-6 animate-spin text-indigo-400" />
        <span className="text-slate-400">Loading fleet status...</span>
      </div>
    );
  }
  if (error) {
    return (
      <div className="p-8 pb-32 w-full text-white min-h-screen">
        <div className="border border-red-500/30 rounded-xl bg-red-950/20 p-4 text-red-300 text-sm">
          {error}
        </div>
      </div>
    );
  }

  const robots = status?.robots ?? [];
  const mounted = status?.mounted_servers ?? [];
  const isHealthy = status?.status === "healthy";

  return (
    <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen">
      <div className="mb-8">
        <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
          <Activity className="w-8 h-8 text-blue-400" />
          Fleet Status
        </h1>
        <p className="text-gray-400">
          Live telemetry from the backend — no mocks.
        </p>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <div className="flex justify-between items-start mb-4">
            <Activity className="w-6 h-6 text-emerald-400" />
          </div>
          <h3 className="text-3xl font-bold mb-1">
            {isHealthy ? "Healthy" : (status?.status ?? "Unknown")}
          </h3>
          <p className="text-gray-400 text-sm">System Health</p>
          <p className="text-xs text-slate-500 mt-1">
            v{status?.version ?? "?"}
          </p>
        </div>
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <div className="flex justify-between items-start mb-4">
            <Server className="w-6 h-6 text-blue-400" />
          </div>
          <h3 className="text-3xl font-bold mb-1">{robots.length}</h3>
          <p className="text-gray-400 text-sm">Registered Robots</p>
          <p className="text-xs text-slate-500 mt-1">
            {robots.filter((r) => r.is_virtual).length} virtual
          </p>
        </div>
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <div className="flex justify-between items-start mb-4">
            <Wifi className="w-6 h-6 text-purple-400" />
          </div>
          <h3 className="text-3xl font-bold mb-1">{mounted.length}</h3>
          <p className="text-gray-400 text-sm">Mounted MCP Servers</p>
          <p className="text-xs text-slate-500 mt-1 truncate">
            {mounted.join(", ") || "none"}
          </p>
        </div>
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <div className="flex justify-between items-start mb-4">
            <Battery className="w-6 h-6 text-amber-400" />
          </div>
          <h3 className="text-3xl font-bold mb-1">
            {status?.tool_count ?? "—"}
          </h3>
          <p className="text-gray-400 text-sm">MCP Tools</p>
          <p className="text-xs text-slate-500 mt-1">port 10707</p>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <h3 className="text-lg font-semibold mb-6 flex items-center gap-2">
            <Cpu className="w-5 h-5 text-indigo-400" /> Compute Resources
          </h3>
          <div className="border border-amber-500/20 bg-amber-950/20 rounded-lg p-4 text-sm text-amber-200">
            No hardware telemetry available — connect Yahboom/Raspi 5 and expose
            metrics via{" "}
            <code className="bg-black/30 px-1 rounded">
              /api/v1/diagnostics
            </code>
            .
            <br />
            <span className="text-xs text-amber-300/70">
              Previously shown 42% CPU / 8.4GB were mocks — removed.
            </span>
          </div>
        </div>
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <h3 className="text-lg font-semibold mb-6 flex items-center gap-2">
            <HardDrive className="w-5 h-5 text-emerald-400" /> Subsystems Matrix
          </h3>
          <div className="space-y-4">
            {mounted.length === 0 ? (
              <div className="text-sm text-slate-500 border border-white/5 bg-white/5 p-3 rounded-lg">
                No mounted servers — start optional bridges (osc-mcp,
                unity3d-mcp).
              </div>
            ) : (
              mounted.map((name) => (
                <div
                  key={name}
                  className="flex items-center justify-between p-3 rounded-lg bg-white/5 border border-white/5"
                >
                  <span className="text-sm font-medium text-slate-200">
                    {name}
                  </span>
                  <span className="text-xs px-2 py-1 rounded-full bg-emerald-500/20 text-emerald-400 border border-emerald-500/30">
                    Mounted
                  </span>
                </div>
              ))
            )}
            {robots.map((r) => (
              <div
                key={r.robot_id}
                className="flex items-center justify-between p-3 rounded-lg bg-white/5 border border-white/5"
              >
                <span className="text-sm font-medium text-slate-200">
                  {r.robot_id} ({r.robot_type})
                </span>
                <span className="text-xs px-2 py-1 rounded-full bg-slate-800 text-slate-400 border border-slate-700">
                  {r.is_virtual ? "Virtual" : "Physical"}
                </span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}
