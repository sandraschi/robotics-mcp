import { Bot, Crosshair, Navigation, Video, Zap } from "lucide-react";
import { useEffect, useState } from "react";

interface RobotStatus {
  robot_id?: string;
  connected?: boolean;
  battery?: { voltage?: number; percentage?: number };
  position?: { x?: number; y?: number; theta?: number };
  error?: string;
}

export default function YahboomPage() {
  const [status, setStatus] = useState<RobotStatus | null>(null);
  const [connected, setConnected] = useState<boolean | null>(null);

  useEffect(() => {
    (async () => {
      try {
        const res = await fetch("/api/v1/robots/yahboom_01/status");
        if (!res.ok) {
          setConnected(false);
          return;
        }
        const data = await res.json();
        // backend returns { data: ... } or flat
        const s = (data.data ?? data) as RobotStatus;
        if (s.error) setConnected(false);
        else setConnected(s.connected ?? true);
        setStatus(s);
      } catch {
        setConnected(false);
      }
    })();
  }, []);

  const isConnected = connected === true;

  return (
    <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
            <Bot className="w-8 h-8 text-orange-400" />
            Yahboom ROS 2 Car
          </h1>
          <p className="text-gray-400">
            Mecanum wheel control, camera feed, and payload telemetry — live
            from backend.
          </p>
        </div>
        <div
          className={`flex items-center gap-3 px-4 py-2 rounded-full border ${isConnected ? "bg-emerald-500/10 text-emerald-400 border-emerald-500/20" : "bg-slate-800 text-slate-400 border-slate-700"}`}
        >
          <div
            className={`w-2 h-2 rounded-full ${isConnected ? "bg-emerald-500 animate-pulse" : "bg-slate-500"}`}
          />
          <span className="text-sm font-semibold">
            {isConnected
              ? "yahboom_01: Connected"
              : "yahboom_01: Not connected — mock removed"}
          </span>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        <div className="lg:col-span-2 glass-card border border-white/10 rounded-xl overflow-hidden flex flex-col">
          <div className="p-4 border-b border-white/10 flex justify-between items-center bg-slate-900/50">
            <h3 className="font-semibold flex items-center gap-2">
              <Video className="w-4 h-4 text-orange-400" /> Forward Chassis
              Camera
            </h3>
            <div className="flex gap-2">
              <span className="text-xs bg-slate-800 px-2 py-1 rounded text-slate-300">
                H264
              </span>
              <span
                className={`text-xs px-2 py-1 rounded ${isConnected ? "bg-emerald-900 text-emerald-300" : "bg-slate-800 text-slate-400"}`}
              >
                {isConnected ? "Live" : "Offline — no feed"}
              </span>
            </div>
          </div>
          <div className="flex-1 bg-black relative min-h-[400px] flex items-center justify-center">
            {isConnected ? (
              <div className="absolute inset-0 bg-slate-900 flex items-center justify-center">
                <span className="text-slate-400 text-sm">
                  Camera feed will appear here when yahboom_01 streams (no mock
                  image).
                </span>
              </div>
            ) : (
              <div className="absolute inset-0 bg-slate-950 flex flex-col items-center justify-center gap-3">
                <Video className="w-12 h-12 text-slate-600" />
                <p className="text-sm text-slate-500">
                  No camera feed — hardware not connected
                </p>
                <p className="text-xs text-slate-600">
                  Previously shown unsplash fake image removed.
                </p>
              </div>
            )}
            <div className="absolute inset-0 pointer-events-none p-6 opacity-30">
              <div className="w-12 h-12 border-l-2 border-t-2 border-emerald-500/30 absolute top-8 left-8" />
              <div className="w-12 h-12 border-r-2 border-t-2 border-emerald-500/30 absolute top-8 right-8" />
              <div className="w-12 h-12 border-l-2 border-b-2 border-emerald-500/30 absolute bottom-8 left-8" />
              <div className="w-12 h-12 border-r-2 border-b-2 border-emerald-500/30 absolute bottom-8 right-8" />
              <Crosshair className="w-8 h-8 text-emerald-400/20 absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2" />
            </div>
          </div>
        </div>

        <div className="flex flex-col gap-6">
          <div className="glass-card p-6 border border-white/10 rounded-xl">
            <h3 className="text-lg font-semibold mb-6 flex items-center gap-2">
              <Navigation className="w-5 h-5 text-indigo-400" /> Chassis Control
            </h3>
            <div className="flex flex-col items-center gap-2 mb-6 p-4 bg-slate-900/30 rounded-xl border border-slate-800">
              <p className="text-xs text-slate-500 font-mono text-center">
                Teleop disabled — no active Yahboom connection.
                <br />
                <span className="text-slate-600">
                  WASD overlay was mock — honest placeholder.
                </span>
              </p>
            </div>
          </div>

          <div className="glass-card p-6 border border-white/10 rounded-xl flex-1">
            <h3 className="text-lg font-semibold mb-6 flex items-center gap-2">
              <Zap className="w-5 h-5 text-yellow-400" /> Embedded Compute
            </h3>
            <div className="space-y-4">
              <div className="bg-slate-900/50 p-3 rounded-lg border border-slate-800">
                <span className="text-xs text-slate-400 block mb-1">
                  SoC Telemetry
                </span>
                {isConnected && status?.battery ? (
                  <span className="font-mono text-sm text-slate-200">
                    Battery{" "}
                    {status.battery.voltage
                      ? `${status.battery.voltage.toFixed(1)}V`
                      : status.battery.percentage
                        ? `${status.battery.percentage}%`
                        : "—"}
                  </span>
                ) : (
                  <span className="text-sm text-slate-500">
                    No telemetry — hardware offline (previously 45°C / 11.4V
                    were mocks)
                  </span>
                )}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
