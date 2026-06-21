import { Bot, Crosshair, Navigation, Video, Zap } from "lucide-react";

export default function YahboomPage() {
  return (
    <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
            <Bot className="w-8 h-8 text-orange-400" />
            Yahboom ROS 2 Car
          </h1>
          <p className="text-gray-400">
            Mecanum wheel control, camera feed, and local 16GB Raspi 5 payload
            telemetry.
          </p>
        </div>
        <div className="flex items-center gap-3 bg-emerald-500/10 text-emerald-400 border border-emerald-500/20 px-4 py-2 rounded-full">
          <div className="w-2 h-2 rounded-full bg-emerald-500 animate-pulse" />
          <span className="text-sm font-semibold">
            sys/ros2/yahboom_01: Connected
          </span>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Camera Feed */}
        <div className="lg:col-span-2 glass-card border border-white/10 rounded-xl overflow-hidden flex flex-col">
          <div className="p-4 border-b border-white/10 flex justify-between items-center bg-slate-900/50">
            <h3 className="font-semibold flex items-center gap-2">
              <Video className="w-4 h-4 text-orange-400" /> Forward Chassis
              Camera
            </h3>
            <div className="flex gap-2">
              <span className="text-xs bg-slate-800 px-2 py-1 rounded text-slate-300">
                1080p 30fps
              </span>
              <span className="text-xs bg-slate-800 px-2 py-1 rounded text-slate-300">
                H264 (Hardware Encoded)
              </span>
            </div>
          </div>
          <div className="flex-1 bg-black relative min-h-[400px] flex items-center justify-center group">
            {/* Fake Stream Target */}
            <div className="absolute inset-0 bg-[url('https://images.unsplash.com/photo-1518770660439-4636190af475?auto=format&fit=crop&q=80')] bg-cover bg-center opacity-40 mix-blend-luminosity" />

            <div className="absolute inset-0 bg-gradient-to-t from-black/80 via-transparent to-transparent pointer-events-none" />

            {/* Overlay GUI */}
            <div className="absolute inset-0 pointer-events-none p-6">
              <div className="w-12 h-12 border-l-2 border-t-2 border-emerald-500/50 absolute top-8 left-8" />
              <div className="w-12 h-12 border-r-2 border-t-2 border-emerald-500/50 absolute top-8 right-8" />
              <div className="w-12 h-12 border-l-2 border-b-2 border-emerald-500/50 absolute bottom-8 left-8" />
              <div className="w-12 h-12 border-r-2 border-b-2 border-emerald-500/50 absolute bottom-8 right-8" />

              <Crosshair className="w-8 h-8 text-emerald-400/50 absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2" />

              <div className="absolute bottom-8 left-8 right-8 flex justify-between items-end">
                <div className="font-mono text-sm text-emerald-400 drop-shadow-md">
                  <p>YAW: 14.2°</p>
                  <p>PITCH: -2.1°</p>
                  <p>VEL: 0.0 m/s</p>
                </div>
                <div className="font-mono text-xs text-yellow-400 drop-shadow-md text-right">
                  <p>OBJECT DETECT: ACTIVE</p>
                  <p>TensorRT YOLOv8n : 42ms</p>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Hardware Controls */}
        <div className="flex flex-col gap-6">
          <div className="glass-card p-6 border border-white/10 rounded-xl">
            <h3 className="text-lg font-semibold mb-6 flex items-center gap-2">
              <Navigation className="w-5 h-5 text-indigo-400" /> Chassis Control
            </h3>

            {/* WASD Representation */}
            <div className="flex flex-col items-center gap-2 mb-6 opacity-50 pointer-events-none p-4 bg-black/20 rounded-xl">
              <div className="w-12 h-12 rounded bg-slate-800 border-b-4 border-slate-900 flex items-center justify-center font-bold">
                W
              </div>
              <div className="flex gap-2">
                <div className="w-12 h-12 rounded bg-slate-800 border-b-4 border-slate-900 flex items-center justify-center font-bold">
                  A
                </div>
                <div className="w-12 h-12 rounded bg-slate-800 border-b-4 border-slate-900 flex items-center justify-center font-bold">
                  S
                </div>
                <div className="w-12 h-12 rounded bg-slate-800 border-b-4 border-slate-900 flex items-center justify-center font-bold">
                  D
                </div>
              </div>
              <p className="text-xs text-slate-400 mt-2 font-mono">
                Teleop Mode Enabled (Focus to drive)
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
                  SoC (Raspi 5) Temp
                </span>
                <div className="flex items-center gap-3">
                  <div className="flex-1 h-1.5 bg-slate-800 rounded-full">
                    <div
                      className="h-full bg-orange-500 rounded-full"
                      style={{ width: "45%" }}
                    />
                  </div>
                  <span className="font-mono text-sm">45°C</span>
                </div>
              </div>
              <div className="bg-slate-900/50 p-3 rounded-lg border border-slate-800">
                <span className="text-xs text-slate-400 block mb-1">
                  Battery (12V Lipo)
                </span>
                <div className="flex items-center gap-3">
                  <div className="flex-1 h-1.5 bg-slate-800 rounded-full">
                    <div
                      className="h-full bg-emerald-500 rounded-full"
                      style={{ width: "78%" }}
                    />
                  </div>
                  <span className="font-mono text-sm">11.4V</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
