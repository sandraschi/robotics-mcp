import { Activity, Cpu, Hash, Network, Terminal } from "lucide-react";

export default function ROS2Page() {
  return (
    <div className="p-8 pb-32 w-full animate-fade-in text-white min-h-screen">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold tracking-tight mb-2 flex items-center gap-3">
            <Network className="w-8 h-8 text-fuchsia-400" />
            ROS 2 Ecosystem
          </h1>
          <p className="text-gray-400">
            Node graph representation, active topics, and pub/sub metrics
            matrix.
          </p>
        </div>
        <div className="flex items-center gap-3 bg-fuchsia-500/10 text-fuchsia-400 border border-fuchsia-500/20 px-4 py-2 rounded-full">
          <span className="text-sm font-mono">DDS: FastRTPS</span>
          <span className="text-sm font-semibold border-l border-fuchsia-500/30 pl-3">
            DOMAIN_ID: 42
          </span>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-6">
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <div className="flex justify-between items-start mb-2">
            <h3 className="text-lg font-semibold text-slate-200">
              Active Nodes
            </h3>
            <Cpu className="w-5 h-5 text-fuchsia-400" />
          </div>
          <span className="text-4xl font-bold font-mono">24</span>
        </div>
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <div className="flex justify-between items-start mb-2">
            <h3 className="text-lg font-semibold text-slate-200">
              Topics Published
            </h3>
            <Hash className="w-5 h-5 text-indigo-400" />
          </div>
          <span className="text-4xl font-bold font-mono">156</span>
        </div>
        <div className="glass-card p-6 border border-white/10 rounded-xl">
          <div className="flex justify-between items-start mb-2">
            <h3 className="text-lg font-semibold text-slate-200">
              Topic Frequency (Avg)
            </h3>
            <Activity className="w-5 h-5 text-emerald-400" />
          </div>
          <span className="text-4xl font-bold font-mono">
            84.2 <span className="text-lg text-slate-500">Hz</span>
          </span>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        <div className="glass-card border border-white/10 rounded-xl overflow-hidden flex flex-col h-[500px]">
          <div className="p-4 border-b border-white/10 bg-slate-900/50 flex justify-between items-center">
            <h3 className="font-semibold flex items-center gap-2">
              <Terminal className="w-4 h-4 text-fuchsia-400" /> rclpy Bridge Log
            </h3>
          </div>
          <div className="flex-1 overflow-auto p-4 font-mono text-xs text-slate-300 space-y-2 bg-[#0a0a0f]">
            <p>
              <span className="text-emerald-400">[INFO]</span> [1708945621.124]
              [bridge]: Established ROS2 socket server on 9090
            </p>
            <p>
              <span className="text-emerald-400">[INFO]</span> [1708945621.455]
              [mcp_client]: Subscribed to /cmd_vel
            </p>
            <p>
              <span className="text-blue-400">[DEBUG]</span> [1708945622.001]
              [mcp_client]: Frame received from /camera/color/image_raw
            </p>
            <p>
              <span className="text-yellow-400">[WARN]</span> [1708945625.688]
              [nav2_controller]: Control loop missed its desired rate of 20.0Hz
            </p>
            <p>
              <span className="text-emerald-400">[INFO]</span> [1708945628.112]
              [bridge]: Client 192.168.1.5 connected
            </p>
            <p>
              <span className="text-blue-400">[DEBUG]</span> [1708945629.500]
              [robot_localization]: Executing EKF prediction step
            </p>
            <p>
              <span className="text-emerald-400">[INFO]</span> [1708945631.220]
              [slam_toolbox]: LaserScan accepted, processing map update #492
            </p>
            <div className="animate-pulse inline-block w-2 h-4 bg-slate-500 mt-2" />
          </div>
        </div>

        <div className="glass-card border border-white/10 rounded-xl overflow-hidden flex flex-col h-[500px]">
          <div className="p-4 border-b border-white/10 bg-slate-900/50 flex justify-between items-center">
            <h3 className="font-semibold flex items-center gap-2">
              <Hash className="w-4 h-4 text-indigo-400" /> Priority Topics
            </h3>
          </div>
          <div className="flex-1 overflow-auto p-0">
            <table className="w-full text-left text-sm">
              <thead className="bg-slate-900/30 text-slate-400 sticky top-0">
                <tr>
                  <th className="px-4 py-3 font-medium">Topic Name</th>
                  <th className="px-4 py-3 font-medium">Type</th>
                  <th className="px-4 py-3 font-medium text-right">Hz</th>
                </tr>
              </thead>
              <tbody className="divide-y divide-white/5 font-mono">
                {[
                  { t: "/cmd_vel", type: "geometry_msgs/Twist", hz: "15.0" },
                  { t: "/odom", type: "nav_msgs/Odometry", hz: "50.0" },
                  { t: "/scan", type: "sensor_msgs/LaserScan", hz: "10.0" },
                  { t: "/tf", type: "tf2_msgs/TFMessage", hz: "100.0" },
                  { t: "/map", type: "nav_msgs/OccupancyGrid", hz: "0.5" },
                  {
                    t: "/camera/color/image_raw",
                    type: "sensor_msgs/Image",
                    hz: "30.0",
                  },
                  { t: "/rosout", type: "rcl_interfaces/Log", hz: "-" },
                ].map((row, i) => (
                  <tr key={i} className="hover:bg-white/5 transition-colors">
                    <td className="px-4 py-3 text-fuchsia-300">{row.t}</td>
                    <td className="px-4 py-3 text-slate-400">{row.type}</td>
                    <td className="px-4 py-3 text-right text-emerald-400">
                      {row.hz}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
      </div>
    </div>
  );
}
