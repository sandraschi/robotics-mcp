import { Battery, Bot, Loader2, MapPin, Wifi, WifiOff } from "lucide-react";
import { useEffect, useState } from "react";
import { Link as RouterLink } from "react-router-dom";
import {
  getRobotStatus,
  getRobots,
  type Robot,
  type RobotStatus,
} from "@/common/api";

function RobotCard({
  robot,
  status,
}: {
  robot: Robot;
  status: RobotStatus | null | undefined;
}) {
  const loading = status === undefined;
  const hasStatus = status && !status.error;
  const simulated = hasStatus && status.simulated === true;
  const battery = status && !status.error ? status.battery : undefined;
  const position = status && !status.error ? status.position : undefined;

  return (
    <div className="border border-white/10 rounded-xl bg-slate-900/50 backdrop-blur p-4 flex flex-col gap-3">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <Bot className="h-5 w-5 text-indigo-400 shrink-0" />
          <span className="font-medium truncate">{robot.robot_id}</span>
          {robot.is_virtual && (
            <span className="text-xs bg-slate-700 text-slate-300 px-1.5 py-0.5 rounded">
              virtual
            </span>
          )}
        </div>
        <div className="flex items-center gap-1.5">
          {loading ? (
            <Loader2 className="h-4 w-4 text-slate-400 animate-spin" />
          ) : hasStatus ? (
            simulated ? (
              <span className="text-xs text-amber-400 flex items-center gap-1">
                <WifiOff className="h-3.5 w-3.5" /> Simulated
              </span>
            ) : (
              <span className="text-xs text-emerald-400 flex items-center gap-1">
                <Wifi className="h-3.5 w-3.5" /> Connected
              </span>
            )
          ) : (
            <span className="text-xs text-slate-500">No status</span>
          )}
        </div>
      </div>
      <div className="text-xs text-slate-400">
        Type: <span className="text-slate-300">{robot.robot_type}</span>
      </div>
      {hasStatus && (
        <div className="flex flex-wrap gap-3 text-xs">
          {battery != null && (
            <span className="flex items-center gap-1 text-slate-300">
              <Battery className="h-3.5 w-3.5" />
              {typeof battery.percentage === "number"
                ? `${Math.round(battery.percentage)}%`
                : typeof battery.voltage === "number"
                  ? `${battery.voltage.toFixed(1)}V`
                  : "—"}
            </span>
          )}
          {position != null &&
            (position.x !== undefined || position.y !== undefined) && (
              <span className="flex items-center gap-1 text-slate-300">
                <MapPin className="h-3.5 w-3.5" />(
                {typeof position.x === "number" ? position.x.toFixed(2) : "—"},{" "}
                {typeof position.y === "number" ? position.y.toFixed(2) : "—"})
              </span>
            )}
        </div>
      )}
      <RouterLink
        to={
          robot.robot_type === "yahboom"
            ? "/yahboom"
            : robot.robot_type === "dreame"
              ? "/dreame"
              : "/control"
        }
        className="text-xs text-indigo-400 hover:text-indigo-300 mt-auto"
      >
        Open →
      </RouterLink>
    </div>
  );
}

export function RobotStatusDisplay() {
  const [robots, setRobots] = useState<Robot[]>([]);
  const [statusByRobot, setStatusByRobot] = useState<
    Record<string, RobotStatus | null>
  >({});
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        const list = await getRobots();
        if (cancelled) return;
        setRobots(list);
        setError(null);
        const statuses: Record<string, RobotStatus | null> = {};
        await Promise.all(
          list.map(async (r) => {
            const s = await getRobotStatus(r.robot_id);
            if (!cancelled) statuses[r.robot_id] = s;
          }),
        );
        if (!cancelled) setStatusByRobot(statuses);
      } catch (e) {
        if (!cancelled)
          setError(e instanceof Error ? e.message : "Failed to load robots");
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
      <div className="border border-white/10 rounded-xl bg-slate-900/50 backdrop-blur p-6 flex items-center gap-3">
        <Loader2 className="h-6 w-6 text-indigo-400 animate-spin" />
        <span className="text-slate-400">Loading robots…</span>
      </div>
    );
  }

  if (error) {
    return (
      <div className="border border-red-500/30 rounded-xl bg-red-950/20 p-4 text-red-300 text-sm">
        {error}
      </div>
    );
  }

  if (robots.length === 0) {
    return (
      <div className="border border-white/10 rounded-xl bg-slate-900/50 backdrop-blur p-6 text-slate-400 text-sm">
        No robots registered. Add them in{" "}
        <code className="text-slate-300 bg-slate-800 px-1 rounded">
          ~/.robotics-mcp/config.yaml
        </code>{" "}
        and restart the server.
      </div>
    );
  }

  return (
    <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-4">
      {robots.map((robot) => (
        <RobotCard
          key={robot.robot_id}
          robot={robot}
          status={statusByRobot[robot.robot_id]}
        />
      ))}
    </div>
  );
}
