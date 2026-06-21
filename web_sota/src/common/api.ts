const API_BASE = "/api/v1";

export interface Robot {
  id: string;
  robot_id: string;
  robot_type: string;
  platform?: string | null;
  connected: boolean;
  is_virtual: boolean;
  metadata?: Record<string, unknown>;
}

export interface RobotStatus {
  robot_id?: string;
  model?: string;
  connected?: boolean;
  simulated?: boolean;
  battery?: { voltage?: number; percentage?: number; charging?: boolean };
  position?: { x?: number; y?: number; theta?: number };
  sensors?: Record<string, unknown>;
  capabilities?: Record<string, unknown>;
  error?: string;
  [key: string]: unknown;
}

export async function getRobots(): Promise<Robot[]> {
  const res = await fetch(`${API_BASE}/robots`);
  if (!res.ok) throw new Error(`Robots: ${res.status}`);
  const data = await res.json();
  return (data.robots ?? []).map((r: Record<string, unknown>) => ({
    id: r.robot_id ?? r.id,
    robot_id: r.robot_id ?? r.id,
    robot_type: r.robot_type ?? "unknown",
    platform: r.platform ?? null,
    connected: Boolean(r.connected),
    is_virtual: Boolean(r.is_virtual),
    metadata: (r.metadata as Record<string, unknown>) ?? {},
  }));
}

export async function getRobotStatus(
  robotId: string,
): Promise<RobotStatus | null> {
  try {
    const res = await fetch(
      `${API_BASE}/robots/${encodeURIComponent(robotId)}/status`,
    );
    if (!res.ok) return null;
    return await res.json();
  } catch {
    return null;
  }
}
