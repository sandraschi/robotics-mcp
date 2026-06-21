export interface AppLink {
  name: string;
  url: string;
  icon: string;
  description: string;
  category: "Flagship" | "MCPServer" | "Other" | "Junk";
}

export const APPS_CATALOG: AppLink[] = [
  {
    name: "MCP Studio",
    url: "http://localhost:10700",
    icon: "🦁",
    description: "Mission Control for the MCP Zoo",
    category: "Flagship",
  },
  {
    name: "Devices",
    url: "http://localhost:10714",
    icon: "🏠",
    description: "Unified Home Security platform",
    category: "Flagship",
  },
  {
    name: "Robotics",
    url: "http://localhost:10706",
    icon: "🤖",
    description: "Current Platform",
    category: "Flagship",
  },
];
