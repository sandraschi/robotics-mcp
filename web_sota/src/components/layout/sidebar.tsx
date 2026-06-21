import {
  Activity,
  ChevronLeft,
  ChevronRight,
  Code2,
  Database,
  LayoutDashboard,
  RotateCcw,
  Sparkles,
} from "lucide-react";
import { Link, useLocation, useNavigate } from "react-router-dom";
import { cn } from "@/common/utils";
import { ONBOARDING_DONE_KEY } from "@/pages/onboarding";

interface SidebarProps {
  collapsed: boolean;
  onToggle: () => void;
}

export function Sidebar({ collapsed, onToggle }: SidebarProps) {
  const location = useLocation();
  const navigate = useNavigate();

  const resetOnboarding = () => {
    localStorage.removeItem(ONBOARDING_DONE_KEY);
    navigate("/onboarding", { replace: true });
  };

  const navItems = [
    { name: "Dashboard", href: "/dashboard", icon: LayoutDashboard },
    { name: "Onboarding", href: "/onboarding", icon: Sparkles },
    { name: "Live Status", href: "/status", icon: Activity },
    { name: "Yahboom ROS2", href: "/yahboom", icon: Code2 },
    { name: "Dreame D20 Pro", href: "/dreame", icon: Code2 },
    { name: "ROS 2 System", href: "/ros2", icon: Database },
    { name: "Control", href: "/control", icon: Code2 },
    { name: "Map / LiDAR", href: "/map", icon: Database },
    { name: "Tools Explorer", href: "/tools", icon: Code2 },
    { name: "VBot Chain", href: "/vbot-ecosystem", icon: Database },
    { name: "Apps Hub", href: "/apps", icon: LayoutDashboard },
    { name: "Logs", href: "/logs", icon: Activity },
  ];

  return (
    <aside
      className={cn(
        "relative flex flex-col border-r border-slate-800 bg-slate-950/50 backdrop-blur-xl transition-all duration-300 ease-in-out",
        collapsed ? "w-16" : "w-64",
      )}
    >
      <div className="flex h-16 items-center border-b border-slate-800 px-4">
        <div className="flex items-center gap-2 font-semibold text-slate-100">
          <Activity className="h-6 w-6 text-blue-500" />
          {!collapsed && (
            <span className="animate-in fade-in duration-300">Resolve-MCP</span>
          )}
        </div>
      </div>

      <nav className="flex-1 space-y-1 p-2">
        {navItems
          .filter((item) => !["/control", "/visualizer"].includes(item.href))
          .map((item) => {
            const isActive = location.pathname === item.href;
            return (
              <Link
                key={item.href}
                to={item.href}
                className={cn(
                  "group flex items-center rounded-md px-3 py-2 text-sm font-medium transition-colors hover:bg-slate-800 hover:text-white",
                  isActive ? "bg-slate-800 text-white" : "text-slate-400",
                  collapsed ? "justify-center" : "justify-start",
                )}
              >
                <item.icon
                  className={cn(
                    "h-5 w-5",
                    !collapsed && "mr-3",
                    isActive && "text-blue-400",
                  )}
                />
                {!collapsed && <span>{item.name}</span>}

                {/* Tooltip for collapsed mode */}
                {collapsed && (
                  <div className="absolute left-full ml-2 hidden rounded bg-slate-800 px-2 py-1 text-xs text-white group-hover:block z-50 whitespace-nowrap">
                    {item.name}
                  </div>
                )}
              </Link>
            );
          })}
      </nav>

      <div className="border-t border-slate-800 p-2 space-y-1">
        <button
          onClick={resetOnboarding}
          className={cn(
            "flex w-full items-center rounded-md p-2 text-slate-400 hover:bg-slate-800 hover:text-amber-400 transition-colors",
            collapsed ? "justify-center" : "justify-start gap-3",
          )}
          title="Reset onboarding"
        >
          <RotateCcw className="h-4 w-4 shrink-0" />
          {!collapsed && <span className="text-xs">Reset onboarding</span>}
        </button>
        <button
          onClick={onToggle}
          className="flex w-full items-center justify-center rounded-md p-2 text-slate-400 hover:bg-slate-800 hover:text-white transition-colors"
        >
          {collapsed ? (
            <ChevronRight className="h-5 w-5" />
          ) : (
            <div className="flex items-center w-full">
              <ChevronLeft className="h-5 w-5 mr-3" />
              <span>Collapse</span>
            </div>
          )}
        </button>
      </div>
    </aside>
  );
}
