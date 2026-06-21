import { Bot, Cable, CheckCircle, Settings, Wifi } from "lucide-react";
import { useState } from "react";
import { useNavigate } from "react-router-dom";

const ONBOARDING_DONE_KEY = "robotics-mcp-onboarding-done";

export default function Onboarding() {
  const [step, setStep] = useState(1);
  const navigate = useNavigate();

  const finish = () => {
    localStorage.setItem(ONBOARDING_DONE_KEY, "true");
    navigate("/dashboard", { replace: true });
  };

  return (
    <div className="min-h-screen flex flex-col items-center justify-center p-6 bg-slate-950 text-slate-50">
      <div className="w-full max-w-lg border border-white/10 rounded-2xl bg-slate-900/80 backdrop-blur-xl p-8 shadow-xl">
        {step === 1 && (
          <>
            <div className="flex justify-center mb-6">
              <Bot className="w-16 h-16 text-indigo-400" />
            </div>
            <h1 className="text-2xl font-bold text-center mb-2">
              Welcome to Robotics MCP
            </h1>
            <p className="text-slate-400 text-center mb-8">
              Unified control for physical and virtual robots: Yahboom, Dreame,
              ROS2, and more.
            </p>
            <button
              onClick={() => setStep(2)}
              className="w-full py-3 px-4 rounded-xl bg-indigo-600 hover:bg-indigo-500 text-white font-medium transition-colors"
            >
              Get started
            </button>
          </>
        )}

        {step === 2 && (
          <>
            <div className="flex justify-center gap-4 mb-6">
              <Wifi className="w-10 h-10 text-emerald-400" />
              <Cable className="w-10 h-10 text-blue-400" />
              <Settings className="w-10 h-10 text-slate-400" />
            </div>
            <h2 className="text-xl font-bold text-center mb-2">Quick setup</h2>
            <div className="text-slate-400 text-sm space-y-4 mb-6">
              <p className="font-medium text-slate-300">Connect your robot</p>
              <ul className="space-y-2 list-none">
                <li className="flex gap-2">
                  <Wifi className="w-4 h-4 text-emerald-400 shrink-0 mt-0.5" />
                  <span>
                    <strong className="text-slate-300">WiFi hotspot:</strong>{" "}
                    Connect this PC to the robot&apos;s WiFi (e.g. Raspbot: SSID{" "}
                    <code className="bg-slate-800 px-1 rounded">Raspbot</code>,
                    IP{" "}
                    <code className="bg-slate-800 px-1 rounded">
                      192.168.1.11
                    </code>
                    ).
                  </span>
                </li>
                <li className="flex gap-2">
                  <Cable className="w-4 h-4 text-blue-400 shrink-0 mt-0.5" />
                  <span>
                    <strong className="text-slate-300">Ethernet:</strong>{" "}
                    Connect the robot to your LAN. Set the robot&apos;s IP in
                    config (e.g.{" "}
                    <code className="bg-slate-800 px-1 rounded">
                      192.168.0.250
                    </code>
                    ). Ensure this PC is on the same network.
                  </span>
                </li>
              </ul>
              <p className="font-medium text-slate-300 pt-2">Config</p>
              <p>
                Edit{" "}
                <code className="text-slate-300 bg-slate-800 px-1.5 py-0.5 rounded">
                  ~/.robotics-mcp/config.yaml
                </code>{" "}
                or run{" "}
                <code className="text-slate-300 bg-slate-800 px-1.5 py-0.5 rounded">
                  Ensure-RaspbotConfig.ps1 -RobotIp &lt;IP&gt;
                </code>
                . Restart the MCP server after changing config.
              </p>
            </div>
            <div className="flex gap-3">
              <button
                onClick={() => setStep(1)}
                className="flex-1 py-3 px-4 rounded-xl border border-slate-600 text-slate-300 hover:bg-slate-800 transition-colors"
              >
                Back
              </button>
              <button
                onClick={() => setStep(3)}
                className="flex-1 py-3 px-4 rounded-xl bg-indigo-600 hover:bg-indigo-500 text-white font-medium transition-colors"
              >
                Next
              </button>
            </div>
          </>
        )}

        {step === 3 && (
          <>
            <div className="flex justify-center mb-6">
              <CheckCircle className="w-16 h-16 text-emerald-400" />
            </div>
            <h2 className="text-xl font-bold text-center mb-2">
              You&apos;re all set
            </h2>
            <p className="text-slate-400 text-center mb-8">
              Use the Dashboard and sidebar to monitor robots, run tools, and
              control hardware. You can return to this page anytime from the
              sidebar.
            </p>
            <button
              onClick={finish}
              className="w-full py-3 px-4 rounded-xl bg-emerald-600 hover:bg-emerald-500 text-white font-medium transition-colors"
            >
              Go to Dashboard
            </button>
          </>
        )}
      </div>
      <p className="mt-6 text-slate-500 text-sm">Step {step} of 3</p>
    </div>
  );
}

export { ONBOARDING_DONE_KEY };
